/*
* tt_only_iterate.cl
*
*  date    : 18.03.2026
*  about   : GPU kernels for threshold-table-only GFP iteration (Algorithm 4).
*
*  Kernel 1: tt_only_column_update
*    Each work item processes one column (fixed key, varying d*).
*    Binary search finds the maximum safe height f(k) per column.
*    Writes raw f(k) to threshold_table_out. Sets changed_flag atomically.
*
*  Kernel 2: tt_only_prefix_max
*    Restores the non-increasing (monotonicity) property of the threshold table
*    along one key dimension at a time (called once per key dimension).
*    Each work item handles one "fiber" -- a 1D slice along the sweep dimension.
*    Sweeps from high coordinate to low, propagating max values downward.
*
*  Together these two kernels implement one full TT-only iteration.
*  The host calls: column_update -> prefix_max(dim0) -> prefix_max(dim1) -> ...
*  then reads changed_flag to check convergence.
* ***********************************************************************
*/

/* ================================================================
 * Kernel 1: Column Update (binary search per column)
 *   NDRange: table_size work items
 *
 *   Two modes (compile-time):
 *     USE_INLINE_DYNAMICS=0: read successor from precomputed next_state_table
 *     USE_INLINE_DYNAMICS=1: compute ODE dynamics inline per probe (no table)
 * ================================================================ */
__kernel void tt_only_column_update(
    __global const int* next_state_table,      /* T[flat * n + d] (unused when inline) */
    __global const int* threshold_table_in,    /* current thresholds (read) */
    __global int* threshold_table_out,         /* raw f(k) per column (write) */
    __global int* changed_flag,                /* set to 1 if any column shrinks */
    __global const float* runtime_params       /* ODE runtime params (inline mode) */
) {
    const int key_flat = get_global_id(0);
    const int ss_dim = @@STATE_DIM@@;
    const int d_star = @@THRESHOLD_D_STAR@@;
    const int table_size = @@THRESHOLD_TABLE_SIZE@@;

    if (key_flat >= table_size) return;

    /* Grid sizes (compile-time arrays) */
    const int N[@@STATE_DIM@@] = @@GRID_SIZES_ARRAY@@;

    /* Unflatten key_flat to 1-based original indices (d_star = 1 placeholder) */
    int orig_idx[@@STATE_DIM@@];
    int temp = key_flat;
    for (int d = 0; d < ss_dim; ++d) {
        if (d == d_star) { orig_idx[d] = 1; continue; }
        orig_idx[d] = (temp % N[d]) + 1;
        temp /= N[d];
    }

    /* Full-space strides (dim 0 fastest, column-major) */
    int full_stride[@@STATE_DIM@@];
    full_stride[0] = 1;
    for (int d = 1; d < ss_dim; ++d)
        full_stride[d] = full_stride[d - 1] * N[d - 1];

    /* Base flat index with d_star coordinate = 1 */
    int base_flat = 0;
    for (int d = 0; d < ss_dim; ++d)
        base_flat += (orig_idx[d] - 1) * full_stride[d];
    const int d_star_stride = full_stride[d_star];

    int tau_old = threshold_table_in[key_flat];
    if (tau_old == 0) {
        threshold_table_out[key_flat] = 0;
        return;
    }

#if @@USE_INLINE_DYNAMICS@@
    /* --- Inline dynamics mode: compute transitions on-the-fly --- */
    const float x_min[@@STATE_DIM@@] = @@X_MIN_ARRAY@@;
    const float x_max[@@STATE_DIM@@] = @@X_MAX_ARRAY@@;
    const float x_res[@@STATE_DIM@@] = @@X_RES_ARRAY@@;
    const int x_priority[@@STATE_DIM@@] = @@X_PRIORITY_ARRAY@@;
    unsigned int x_numCells[@@STATE_DIM@@];
    for (int d = 0; d < ss_dim; ++d)
        x_numCells[d] = (unsigned int)N[d];

    float rt_params[4];
    rt_params[0] = runtime_params[0];
    rt_params[1] = runtime_params[1];
    rt_params[2] = runtime_params[2];
    rt_params[3] = runtime_params[3];

    int f = 0, lo = 1, hi = tau_old;
    while (lo <= hi) {
        int mid = (lo + hi) >> 1;

        /* Build full index with d_star = mid */
        int probe_idx[@@STATE_DIM@@];
        for (int d = 0; d < ss_dim; ++d)
            probe_idx[d] = orig_idx[d];
        probe_idx[d_star] = mid;

        /* Convert to continuous state */
        float x[@@STATE_DIM@@];
        idx_to_state(probe_idx, x_max, x_min, x_priority, x_res, x);

        /* Compute worst-case inputs */
        float u[@@INPUT_DIM@@], w[@@DISTURB_DIM@@];
        get_worst_case_inputs(x, u, w);

        /* Integrate ODE */
        float x_plus[@@STATE_DIM@@];
        rk4_step(x, u, w, @@SAMPLING_TIME@@, x_plus, rt_params);

        /* Convert successor to grid index */
        int x_plus_idx[@@STATE_DIM@@];
        bool valid = state_to_idx(x_plus, x_min, x_max, x_res, x_priority, x_numCells, x_plus_idx);

        int next_d_star = valid ? x_plus_idx[d_star] : -1;
        if (next_d_star == -1 || next_d_star <= 0 || next_d_star > N[d_star]) {
            hi = mid - 1;
            continue;
        }

        /* Compute successor key flat index */
        int succ_key_flat = 0;
        int oob = 0;
        int succ_stride = 1;
        for (int d = 0; d < ss_dim; ++d) {
            if (d == d_star) continue;
            int c = (valid ? x_plus_idx[d] : -1) - 1;
            if (c < 0 || c >= N[d]) { oob = 1; break; }
            succ_key_flat += c * succ_stride;
            succ_stride *= N[d];
        }

        if (oob) {
            hi = mid - 1;
            continue;
        }

        if (next_d_star <= threshold_table_in[succ_key_flat]) {
            f = mid;
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
#else
    /* --- Table lookup mode: read from precomputed next_state_table --- */
    int f = 0, lo = 1, hi = tau_old;
    while (lo <= hi) {
        int mid = (lo + hi) >> 1;
        int cell_flat = base_flat + (mid - 1) * d_star_stride;
        const int base_off = cell_flat * ss_dim;

        /* Check for invalid transition (T = bot) */
        int next_d_star = next_state_table[base_off + d_star];
        if (next_d_star == -1) {
            hi = mid - 1;
            continue;
        }

        /* Compute successor key flat index */
        int succ_key_flat = 0;
        int oob = 0;
        int succ_stride = 1;
        for (int d = 0; d < ss_dim; ++d) {
            if (d == d_star) continue;
            int c = next_state_table[base_off + d] - 1;
            if (c < 0 || c >= N[d]) { oob = 1; break; }
            succ_key_flat += c * succ_stride;
            succ_stride *= N[d];
        }

        if (oob || next_d_star <= 0 || next_d_star > N[d_star]) {
            hi = mid - 1;
            continue;
        }

        if (next_d_star <= threshold_table_in[succ_key_flat]) {
            f = mid;
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
#endif /* USE_INLINE_DYNAMICS */

    threshold_table_out[key_flat] = f;
    if (f < tau_old) {
        atomic_or(&changed_flag[0], 1);
    }
}

/* ================================================================
 * Kernel 2: Prefix-Max Sweep (ALL key dimensions in one launch)
 *   Restores non-increasing property along each key dimension.
 *
 *   Compile-time parameters:
 *     @@NUM_KEY_DIMS@@         = n-1 (number of key dimensions)
 *     @@KEY_STRIDES_ARRAY@@   = {stride_k0, stride_k1, ...} in key-flat space
 *     @@KEY_SIZES_ARRAY@@     = {N_k0, N_k1, ...}
 *
 *   NDRange: table_size work items.
 *   Each work item processes one entry. For each key dimension, it
 *   checks if it's NOT at the highest coordinate and propagates
 *   the max from the next-higher neighbor.
 *
 *   Multiple passes are needed per dimension (N_k - 1 passes each).
 *   Since we can't do global barriers in OpenCL, we use a single
 *   kernel launch per key dimension instead.
 *
 *   REVISED: sweep_params are runtime parameters, one launch per dim.
 *   Parameters passed via sweep_params buffer:
 *     sweep_params[0] = stride_k  (stride of sweep dimension in key-flat space)
 *     sweep_params[1] = N_k       (size of sweep dimension)
 *
 *   NDRange: table_size / N_k work items (one per fiber)
 *   Each fiber sweeps from coordinate N_k-1 down to 0, propagating max.
 * ================================================================ */
__kernel void tt_only_prefix_max(
    __global int* threshold_table,    /* in-place sweep */
    __global const int* sweep_params  /* [stride_k, N_k] */
) {
    const int fiber_id = get_global_id(0);
    const int table_size = @@THRESHOLD_TABLE_SIZE@@;
    const int stride_k = sweep_params[0];
    const int N_k = sweep_params[1];
    const int num_fibers = table_size / N_k;

    if (fiber_id >= num_fibers) return;

    /* Decompose fiber_id into (outer_block, inner_offset) */
    /* The key-flat layout for dimension with stride_k and size N_k: */
    /*   idx = outer_block * (stride_k * N_k) + coord * stride_k + inner_offset */
    const int block = stride_k * N_k;
    const int outer_block = fiber_id / stride_k;
    const int inner_offset = fiber_id % stride_k;
    const int base = outer_block * block + inner_offset;

    /* Sweep from high to low coordinate, propagating max */
    for (int c = N_k - 2; c >= 0; --c) {
        const int idx_lo = base + c * stride_k;
        const int idx_hi = idx_lo + stride_k;
        int val_hi = threshold_table[idx_hi];
        if (val_hi > threshold_table[idx_lo])
            threshold_table[idx_lo] = val_hi;
    }
}
