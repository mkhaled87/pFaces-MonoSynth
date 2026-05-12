/*
* tt_only_iterate.cl
*
*  date    : 18.03.2026
*  about   : GPU kernels for threshold-table-only GFP iteration (Algorithm 4).
*
*  Kernel 1: tt_only_column_update
*    Each work item processes one column (fixed key, varying d*).
*    Binary search finds the maximum safe height f(k) per column.
*    Quick-check: first tests if the current threshold is still valid
*    (avoids full binary search for stable entries in later iterations).
*
*  Kernel 2: tt_only_prefix_max
*    Restores the non-increasing (monotonicity) property of the threshold
*    table along one key dimension at a time (one launch per dimension).
*    Each work item sweeps one "fiber" from high to low, propagating max.
*
*  The host calls: column_update -> prefix_max(dim0) -> prefix_max(dim1) -> ...
*  then reads changed_flag to check convergence.
* ***********************************************************************
*/

/* ----------------------------------------------------------------
 * Helper: compute the successor's key-flat index from full-space
 * successor indices. Returns -1 if any key dimension is out of bounds.
 * ---------------------------------------------------------------- */
inline int compute_succ_key_flat(const int* succ_idx,
                                 const int* N,
                                 int d_star,
                                 int ss_dim) {
    int key_flat = 0, stride = 1;
    for (int d = 0; d < ss_dim; ++d) {
        if (d == d_star) continue;
        int c = succ_idx[d] - 1;
        if (c < 0 || c >= N[d]) return -1;
        key_flat += c * stride;
        stride *= N[d];
    }
    return key_flat;
}

#if @@USE_INLINE_DYNAMICS@@
/* ----------------------------------------------------------------
 * Inline-dynamics probe: integrate ODE from grid index, return
 * the successor's d* coordinate and key-flat index.
 * Sets succ_d_star = -1 on failure (OOB or invalid transition).
 * ---------------------------------------------------------------- */
inline void probe_dynamics(const int* orig_idx, int d_star_val,
                           const float* x_min, const float* x_max,
                           const float* x_res, const int* x_priority,
                           const unsigned int* x_numCells, const int* N,
                           int d_star, int ss_dim,
                           const float* rt_params,
                           int* succ_d_star, int* succ_key_flat) {
    int probe_idx[@@STATE_DIM@@];
    for (int d = 0; d < ss_dim; ++d) probe_idx[d] = orig_idx[d];
    probe_idx[d_star] = d_star_val;

    float x[@@STATE_DIM@@];
    idx_to_state(probe_idx, x_max, x_min, x_priority, x_res, x);

    float u[@@INPUT_DIM@@], w[@@DISTURB_DIM@@];
    get_worst_case_inputs(x, u, w);

    float x_plus[@@STATE_DIM@@];
    rk4_step(x, u, w, @@SAMPLING_TIME@@, x_plus, rt_params);

    int x_plus_idx[@@STATE_DIM@@];
    bool valid = state_to_idx(x_plus, x_min, x_max, x_res,
                              x_priority, x_numCells, x_plus_idx);

    *succ_d_star = valid ? x_plus_idx[d_star] : -1;
    if (*succ_d_star < 1 || *succ_d_star > N[d_star]) {
        *succ_d_star = -1;
        *succ_key_flat = -1;
        return;
    }
    *succ_key_flat = compute_succ_key_flat(x_plus_idx, N, d_star, ss_dim);
}
#else
/* ----------------------------------------------------------------
 * Table-lookup probe: read successor from precomputed table, return
 * the successor's d* coordinate and key-flat index.
 * ---------------------------------------------------------------- */
inline void probe_table(__global const unsigned int* next_state_table,
                        int cell_flat, int ss_dim, int d_star,
                        const int* N,
                        int* succ_d_star, int* succ_key_flat) {
    unsigned int flat_succ = next_state_table[cell_flat];
    if (flat_succ == 0xFFFFFFFFu) {
        *succ_d_star = -1;
        *succ_key_flat = -1;
        return;
    }
    int succ_idx[@@STATE_DIM@@];
    {
        unsigned int tmp = flat_succ;
        for (int d = 0; d < ss_dim; ++d) {
            succ_idx[d] = (int)(tmp % (unsigned int)N[d]) + 1;
            tmp /= (unsigned int)N[d];
        }
    }
    *succ_d_star = succ_idx[d_star];
    if (*succ_d_star < 1 || *succ_d_star > N[d_star]) {
        *succ_d_star = -1;
        *succ_key_flat = -1;
        return;
    }
    *succ_key_flat = compute_succ_key_flat(succ_idx, N, d_star, ss_dim);
}
#endif /* USE_INLINE_DYNAMICS */

/* ================================================================
 * Kernel 1: Column Update (binary search per column)
 *   NDRange: table_size work items (one per column/key)
 * ================================================================ */
__kernel void tt_only_column_update(
    __global const unsigned int* next_state_table,
    __global const int* threshold_table_in,
    __global int* threshold_table_out,
    __global int* changed_flag,
    __global const float* runtime_params
) {
    const int key_flat = get_global_id(0);
    const int ss_dim = @@STATE_DIM@@;
    const int d_star = @@THRESHOLD_D_STAR@@;
    const int table_size = @@THRESHOLD_TABLE_SIZE@@;

    if (key_flat >= table_size) return;

    const int N[@@STATE_DIM@@] = @@GRID_SIZES_ARRAY@@;

    /* Unflatten key_flat -> 1-based original indices (d_star = 1 placeholder) */
    int orig_idx[@@STATE_DIM@@];
    {
        int tmp = key_flat;
        for (int d = 0; d < ss_dim; ++d) {
            if (d == d_star) { orig_idx[d] = 1; continue; }
            orig_idx[d] = (tmp % N[d]) + 1;
            tmp /= N[d];
        }
    }

    int tau_old = threshold_table_in[key_flat];
    if (tau_old == 0) {
        threshold_table_out[key_flat] = 0;
        return;
    }

    /* Setup for probes (mode-dependent constants) */
#if @@USE_INLINE_DYNAMICS@@
    const float x_min[@@STATE_DIM@@]    = @@X_MIN_ARRAY@@;
    const float x_max[@@STATE_DIM@@]    = @@X_MAX_ARRAY@@;
    const float x_res[@@STATE_DIM@@]    = @@X_RES_ARRAY@@;
    const int   x_priority[@@STATE_DIM@@] = @@X_PRIORITY_ARRAY@@;
    unsigned int x_numCells[@@STATE_DIM@@];
    for (int d = 0; d < ss_dim; ++d) x_numCells[d] = (unsigned int)N[d];
    float rt_params[4];
    for (int i = 0; i < 4; ++i) rt_params[i] = runtime_params[i];
#else
    /* Strides for converting d* coordinate -> flat cell index */
    int full_stride[@@STATE_DIM@@];
    full_stride[0] = 1;
    for (int d = 1; d < ss_dim; ++d) full_stride[d] = full_stride[d-1] * N[d-1];
    int base_flat = 0;
    for (int d = 0; d < ss_dim; ++d) base_flat += (orig_idx[d]-1) * full_stride[d];
    const int d_star_stride = full_stride[d_star];
#endif

    /* --- Quick-check: is tau_old still safe? ---
     * In later iterations most entries are stable; testing tau_old first
     * saves ~log2(N_d*) probe evaluations per stable entry. */
    {
        int sd, skf;
#if @@USE_INLINE_DYNAMICS@@
        probe_dynamics(orig_idx, tau_old, x_min, x_max, x_res, x_priority,
                       x_numCells, N, d_star, ss_dim, rt_params, &sd, &skf);
#else
        probe_table(next_state_table, base_flat + (tau_old-1)*d_star_stride,
                    ss_dim, d_star, N, &sd, &skf);
#endif
        if (sd >= 1 && skf >= 0 && sd <= threshold_table_in[skf]) {
            threshold_table_out[key_flat] = tau_old;
            return;
        }
    }

    /* --- Binary search for new (lower) threshold --- */
    int f = 0, lo = 1, hi = tau_old - 1;
    while (lo <= hi) {
        int mid = (lo + hi) >> 1;
        int sd, skf;
#if @@USE_INLINE_DYNAMICS@@
        probe_dynamics(orig_idx, mid, x_min, x_max, x_res, x_priority,
                       x_numCells, N, d_star, ss_dim, rt_params, &sd, &skf);
#else
        probe_table(next_state_table, base_flat + (mid-1)*d_star_stride,
                    ss_dim, d_star, N, &sd, &skf);
#endif
        if (sd < 1 || skf < 0) {
            hi = mid - 1;
        } else if (sd <= threshold_table_in[skf]) {
            f = mid;
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }

    threshold_table_out[key_flat] = f;
    if (f < tau_old) atomic_or(&changed_flag[0], 1);
}

/* ================================================================
 * Kernel 2: Prefix-Max Sweep (one key dimension per launch)
 *   Restores the non-increasing (monotonicity) property of the
 *   threshold table along one key dimension.
 *
 *   NDRange: table_size / N_k work items (one per fiber).
 *   Each fiber sweeps from high to low coordinate, propagating max.
 *
 *   Runtime parameters via sweep_params buffer:
 *     sweep_params[0] = stride_k  (stride of sweep dim in key-flat space)
 *     sweep_params[1] = N_k       (size of sweep dimension)
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
