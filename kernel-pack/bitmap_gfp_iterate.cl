/*
 * bitmap_gfp_iterate.cl
 *
 * Direct bit-packed predecessor pass:
 *   P_k(q) = (T(q) != bot) && S_k(T(q)).
 * The host schedule lower-closes P_k and then advances S_{k+1}.
 */

inline uint bitmap_get(__global const uint* bitmap, size_t idx) {
    return (bitmap[idx >> 5] >> (uint)(idx & (size_t)31)) & 1u;
}

#if @@USE_INLINE_DYNAMICS@@
inline void bitmap_unflatten_index(size_t flat_idx, const int* dims, int* idx) {
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        idx[d] = (int)(flat_idx % (size_t)dims[d]) + 1;
        flat_idx /= (size_t)dims[d];
    }
}

inline size_t bitmap_flatten_index(const int* idx, const int* dims) {
    size_t flat = 0;
    size_t stride = 1;
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        flat += (size_t)(idx[d] - 1) * stride;
        stride *= (size_t)dims[d];
    }
    return flat;
}

inline size_t bitmap_inline_successor(size_t cell_flat,
                                      __global const float* runtime_params) {
    const float x_min[@@STATE_DIM@@] = @@X_MIN_ARRAY@@;
    const float x_max[@@STATE_DIM@@] = @@X_MAX_ARRAY@@;
    const float x_res[@@STATE_DIM@@] = @@X_RES_ARRAY@@;
    const int x_priority[@@STATE_DIM@@] = @@X_PRIORITY_ARRAY@@;
    const int N_grid[@@STATE_DIM@@] = @@GRID_SIZES_ARRAY@@;

    unsigned int x_numCells[@@STATE_DIM@@];
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        x_numCells[d] = (unsigned int)N_grid[d];
    }

    int x_idx[@@STATE_DIM@@];
    bitmap_unflatten_index(cell_flat, N_grid, x_idx);

    float x[@@STATE_DIM@@];
    idx_to_state(x_idx, x_max, x_min, x_priority, x_res, x);

    float u[@@INPUT_DIM@@], w[@@DISTURB_DIM@@];
    get_worst_case_inputs(x, u, w);

    float rt_params[4];
    for (int i = 0; i < 4; ++i) rt_params[i] = runtime_params[i];

    float x_plus[@@STATE_DIM@@];
    rk4_step(x, u, w, @@SAMPLING_TIME@@, x_plus, rt_params);

    int x_plus_idx[@@STATE_DIM@@];
    for (int d = 0; d < @@STATE_DIM@@; ++d) x_plus_idx[d] = -1;
    if (!state_to_idx(x_plus, x_min, x_max, x_res, x_priority,
                      x_numCells, x_plus_idx)) {
        return (size_t)(-1);
    }
    return bitmap_flatten_index(x_plus_idx, N_grid);
}
#endif

__kernel void bitmap_gfp_iterate(
    __global const ulong* next_state_table,
    __global const uint* bitmap_in,
    __global uint* bitmap_out,
    __global int* changed_flag,
    __global const float* runtime_params
) {
    (void)changed_flag;
    const size_t lane = get_global_id(0);
    const size_t lane_count = get_global_size(0);
    for (size_t gid_s = lane; gid_s < (size_t)(@@TOTAL_STATES@@);
         gid_s += lane_count) {

#if @@USE_INLINE_DYNAMICS@@
        const size_t succ = bitmap_inline_successor(gid_s, runtime_params);
        const uint new_safe = (succ != (size_t)(-1))
            ? bitmap_get(bitmap_in, succ) : 0u;
#else
        (void)runtime_params;
        const ulong successor = next_state_table[gid_s];
        const uint new_safe = successor != ULONG_MAX
            ? bitmap_get(bitmap_in, (size_t)successor) : 0u;
#endif

        if (new_safe) {
            atomic_or((volatile __global uint*)&bitmap_out[gid_s >> 5],
                      1u << (uint)(gid_s & (size_t)31));
        }
    }
}
