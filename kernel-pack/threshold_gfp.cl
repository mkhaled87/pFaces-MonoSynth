/* Resident, double-buffered threshold GFP step. */

inline ulong threshold_key(const int* successor, const int* widths) {
    ulong key = 0;
    ulong stride = 1;
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        if (d == @@THRESHOLD_D_STAR@@) continue;
        const int coordinate = successor[d] - 1;
        if (coordinate < 0 || coordinate >= widths[d]) return ULONG_MAX;
        key += (ulong)coordinate * stride;
        stride *= (ulong)widths[d];
    }
    return key;
}

#if @@USE_INLINE_DYNAMICS@@
inline void threshold_probe_inline(
    const int* fixed_coordinates,
    uint height,
    const int* widths,
    const float* x_min,
    const float* x_max,
    const float* x_res,
    const int* x_priority,
    const uint* x_num_cells,
    const float* runtime_params,
    int* successor_height,
    ulong* successor_key
) {
    int coordinates[@@STATE_DIM@@];
    for (int d = 0; d < @@STATE_DIM@@; ++d) coordinates[d] = fixed_coordinates[d];
    coordinates[@@THRESHOLD_D_STAR@@] = (int)height;

    float state[@@STATE_DIM@@];
    idx_to_state(coordinates, x_max, x_min, x_priority, x_res, state);
    float input[@@INPUT_DIM@@], disturbance[@@DISTURB_DIM@@];
    get_worst_case_inputs(state, input, disturbance);
    float successor_state[@@STATE_DIM@@];
    rk4_step(state, input, disturbance, @@SAMPLING_TIME@@,
             successor_state, runtime_params);

    int successor[@@STATE_DIM@@];
    const bool valid = state_to_idx(successor_state, x_min, x_max, x_res,
                                    x_priority, x_num_cells, successor);
    if (!valid) {
        *successor_height = -1;
        *successor_key = ULONG_MAX;
        return;
    }
    *successor_height = successor[@@THRESHOLD_D_STAR@@];
    *successor_key = threshold_key(successor, widths);
}
#else
inline void threshold_probe_table(
    __global const ulong* next_state_table,
    ulong state,
    const int* widths,
    int* successor_height,
    ulong* successor_key
) {
    const ulong flat = next_state_table[state];
    if (flat == ULONG_MAX) {
        *successor_height = -1;
        *successor_key = ULONG_MAX;
        return;
    }
    int successor[@@STATE_DIM@@];
    ulong rest = flat;
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        successor[d] = (int)(rest % (ulong)widths[d]) + 1;
        rest /= (ulong)widths[d];
    }
    *successor_height = successor[@@THRESHOLD_D_STAR@@];
    *successor_key = threshold_key(successor, widths);
}
#endif

__kernel void threshold_gfp_step(
    __global const ulong* next_state_table,
    __global uint* threshold_a,
    __global uint* threshold_b,
    __global const uint* source_parity,
    __global uint* changed_flag,
    __global const float* runtime_params
) {
    const ulong key = (ulong)get_global_id(0);
    if (key >= (ulong)@@THRESHOLD_TABLE_SIZE@@) return;

    __global const uint* source = source_parity[0] ? threshold_b : threshold_a;
    __global uint* destination = source_parity[0] ? threshold_a : threshold_b;
    const int widths[@@STATE_DIM@@] = @@GRID_SIZES_ARRAY@@;

    int coordinates[@@STATE_DIM@@];
    ulong rest = key;
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        if (d == @@THRESHOLD_D_STAR@@) {
            coordinates[d] = 1;
        } else {
            coordinates[d] = (int)(rest % (ulong)widths[d]) + 1;
            rest /= (ulong)widths[d];
        }
    }

#if @@USE_INLINE_DYNAMICS@@
    const float x_min[@@STATE_DIM@@] = @@X_MIN_ARRAY@@;
    const float x_max[@@STATE_DIM@@] = @@X_MAX_ARRAY@@;
    const float x_res[@@STATE_DIM@@] = @@X_RES_ARRAY@@;
    const int x_priority[@@STATE_DIM@@] = @@X_PRIORITY_ARRAY@@;
    uint x_num_cells[@@STATE_DIM@@];
    for (int d = 0; d < @@STATE_DIM@@; ++d) x_num_cells[d] = (uint)widths[d];
    float parameters[4];
    for (int i = 0; i < 4; ++i) parameters[i] = runtime_params[i];
#else
    ulong full_stride[@@STATE_DIM@@];
    full_stride[0] = 1;
    for (int d = 1; d < @@STATE_DIM@@; ++d) {
        full_stride[d] = full_stride[d - 1] * (ulong)widths[d - 1];
    }
    ulong base = 0;
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        base += (ulong)(coordinates[d] - 1) * full_stride[d];
    }
#endif

    const uint old_height = source[key];
    if (old_height == 0) {
        destination[key] = 0;
        return;
    }

    int successor_height;
    ulong successor_key;
#if @@USE_INLINE_DYNAMICS@@
    threshold_probe_inline(coordinates, old_height, widths, x_min, x_max,
                           x_res, x_priority, x_num_cells, parameters,
                           &successor_height, &successor_key);
#else
    threshold_probe_table(next_state_table,
                          base + (ulong)(old_height - 1) *
                                     full_stride[@@THRESHOLD_D_STAR@@],
                          widths, &successor_height, &successor_key);
#endif
    if (successor_key != ULONG_MAX && successor_height >= 1 &&
        (uint)successor_height <= source[successor_key]) {
        destination[key] = old_height;
        return;
    }

    uint best = 0;
    uint lower = 1;
    uint upper = old_height - 1;
    while (lower <= upper) {
        const uint middle = lower + ((upper - lower) >> 1);
#if @@USE_INLINE_DYNAMICS@@
        threshold_probe_inline(coordinates, middle, widths, x_min, x_max,
                               x_res, x_priority, x_num_cells, parameters,
                               &successor_height, &successor_key);
#else
        threshold_probe_table(next_state_table,
                              base + (ulong)(middle - 1) *
                                         full_stride[@@THRESHOLD_D_STAR@@],
                              widths, &successor_height, &successor_key);
#endif
        const bool is_controlled = successor_key != ULONG_MAX &&
            successor_height >= 1 &&
            (uint)successor_height <= source[successor_key];
        if (is_controlled) {
            best = middle;
            lower = middle + 1;
        } else {
            if (middle == 0) break;
            upper = middle - 1;
        }
    }

    destination[key] = best;
    if (best != old_height) atomic_or((volatile __global uint*)changed_flag, 1u);
}
