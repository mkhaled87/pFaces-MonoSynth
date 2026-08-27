/* Exact predecessor-membership backends for fixed antichain targets. */

#define MEMBERSHIP_LOCAL_SIZE 64

inline void decode_grid_index(ulong flat, const int* widths, int* coordinates) {
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        coordinates[d] = (int)(flat % (ulong)widths[d]) + 1;
        flat /= (ulong)widths[d];
    }
}

__attribute__((reqd_work_group_size(MEMBERSHIP_LOCAL_SIZE, 1, 1)))
__kernel void predecessor_scan(
    __global const ulong* query_flat_indices,
    __global const ulong* query_count,
    __global const ulong* next_state_table,
    __global const uint* target_basis,
    __global const ulong* target_basis_size,
    __global uint* controlled_flags
) {
    const ulong query = (ulong)get_group_id(0);
    const uint lane = (uint)get_local_id(0);
    if (query >= query_count[0]) return;

    __local uint found[MEMBERSHIP_LOCAL_SIZE];
    found[lane] = 0;

    const ulong state = query_flat_indices[query];
    if (state < (ulong)@@TOTAL_STATES@@) {
        const ulong successor = next_state_table[state];
        if (successor != ULONG_MAX) {
            const int widths[@@STATE_DIM@@] = @@GRID_SIZES_ARRAY@@;
            int successor_coordinates[@@STATE_DIM@@];
            decode_grid_index(successor, widths, successor_coordinates);
            for (ulong i = lane; i < target_basis_size[0];
                 i += MEMBERSHIP_LOCAL_SIZE) {
                uint dominated = 1;
                for (int d = 0; d < @@STATE_DIM@@; ++d) {
                    if (successor_coordinates[d] >
                        (int)target_basis[i * (ulong)@@STATE_DIM@@ + (ulong)d]) {
                        dominated = 0;
                        break;
                    }
                }
                if (dominated) {
                    found[lane] = 1;
                    break;
                }
            }
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    for (uint offset = MEMBERSHIP_LOCAL_SIZE / 2; offset != 0; offset >>= 1) {
        if (lane < offset) found[lane] |= found[lane + offset];
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    if (lane == 0) controlled_flags[query] = found[0];
}

__kernel void predecessor_threshold(
    __global const ulong* query_flat_indices,
    __global const ulong* query_count,
    __global const ulong* next_state_table,
    __global const uint* target_threshold,
    __global uint* controlled_flags
) {
    const ulong query = (ulong)get_global_id(0);
    if (query >= query_count[0]) return;

    const ulong state = query_flat_indices[query];
    if (state >= (ulong)@@TOTAL_STATES@@) {
        controlled_flags[query] = 0;
        return;
    }
    const ulong successor = next_state_table[state];
    if (successor == ULONG_MAX) {
        controlled_flags[query] = 0;
        return;
    }

    const int widths[@@STATE_DIM@@] = @@GRID_SIZES_ARRAY@@;
    int coordinates[@@STATE_DIM@@];
    decode_grid_index(successor, widths, coordinates);
    ulong key = 0;
    ulong stride = 1;
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        if (d == @@THRESHOLD_D_STAR@@) continue;
        key += (ulong)(coordinates[d] - 1) * stride;
        stride *= (ulong)widths[d];
    }
    controlled_flags[query] =
        coordinates[@@THRESHOLD_D_STAR@@] <= (int)target_threshold[key];
}
