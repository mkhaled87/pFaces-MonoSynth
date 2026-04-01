/*
* build_bitmap.cl
*
*  date    : 05.03.2026
*  about   : GPU kernel for bitmap construction from basis elements.
* ***********************************************************************
*/

/**
 * Build bitmap (downward closure) from basis elements on GPU.
 *
 * Convention: 1-based internal monotone coordinates with dim 0 fastest.
 * Lower internal indices are always safer in every dimension.
 *
 * Complexity per work item: O(|B| x n) where n = state dimension.
 * Fully parallel across all grid cells.
 */

__kernel void build_bitmap(
    __global int* bitmap,
    __global const int* basis_list,
    __global const int* basis_list_size_ptr
) {
    const int flat_idx = get_global_id(0);
    if (flat_idx >= @@TOTAL_STATES@@) return;

    const int ss_dim = @@STATE_DIM@@;
    const int basis_size = *basis_list_size_ptr;
    const float x_min[@@STATE_DIM@@] = @@X_MIN_ARRAY@@;
    const float x_max[@@STATE_DIM@@] = @@X_MAX_ARRAY@@;
    const float x_res[@@STATE_DIM@@] = @@X_RES_ARRAY@@;

    unsigned int x_numCells[@@STATE_DIM@@];
    for (int i = 0; i < ss_dim; ++i) {
        x_numCells[i] = (unsigned int)ceil((x_max[i] - x_min[i]) / x_res[i]) + 1;
    }

    int cell_idx[@@STATE_DIM@@];
    int temp = flat_idx;
    for (int d = 0; d < ss_dim; ++d) {
        cell_idx[d] = (temp % x_numCells[d]) + 1;
        temp /= x_numCells[d];
    }

    int safe = 0;
    for (int b = 0; b < basis_size; ++b) {
        int dominated = 1;
        for (int d = 0; d < ss_dim; ++d) {
            int bv = basis_list[b * ss_dim + d];
            if (cell_idx[d] > bv) { dominated = 0; break; }
        }
        if (dominated) { safe = 1; break; }
    }

    bitmap[flat_idx] = safe;
}
