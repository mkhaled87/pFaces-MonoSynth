/*
* check_basis_safety.cl
*
*  date    : 20.01.2026
*  about   : Safety check kernel for monotone synthesis.
* ***********************************************************************
*/

/**
 * Checks if each basis element remains safe after one worst-case transition.
 *
 * Convention: 1-based internal monotone coordinates.
 * Lower internal indices are always safer in every dimension.
 */

kernel void check_basis_safety(
    __global const int* basis_flat_idx,
    __global const int* next_state_table,
    __global const int* basis_list,
    __global const int* basis_list_size_ptr,
    __global int* unsafe_flags
) {
    const int basis_idx = get_global_id(0);
    const int basis_list_size = basis_list_size_ptr[0];
    if (basis_idx >= basis_list_size) return;

    const int flat_idx = basis_flat_idx[basis_idx];
    if (flat_idx < 0 || flat_idx >= @@TOTAL_STATES@@) {
        unsafe_flags[basis_idx] = 1;
        return;
    }

    int next_state[@@STATE_DIM@@];
    for (int i = 0; i < @@STATE_DIM@@; ++i) {
        next_state[i] = next_state_table[flat_idx * @@STATE_DIM@@ + i];
    }

    if (next_state[0] == -1) {
        unsafe_flags[basis_idx] = 1;
        return;
    }

    int unsafe = 1;
    for (int i = 0; i < basis_list_size; ++i) {
        int dominated = 1;
        for (int j = 0; j < @@STATE_DIM@@; ++j) {
            int b = basis_list[i * @@STATE_DIM@@ + j];
            if (next_state[j] > b) { dominated = 0; break; }
        }
        if (dominated) {
            unsafe = 0;
            break;
        }
    }

    unsafe_flags[basis_idx] = unsafe;
}
