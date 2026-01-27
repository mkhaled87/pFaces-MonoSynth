// Safety Check Kernel - Monotone Synthesis
// Checks if each basis element remains safe after one transition.
// Requires: SS_DIM (state dimension), TOTAL_STATES (grid size)

kernel void check_basis_safety(
    __global const int* basis_flat_idx,        // Flattened indices of basis elements
    __global const int* next_state_table,      // Precomputed transitions (total_states * SS_DIM)
    __global const int* basis_list,            // Basis element coordinates (basis_size * SS_DIM)
    __global const int* basis_list_size_ptr,   // Pointer to number of basis elements
    __global int* unsafe_flags                 // Output: 1 if unsafe, 0 otherwise
) {
    const int basis_idx = get_global_id(0);
    const int basis_list_size = basis_list_size_ptr[0];
    if (basis_idx >= basis_list_size) return;

    // Get flat index and validate
    const int flat_idx = basis_flat_idx[basis_idx];
    if (flat_idx < 0 || flat_idx >= TOTAL_STATES) {
        unsafe_flags[basis_idx] = 1;
        return;
    }

    // Look up next state from transition table
    int next_state[SS_DIM];
    for (int i = 0; i < SS_DIM; ++i) {
        next_state[i] = next_state_table[flat_idx * SS_DIM + i];
    }

    // Check if next state is out-of-bounds (-1 signals OOB)
    if (next_state[0] == -1) {
        unsafe_flags[basis_idx] = 1;
        return;
    }

    // Check if next state is dominated by ANY current basis element
    // Dominated means basis[i][j] <= next_state[j] for all j
    int unsafe = 1;
    for (int i = 0; i < basis_list_size; ++i) {
        int dominated = 1;
        for (int j = 0; j < SS_DIM; ++j) {
            if (next_state[j] > basis_list[i * SS_DIM + j]) {
                dominated = 0;
                break;
            }
        }
        if (dominated) {
            unsafe = 0;
            break;
        }
    }

    unsafe_flags[basis_idx] = unsafe;
}