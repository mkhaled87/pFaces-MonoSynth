/************************************************************************************************/
/************************************************************************************************/
/* KERNEL-Function: CHECK_SAFE_SET    Test kernel for safe set checking                        */
/************************************************************************************************/
/************************************************************************************************/

// Bag structure for read-only parameters
typedef struct {
    int state_dim;          // Number of state dimensions (2 or 3)
    int max_basis_size;     // Maximum number of basis elements
} ro_params_t;

/************************************************************************************************/
/* TEST KERNEL: Simple read/write test                                                          */
/* Each thread:                                                                                 */
/*   1. Reads its basis element from BASIS_bag                                                  */
/*   2. Writes it to UNSAFE_bag (dummy test - marks all as unsafe)                             */
/*   3. Increments unsafe_count atomically                                                      */
/************************************************************************************************/
__kernel void check_safe_set(
	__global int*			BASIS_bag,      // Input: safe_set_basis [basis_size * state_dim]
	__global int*			UNSAFE_bag,     // Output: unsafe indices [basis_size * state_dim]
	__global int*			COUNTERS_bag,   // [0]=basis_size, [1]=unsafe_count
	__constant ro_params_t*	RO_params       // Read-only parameters
) {	
	// Get my thread ID
	int thread_id = get_global_id(0);
	
	// Read parameters
	int state_dim = RO_params->state_dim;
	int basis_size = COUNTERS_bag[0];
	
	// Boundary check: only process valid basis elements
	if (thread_id >= basis_size) {
		return;
	}
	
	// Read my basis element from BASIS_bag
	int my_basis_idx[3];  // Max 3 dimensions
	for (int j = 0; j < state_dim; ++j) {
		my_basis_idx[j] = BASIS_bag[thread_id * state_dim + j];
	}
	
	// DUMMY TEST: Mark all elements as "unsafe" and write to UNSAFE_bag
	// In real implementation, this would check transition safety
	bool is_unsafe = true;  // Dummy: all marked unsafe for test
	
	if (is_unsafe) {
		// Write my basis indices to UNSAFE_bag at my thread position
		for (int j = 0; j < state_dim; ++j) {
			UNSAFE_bag[thread_id * state_dim + j] = my_basis_idx[j];
		}
		
		// Atomically increment unsafe_count
		atomic_inc(&COUNTERS_bag[1]);
	}
	
	// Optional: Write a test value to verify memory access
	// We can add a flag or marker here for validation
	
	return;
}
