/************************************************************************************************/
/************************************************************************************************/
/* KERNEL-Function: ABSTRACT			parallel algorrithm in (X,U)				            */
/************************************************************************************************/
/************************************************************************************************/
__kernel void abstract(
	__global char*			XU_bag_process, 	// XU Data Main Buffer pointer shared among devices in the same process - Will be availble for READ ONLY
	__global char*			XU_bag_local, 		// XU Data Sub-Buffer pointer, a window inside th main buffer - Will be availble for READ and WRITE
	__constant char*		RO_bag				// A read-only shared data bag
) {	
	/* ABSTRACTION INFO :: WILL BE REPLACED IN FILE */
	__private concrete_t sseta[ssDim] = { @@SS_ETA@@ };
	__private concrete_t sslb[ssDim]  = { @@SS_LB@@ };
	__private concrete_t ssub[ssDim]  = { @@SS_UB@@ };
	__private concrete_t sserr[ssDim] = { @@SS_ERR@@ };	
	__private concrete_t iseta[isDim] = { @@IS_ETA@@ };
	__private concrete_t islb[isDim]  = { @@IS_LB@@ };
	__private concrete_t isub[isDim]  = { @@IS_UB@@ };
	__private concrete_t iserr[isDim] = { @@IS_ERR@@ };

	/* flat domain base and indicies */
	__private flat_t		grid_base_x[FLAT_TYPE_SIZE];
	__private flat_t		grid_base_u[FLAT_TYPE_SIZE];
	__private flat_t	 		x_flat[FLAT_TYPE_SIZE];
	__private flat_t	 		u_flat[FLAT_TYPE_SIZE];
	__private symbolic_t	universal_idx_x;
	__private symbolic_t	universal_idx_u;

	/* concrete values for post/growth computations */
	__private concrete_t xx[ssDim];
	__private concrete_t rr[ssDim];
	__private concrete_t  r[ssDim];
	__private concrete_t  x[ssDim];
	__private concrete_t  u[isDim];
	__private concrete_t cnc_dest_states_lb[ssDim];
	__private concrete_t cnc_dest_states_ub[ssDim];

	/* an index of the current thread pointing to its correct memory location*/
	__private symbolic_t flat_thread_idx;

	/* a byte to collect flags before writing to memory */
	__private char flags;

	/* getting my global position */
	universal_idx_x = UNIVERSAL_INDEX_X;
	universal_idx_u = UNIVERSAL_INDEX_Y;

	/* what is my memory position (including the case of sub-buffering) */
	flat_thread_idx = (universal_idx_u-GLOBAL_OFFSET_Y) + (universal_idx_x-GLOBAL_OFFSET_X)*PROCESS_WIDTH_Y;
	
	/* reading and seetting the flat=domain base as given by the manager code */
	for(unsigned int i=0; i<FLAT_TYPE_SIZE; i++){
		grid_base_x[i] = ((__constant ro_bag_t*)RO_bag)[0].grid_base_x[i];
		grid_base_u[i] = ((__constant ro_bag_t*)RO_bag)[0].grid_base_u[i];
	}
	flat_assign_singleton_symbolic(x_flat, &universal_idx_x);
	flat_assign_singleton_symbolic(u_flat, &universal_idx_u);
	flat_add(x_flat, x_flat, grid_base_x);
	flat_add(u_flat, u_flat, grid_base_u);

	/* converting my flat indicies to concrete-valued n-dimentional values*/
	flat_to_concrete_ss(x_flat, sslb, ssub, sseta, x);
	flat_to_concrete_is(u_flat, islb, isub, iseta, u);

#ifndef MEMORY_EFFICIENT

	/* in case we use an ODE solver. if not, we just call the post function */
#ifdef USE_ODE_SOLVER_POST
	rk4OdeSolver(xx, x, u, 'x');
#else
	post_dynamics(xx, x, u);
#endif	

	/* initializing the radius starting values for the growth bound computation*/
	for (unsigned int i = 0; i<ssDim; i++)
		r[i] = sseta[i] / 2.0f + sserr[i];

	/* in case we use an ODE solver. if not, we just call the radius function */
#ifdef USE_ODE_SOLVER_RADIUS
	rk4OdeSolver(rr, r, u, 'r');
#else
	radius_dynamics(rr, r, u);
#endif

#ifdef USE_POST_ODE_FUNCTION
	post_ODE(xx, rr, x, r, u);
#endif	

	/* computing the actual hyber-cube got from the growth bound computation */
	for (unsigned int i = 0; i<ssDim; i++) {
		cnc_dest_states_lb[i] = xx[i] - rr[i] - sserr[i];
		cnc_dest_states_ub[i] = xx[i] + rr[i] + sserr[i];
	}

	/* writing the hyper-cube to the memory bag */
	for (unsigned int i = 0; i<ssDim; i++) {
		((__global xu_bag_t*)XU_bag_local)[flat_thread_idx].cnc_dest_states_lb[i] = cnc_dest_states_lb[i];
		((__global xu_bag_t*)XU_bag_local)[flat_thread_idx].cnc_dest_states_ub[i] = cnc_dest_states_ub[i];
	}	

#endif

	/* initialize flags : all off*/
	flags = 0x00;

	/* is reachibility problem ?: set target-bit in the flags if this (x,u) has x in taget set */
#ifdef HAS_TARGET
	__private char x_in_target = is_target(x);
	if(x_in_target)
		flags |= 0x81; /*was: 0b10000001*/
	else
		flags &= 0x7E; /*was: 0b01111110*/
#endif

	/* is safety problem ?: set safe-bit in the flags if this (x,u) has x in taget set */
#ifdef HAS_SAFE
	__private char x_in_safe = is_safe(x);
	if(x_in_safe)
		flags |= 0x82; /*was: 0b10000010*/
	else
		flags &= 0x7D; /*was: 0b01111101*/
#endif

	// am i the leader of the group (x,:) ?
	if(universal_idx_u == 0){ 
		/* is reachability problem ? */
#ifdef HAS_SAFE
		if(x_in_safe)
			flags |= 0x10; /*was: 0b00010000*/ /* set isPi- = 1*/
#endif
		/* is safety problem ? */
#ifdef HAS_TARGET
		if(x_in_target)
			flags |= 0x10; /*was: 0b00010000*/ /* set isPi- = 1*/
#endif
	}

	/* oh, if this (x,u) has x to be avoided, neglect all flags and set it as avoid 0x04 */
#ifdef HAS_AVOID
	__private char x_in_avoid = is_avoid(x);
	if(x_in_avoid)
		flags = 0x04; /*was: 0b00000100*/
#endif

	/* put the flags to their position in the memory */
	((__global xu_bag_t*)XU_bag_local)[flat_thread_idx].flags = flags;

	return;
}
