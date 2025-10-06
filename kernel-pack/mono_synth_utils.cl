/* Half-precision is a feauture allowed by some GPUs - make sure to check for its support first.*/
/* This directive enables half-precision for storage with floats as base for openCl functions.*/
/* In case you want to use this, send the define USE_HALF_FLOATS in while compiling this kernel.*/
#ifdef USE_HALF_FLOATS
	#pragma OPENCL EXTENSION cl_khr_fp16 : enable
#endif

/* change this to control the upper bound on posts showing an error if number of posts went beyond it */
#define MAX_POSTS_COUNT @@MAX_POSTS_COUNT@@

/* this is the bigInt base type */
#define FLAT_TYPE @@FLAT_DATA_TYPE@@

/* this is the length of bigInt array */
#define FLAT_TYPE_SIZE @@BIGINT_SIZE@@

/* those are the used data types to represent flat (flattned deimentional), symbolic (multi dimentional integers) and concrete (real) values */
#define concrete_t @@CONCRETE_DATA_TYPE@@
#define symbolic_t @@SYMBOLIC_DATA_TYPE@@
#define flat_t FLAT_TYPE

/* the sampling period TAU used to solve the ODE in case we use an ODE solver */
#define SAMPLING_PERIOD @@SAMPLING_PERIOD@@

/* the diemnsions of the state space and input spcae */
#define ssDim @@SSDIM@@
#define isDim @@ISDIM@@

/* some flags to instruct the OpenCL compiler */
@@DEFINE_USE_DOUBLE_PRECISION@@

/* indicatiors for whether we use ODE solvers ior not + whether to use code only or not*/
@@DEFINE_USE_ODE_SOLVER_POST@@
@@DEFINE_CODE_ONLY_SOLVER_POST@@
@@DEFINE_USE_ODE_SOLVER_RADIUS@@
@@DEFINE_CODE_ONLY_SOLVER_RADIUS@@

/* indicatiors for whether this is a reacchability problem of a safety problem */
@@DEFINE_HAS_TARGET@@
#define TARGET_COUNT @@TARGET_COUNT@@
@@DEFINE_HAS_SAFE@@
#define SAFE_COUNT @@SAFE_COUNT@@
@@DEFINE_HAS_AVOID@@
#define AVOID_COUNT @@AVOID_COUNT@@

#ifdef HAS_TARGET
	#ifdef HAS_SAFE
		#error "This kernel works either for safety specification or reachibility specification not both at the same time"
	#endif
#endif

#ifndef HAS_TARGET
	#ifndef HAS_SAFE
		#error "This kernel works either for safety specification or reachibility specification. You need to specify one (i.e., a target set or a safe set)."
	#endif
#endif

#include "pfaces.cl"
@@EXTRA_INCLUDE@@

/* the dynamics post function used by the ODE solver */
/* do we use halfs with floats as base ? */
#ifdef USE_HALF_FLOATS
void post_dynamics(concrete_t* xxold, concrete_t* xold, concrete_t* uold) {
	float xx[ssDim], x[ssDim], u[isDim];
	@@POST_DYNAMICS_INIT_CODE@@
	for (int i = 0; i < ssDim; i++) x[i] = vload_half(0, &xold[i]);
	for (int i = 0; i < isDim; i++) u[i] = vload_half(0, &uold[i]);
#else
void post_dynamics(concrete_t* xx, concrete_t* x, concrete_t* u) {
	@@POST_DYNAMICS_INIT_CODE@@
#endif
#ifndef USE_CODE_ONLY_POST
	@@POST_DYNAMICS@@
#endif
#ifdef USE_HALF_FLOATS
	for (int i = 0; i < ssDim; i++) vstore_half(xx[i], 0, &xxold[i]);
#endif
	@@POST_DYNAMICS_FINISH_CODE@@
}

/* the dynamics growth bound used to compute the OARS */
/* do we use halfs with floats as base ? */
#ifdef USE_HALF_FLOATS
void radius_dynamics(concrete_t* rrold, concrete_t* rold, concrete_t* uold) {	
	float rr[ssDim], r[ssDim], u[isDim];
	@@RADIUS_DYNAMICS_INIT_CODE@@
	for (int i = 0; i < ssDim; i++) r[i] = vload_half(0, &rold[i]);
	for (int i = 0; i < isDim; i++) u[i] = vload_half(0, &uold[i]);
#else
void radius_dynamics(concrete_t* rr, concrete_t* r, concrete_t* u) {
	@@RADIUS_DYNAMICS_INIT_CODE@@
#endif	
#ifndef USE_CODE_ONLY_RADIUS
	@@RADIUS_DYNAMICS@@
#endif
#ifdef USE_HALF_FLOATS
	for (int i = 0; i < ssDim; i++) vstore_half(rr[i], 0, &rrold[i]);
#endif
	@@RADIUS_DYNAMICS_FINISH_CODE@@
}

/* in clude the RUNG-KUTTA SOLVER*/
#include "rk4ode.cl"

/* in case this is a reachability problem */
#ifdef HAS_TARGET
char is_target(concrete_t* x) {
	concrete_t  target[TARGET_COUNT][ssDim][2] = { @@TARGET_DATA@@};
	for (unsigned int k = 0; k < TARGET_COUNT; k++) {
		for (unsigned int i = 0; i < ssDim; i++) {
			if (x[i]<target[k][i][0] || x[i]>target[k][i][1]) {
				return 0;
			}
		}
	}
	return 1;
}
#endif

/* in case this is a safety problem */
#ifdef HAS_SAFE
char is_safe(concrete_t* x) {
	concrete_t  safe[SAFE_COUNT][ssDim][2] = { @@SAFE_DATA@@};
	for (unsigned int k = 0; k < SAFE_COUNT; k++) {
		for (unsigned int i = 0; i < ssDim; i++) {
			if (x[i]<safe[k][i][0] || x[i]>safe[k][i][1]) {
					return 0;
			}
		}
	}
	return 1;
}
#endif

/* in case some parts of the state space need to be avoided/excluded from the analysis */
#ifdef HAS_AVOID
char is_avoid(concrete_t* x) {
	concrete_t  avoid[AVOID_COUNT][ssDim][2] = { @@AVOID_DATA@@};
	for (unsigned int k = 0; k < AVOID_COUNT; k++) {
		char belongs_to_this_avoid = 1;
		for (unsigned int i = 0; i < ssDim; i++) {
			if (x[i]<avoid[k][i][0] || x[i]>avoid[k][i][1]) {
				belongs_to_this_avoid = 0;
			}
		}
		if (belongs_to_this_avoid == 1)
			return 1;
	}
	return 0;
}
#endif

/* the bag that holds information for abstraction and synthesis */
typedef struct __attribute__((packed)) xu_bag {
#ifndef MEMORY_EFFICIENT
	/* Transition cone*/
	concrete_t  cnc_dest_states_lb[ssDim];
	concrete_t  cnc_dest_states_ub[ssDim];
#endif
	char	flags;		/*	mult-ipurpose 8-bits flags:		 GENERAL:	| is_Z(k) | is_PiZ+ | is_Z(k-1) | is_PiZ- | +/- | isX_A | isX_S | isX_T | */
						/*								REACHABILITY:                                                           | FPchk | is_C  | */
						/*								      SAFETY:                                                                   | FPchk | */
#ifdef PFACES_USE_MPI
	symbolic_t num_req_posts;
	uint posts_req_resp[MAX_POSTS_COUNT/(sizeof(uint) * 8)];
#endif
} xu_bag_t;

/* a memory bag used for constant memor (RO: Read Only)*/
typedef struct __attribute__((packed)) ro_bag {
	flat_t		grid_base_x[FLAT_TYPE_SIZE];
	flat_t		grid_base_u[FLAT_TYPE_SIZE];
} ro_bag_t;

