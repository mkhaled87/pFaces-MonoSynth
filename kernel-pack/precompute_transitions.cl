/**
 * Generic N-Dimensional Worst-Case Transition Kernel (pFaces version)
 * 
 * HOW TO USE:
 * 1. Configure your problem dimensions and parameters (Section 1)
 * 2. Implement your dynamics functions (Section 2)
 * 3. Done! Everything else is automatic.
 * 
 * Adapted from standalone version - uses float instead of double
 */

// ============================================================================
// SECTION 1: PROBLEM CONFIGURATION (USER EDITS THIS)
// ============================================================================

// Dimensions
#define STATE_DIM 3              // Number of state variables
#define INPUT_DIM 1              // Number of control inputs
#define DISTURB_DIM 1            // Number of disturbances

// Solver configuration
#define ODE_STEPS 1000           // RK4 integration steps
#define SAMPLING_TIME 0.4f       // Time step for integration

// State space bounds and discretization (pFaces style)
#define H_MIN_3D 0.0f
#define H_MAX_3D 80.0f
#define V_MIN_3D 0.0f
#define V_MAX_3D 20.0f
#define H_RES_3D 0.8f
#define V_RES_3D 0.4f

// Priority directions (0 = max-priority, 1 = min-priority)
#define X_PRIORITY_0 0
#define X_PRIORITY_1 1
#define X_PRIORITY_2 0

// Vehicle parameters (example - replace with your parameters)
#define T_MAX 1200.0f
#define T_BRAKE_MIN -1800.0f
#define T_BRAKE_MAX -2400.0f
#define M_MIN 2000.0f
#define M_MAX 2500.0f
#define R_W_MIN 0.30f
#define R_W_MAX 0.35f
#define ALPHA_MIN 300.0f
#define ALPHA_MAX 350.0f
#define BETA_MIN 0.10f
#define BETA_MAX 0.25f
#define GAMMA_MIN 0.30f
#define GAMMA_MAX 0.65f
#define V_MIN 0.0f
#define V_MAX 20.0f

// ============================================================================
// SECTION 2: USER DYNAMICS (USER IMPLEMENTS THIS)
// ============================================================================

/**
 * REQUIRED: Compute worst-case inputs/disturbances
 * 
 * Given current state x, determine worst-case control u and disturbance w.
 * 
 * @param x[STATE_DIM]: Current state
 * @param u[INPUT_DIM]: Output - worst-case input
 * @param w[DISTURB_DIM]: Output - worst-case disturbance
 */
inline void get_worst_case_inputs(const float* x, float* u, float* w) {
    // Example: Vehicle following - minimum ego torque, maximum lead braking
    u[0] = T_BRAKE_MIN;
    w[0] = T_BRAKE_MAX;
}

/**
 * Helper: Compute vehicle acceleration (example helper function)
 */
inline float compute_vehicle_accel(float v, float T, bool is_lead) {
    float R_w = (T > 0.0f) ? (is_lead ? R_W_MAX : R_W_MIN) : (is_lead ? R_W_MIN : R_W_MAX);
    float M = ((T / R_w - (is_lead ? ALPHA_MAX : ALPHA_MIN)) > 0.0f) ? 
               (is_lead ? M_MAX : M_MIN) : (is_lead ? M_MIN : M_MAX);
    float a = (1.0f / M) * (T / R_w - (is_lead ? ALPHA_MAX : ALPHA_MIN));
    float b = -(1.0f / (is_lead ? M_MIN : M_MAX)) * (is_lead ? BETA_MAX : BETA_MIN);
    float c = -(1.0f / (is_lead ? M_MIN : M_MAX)) * (is_lead ? GAMMA_MAX : GAMMA_MIN);
    
    float dvdt = a + b * v + c * v * v;
    if (v <= V_MIN_3D && dvdt < 0.0f) dvdt = 0.0f;
    if (is_lead && v >= V_MAX_3D && dvdt > 0.0f) dvdt = 0.0f;
    return dvdt;
}

/**
 * REQUIRED: ODE right-hand side dx/dt = f(x, u, w)
 * 
 * Compute state derivatives for RK4 solver.
 * 
 * @param x[STATE_DIM]: Current state
 * @param u[INPUT_DIM]: Control input
 * @param w[DISTURB_DIM]: Disturbance
 * @param dxdt[STATE_DIM]: Output - state derivatives
 */
inline void ode_rhs(const float* x, const float* u, const float* w, float* dxdt) {
    // Example: 3D vehicle following
    // State: [headway, ego_velocity, lead_velocity]
    
    float a_ego = compute_vehicle_accel(x[1], u[0], false);
    float a_lead = compute_vehicle_accel(x[2], w[0], true);
    
    dxdt[0] = x[2] - x[1];           // dh/dt = v_lead - v_ego
    dxdt[1] = a_ego;                 // dv_ego/dt
    dxdt[2] = a_lead;                // dv_lead/dt
}

/**
 * OPTIONAL: Apply state constraints
 * 
 * Clamp or constrain states after integration (e.g., velocity limits).
 * Default implementation does nothing.
 * 
 * @param x[STATE_DIM]: State to constrain (modified in place)
 */
inline void apply_state_constraints(float* x) {
    // Example: Velocity bounds for vehicle dynamics
    if (STATE_DIM >= 2) x[1] = fmax(V_MIN, fmin(x[1], V_MAX));
    if (STATE_DIM >= 3) x[2] = fmax(V_MIN, fmin(x[2], V_MAX));
    
    // For other problems, modify or remove constraints as needed
}

// ============================================================================
// SECTION 3: GENERIC ODE SOLVER (DON'T EDIT)
// ============================================================================

/**
 * Generic 4th-order Runge-Kutta solver with fixed step size
 */
inline void rk4_step(const float* x, const float* u, const float* w,
                     float dt, float* x_plus) {
    const float h = dt / ODE_STEPS;
    float x_curr[STATE_DIM];
    float k1[STATE_DIM], k2[STATE_DIM], k3[STATE_DIM], k4[STATE_DIM];
    float x_temp[STATE_DIM];
    
    // Initialize
    for (int i = 0; i < STATE_DIM; ++i) x_curr[i] = x[i];
    
    // RK4 integration loop
    for (int step = 0; step < ODE_STEPS; ++step) {
        // k1 = f(x)
        ode_rhs(x_curr, u, w, k1);
        
        // k2 = f(x + h*k1/2)
        for (int i = 0; i < STATE_DIM; ++i) 
            x_temp[i] = x_curr[i] + h * 0.5f * k1[i];
        ode_rhs(x_temp, u, w, k2);
        
        // k3 = f(x + h*k2/2)
        for (int i = 0; i < STATE_DIM; ++i)
            x_temp[i] = x_curr[i] + h * 0.5f * k2[i];
        ode_rhs(x_temp, u, w, k3);
        
        // k4 = f(x + h*k3)
        for (int i = 0; i < STATE_DIM; ++i)
            x_temp[i] = x_curr[i] + h * k3[i];
        ode_rhs(x_temp, u, w, k4);
        
        // x_next = x + h/6 * (k1 + 2*k2 + 2*k3 + k4)
        for (int i = 0; i < STATE_DIM; ++i)
            x_curr[i] += (h / 6.0f) * (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]);
    }
    
    // Copy result and apply constraints
    for (int i = 0; i < STATE_DIM; ++i) {
        x_plus[i] = x_curr[i];
    }
    apply_state_constraints(x_plus);
}

// ============================================================================
// SECTION 4: GENERIC INDEX MAPPING (DON'T EDIT)
// ============================================================================

/**
 * Convert flat index to N-dimensional grid indices
 */
inline void unflatten_index(int flat_idx, const unsigned int* dims, int* idx) {
    for (int i = 0; i < STATE_DIM; ++i) {
        idx[i] = (flat_idx % dims[i]) + 1;
        flat_idx /= dims[i];
    }
}

/**
 * Convert grid indices to continuous state values
 */
inline void idx_to_state(const int* idx,
                         const float* x_min,
                         const float* x_max,
                         const float* x_res,
                         const int* x_priority,
                         float* x) {
    for (int i = 0; i < STATE_DIM; ++i) {
        x[i] = (x_priority[i] == 1)
               ? x_min[i] + (idx[i] - 1) * x_res[i]
               : x_max[i] - (idx[i] - 1) * x_res[i];
    }
}

/**
 * Convert continuous state to grid indices (with bounds checking)
 */
inline bool state_to_idx(const float* x,
                         const float* x_min,
                         const float* x_max,
                         const float* x_res,
                         const int* x_priority,
                         const unsigned int* x_numCells,
                         int* idx) {
    const float tol = 1e-2f;
    
    for (int i = 0; i < STATE_DIM; ++i) {
        // Check bounds
        if (x[i] < x_min[i] - tol || x[i] > x_max[i] + tol) {
            for (int j = 0; j < STATE_DIM; ++j) idx[j] = -1;
            return false;
        }
        
        // Clamp and compute index
        float v = fmax(x_min[i], fmin(x[i], x_max[i]));
        float q = (x_priority[i] == 1)
                   ? (v - x_min[i]) / x_res[i] - 1e-5f
                   : (x_max[i] - v) / x_res[i] - 1e-5f;
        idx[i] = max(1, min((int)ceil(q) + 1, (int)x_numCells[i]));
    }
    return true;
}

// ============================================================================
// SECTION 5: MAIN KERNEL (DON'T EDIT - pFaces specific)
// ============================================================================

/**
 * Main kernel: Precompute worst-case transitions for all grid states
 * pFaces version - single parameter, reads config from defines
 */
__kernel void precompute_transitions(
    __global unsigned int* next_state_table
) {
    int gid = get_global_id(0);
    
    // State space configuration (from Section 1 defines)
    const float x_min[STATE_DIM] = {H_MIN_3D, V_MIN_3D, V_MIN_3D};
    const float x_max[STATE_DIM] = {H_MAX_3D, V_MAX_3D, V_MAX_3D};
    const float x_res[STATE_DIM] = {H_RES_3D, V_RES_3D, V_RES_3D};
    const int x_priority[STATE_DIM] = {X_PRIORITY_0, X_PRIORITY_1, X_PRIORITY_2};
    
    // Compute grid dimensions
    unsigned int x_numCells[STATE_DIM];
    for (int i = 0; i < STATE_DIM; ++i) {
        x_numCells[i] = (unsigned int)ceil((x_max[i] - x_min[i]) / x_res[i]) + 1;
    }
    
    // Current state indices
    int x_idx[STATE_DIM];
    unflatten_index(gid, x_numCells, x_idx);
    
    // Current state values
    float x[STATE_DIM];
    idx_to_state(x_idx, x_min, x_max, x_res, x_priority, x);
    
    // Worst-case inputs
    float u[INPUT_DIM], w[DISTURB_DIM];
    get_worst_case_inputs(x, u, w);
    
    // Next state via RK4 integration
    float x_plus[STATE_DIM];
    rk4_step(x, u, w, SAMPLING_TIME, x_plus);
    
    // Next state indices (with bounds checking)
    int x_plus_idx[STATE_DIM];
    for (int i = 0; i < STATE_DIM; ++i) x_plus_idx[i] = -1;
    state_to_idx(x_plus, x_min, x_max, x_res, x_priority, x_numCells, x_plus_idx);
    
    // Write result (pFaces format: flat_idx * STATE_DIM + dimension)
    int base = gid * STATE_DIM;
    for (int i = 0; i < STATE_DIM; ++i) {
        next_state_table[base + i] = (unsigned int)x_plus_idx[i];
    }
}
