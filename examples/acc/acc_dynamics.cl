/**
 * ACC (Adaptive Cruise Control) - User Dynamics
 * 
 * System:
 *   State: [h, v_ego, v_lead]  (headway, ego velocity, lead velocity)
 *   Input: [T_ego]              (ego braking torque)
 *   Disturbance: [T_lead]       (lead vehicle braking torque)
 * 
 * Goal: Keep safe headway despite worst-case lead braking
 */

// ============================================================================
// USER-DEFINED CONSTANTS
// ============================================================================

// Vehicle parameters
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
// USER-DEFINED DYNAMICS
// ============================================================================

/**
 * Worst-case selection for ACC
 * Ego uses minimum braking (least safe), lead uses maximum braking (most dangerous)
 */
inline void get_worst_case_inputs(const float* x, float* u, float* w) {
    u[0] = T_BRAKE_MIN;  // Ego: minimum braking (worst control)
    w[0] = T_BRAKE_MAX;  // Lead: maximum braking (worst disturbance)
}

/**
 * Helper: Compute vehicle acceleration from powertrain model
 * Uses interval arithmetic for uncertain parameters (M, R_w, alpha, beta, gamma)
 */
inline float compute_vehicle_accel(float v, float T, bool is_lead) {
    const float R_w = (T > 0.0f) ? (is_lead ? R_W_MAX : R_W_MIN) : (is_lead ? R_W_MIN : R_W_MAX);
    const float M = ((T / R_w - (is_lead ? ALPHA_MAX : ALPHA_MIN)) > 0.0f) ? 
               (is_lead ? M_MAX : M_MIN) : (is_lead ? M_MIN : M_MAX);
    const float a = (1.0f / M) * (T / R_w - (is_lead ? ALPHA_MAX : ALPHA_MIN));
    const float b = -(1.0f / (is_lead ? M_MIN : M_MAX)) * (is_lead ? BETA_MAX : BETA_MIN);
    const float c = -(1.0f / (is_lead ? M_MIN : M_MAX)) * (is_lead ? GAMMA_MAX : GAMMA_MIN);
    
    float dvdt = a + b * v + c * v * v;
    if (v <= 0.0f && dvdt < 0.0f) dvdt = 0.0f;
    if (is_lead && v >= V_MAX && dvdt > 0.0f) dvdt = 0.0f;
    return dvdt;
}

/**
 * ODE right-hand side: dx/dt = f(x, u, w)
 * State: [headway, ego_velocity, lead_velocity]
 */
inline void ode_rhs(const float* x, const float* u, const float* w, float* dxdt) {
    const float a_ego = compute_vehicle_accel(x[1], u[0], false);
    const float a_lead = compute_vehicle_accel(x[2], w[0], true);
    
    dxdt[0] = x[2] - x[1];  // dh/dt = v_lead - v_ego
    dxdt[1] = a_ego;         // dv_ego/dt
    dxdt[2] = a_lead;        // dv_lead/dt
}

/**
 * Apply state constraints after integration
 */
inline void apply_state_constraints(float* x) {
    x[1] = fmax(V_MIN, fmin(x[1], V_MAX));
    x[2] = fmax(V_MIN, fmin(x[2], V_MAX));
}
