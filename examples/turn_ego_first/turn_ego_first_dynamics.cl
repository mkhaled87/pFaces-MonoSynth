/**
 * Turn Ego First - User Dynamics
 * 
 * System:
 *   State: [s, v, s0] (ego position, ego velocity, oncoming position)
 *   Input: [T_ego]    (ego braking/acceleration torque)
 * 
 * Goal: Cross intersection safely before oncoming vehicle
 */

// ============================================================================
// USER-DEFINED CONSTANTS
// ============================================================================

#define T_MAX 1200.0f
#define T_MIN -1800.0f
#define M_MIN 2000.0f
#define M_MAX 2250.0f
#define R_W_MIN 0.30f
#define R_W_MAX 0.35f
#define ALPHA_MIN 300.0f
#define ALPHA_MAX 350.0f
#define BETA_MIN 0.10f
#define BETA_MAX 0.25f
#define GAMMA_MIN 0.30f
#define GAMMA_MAX 0.65f
#define V_MIN 0.0f
#define V_MAX 12.0f
#define V0_MAX 12.0f
#define COLLISION_ZONE_WIDTH 10.0f

// Bounds (must match .cfg)
#define S_MIN -50.0f
#define S_MAX 10.0f
#define S0_MIN -50.0f
#define S0_MAX 10.0f


// ============================================================================
// USER-DEFINED DYNAMICS
// ============================================================================

inline int get_zone(float s) {
    if (s < -COLLISION_ZONE_WIDTH) return 1;  // before intersection
    if (s < COLLISION_ZONE_WIDTH) return 2;   // in intersection [-W, W)
    return 3;                                   // past intersection (s >= W)
}

inline bool is_safe_condition(float s_ego, float s_oncoming) {
    int ego_zone = get_zone(s_ego);
    int oncoming_zone = get_zone(s_oncoming);
    // Ego First: unsafe if oncoming cuts in front (ego_zone < oncoming_zone) 
    // or both in intersection (ego_zone == 2 && oncoming_zone == 2)
    return !((ego_zone < oncoming_zone) || (ego_zone == 2 && oncoming_zone == 2));
}

inline void get_worst_case_inputs(const float* x, float* u, float* w) {
    // MATLAB v3 convention: with priority 0, index 1 corresponds to x_max.
    // For the ego-first scenario, the safest input is maximum acceleration.
    u[0] = T_MAX; 
}

inline float compute_vehicle_accel(float v, float T) {
    float R_w = (T > 0.0f) ? R_W_MAX : R_W_MIN;
    float M = ((T / R_w - ALPHA_MAX) > 0.0f) ? M_MAX : M_MIN;
    float a = (1.0f / M) * (T / R_w - ALPHA_MAX);
    float b = -(1.0f / M_MIN) * BETA_MAX;
    float c = -(1.0f / M_MIN) * GAMMA_MAX;
    
    float dvdt = a + b * v + c * v * v;
    if (v <= 0.0f && dvdt < 0.0f) dvdt = 0.0f;
    if (v >= V_MAX && dvdt > 0.0f) dvdt = 0.0f;
    return dvdt;
}

inline void ode_rhs(const float* x, const float* u, const float* w, float* dxdt, const float* rt_params) {
    // Use runtime V0_MAX if provided (rt_params[0] > 0), else compile-time default.
    float v0_max = (rt_params[0] > 0.0f) ? rt_params[0] : V0_MAX;

    // Check if goal reached (self-loop logic)
    // Goal is Zone 3 (s > 10). 
    // But since s_max is 10, we loop at s == 10.
    if (is_safe_condition(x[0], x[2]) && x[0] >= 10.0f) {
        dxdt[0] = 0.0f;
        dxdt[1] = 0.0f;
        dxdt[2] = 0.0f;
        return;
    }

    dxdt[0] = x[1]; // ds/dt = v
    dxdt[1] = compute_vehicle_accel(x[1], u[0]);
    dxdt[2] = v0_max; // ds0/dt = v0_max (runtime parameter)
}

inline void apply_state_constraints(float* x) {
    x[1] = fmax(V_MIN, fmin(x[1], V_MAX));

    // All terminal-success coordinates are semantically equivalent.  Collapse
    // them to one favorable absorbing corner to preserve the monotone order.
    if (is_safe_condition(x[0], x[2]) && x[0] >= S_MAX) {
        x[0] = S_MAX;
        x[1] = V_MAX;
        x[2] = S0_MIN;
        return;
    }

    if (!is_safe_condition(x[0], x[2])) {
        // Map to an unsafe state (outside bounds)
        // Push toward less-safe direction for priorities (0,0,1)
        // s_ego, v_ego: max-good -> lower is worse; s_onc: min-good -> higher is worse
        x[0] = -100.0f;
        x[1] = 0.0f;
        x[2] = 100.0f;
    } else {
        // Clamping to boundaries to prevent safe paths falling off the grid
        x[0] = fmax(S_MIN, fmin(x[0], S_MAX));
        x[2] = fmax(S0_MIN, fmin(x[2], S0_MAX));
    }
}
