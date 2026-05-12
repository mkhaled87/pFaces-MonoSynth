/**
 * ACC 5D with Actuator Lag
 *
 * State:
 *   x = [h, v_ego, v_lead, T_ego_actual, T_lead_actual]
 *
 * Input / disturbance commands:
 *   u[0] = T_ego_cmd
 *   w[0] = T_lead_cmd
 *
 * Dynamics:
 *   dh/dt          = v_lead - v_ego
 *   dv_ego/dt      = A_ego(v_ego, T_ego_actual)
 *   dv_lead/dt     = A_lead(v_lead, T_lead_actual)
 *   dT_ego/dt      = (T_ego_cmd  - T_ego_actual)  / tau_ego
 *   dT_lead/dt     = (T_lead_cmd - T_lead_actual) / tau_lead
 */

#define T_MAX           1200.0f
#define T_EGO_MIN      -1800.0f
#define T_LEAD_MIN     -2400.0f

#define M_MIN           2000.0f
#define M_MAX           2500.0f
#define R_W_MIN            0.30f
#define R_W_MAX            0.35f
#define ALPHA_MIN        300.0f
#define ALPHA_MAX        350.0f
#define BETA_MIN           0.10f
#define BETA_MAX           0.25f
#define GAMMA_MIN          0.30f
#define GAMMA_MAX          0.65f

#define V_MIN              0.0f
#define V_MAX             20.0f

#define TAU_EGO            0.30f
#define TAU_LEAD           0.30f

inline void get_worst_case_inputs(const float* x, float* u, float* w) {
    (void)x;
    u[0] = T_EGO_MIN;   // least braking for ego (worst for safety)
    w[0] = T_LEAD_MIN;  // strongest braking for lead (worst for safety)
}

inline float compute_vehicle_accel(float v, float T, bool is_lead) {
    const float R_w = (T > 0.0f)
        ? (is_lead ? R_W_MAX : R_W_MIN)
        : (is_lead ? R_W_MIN : R_W_MAX);

    const float alpha = is_lead ? ALPHA_MAX : ALPHA_MIN;
    const float M = ((T / R_w - alpha) > 0.0f)
        ? (is_lead ? M_MAX : M_MIN)
        : (is_lead ? M_MIN : M_MAX);

    const float a = (1.0f / M) * (T / R_w - alpha);
    const float b = -(1.0f / (is_lead ? M_MIN : M_MAX)) * (is_lead ? BETA_MAX : BETA_MIN);
    const float c = -(1.0f / (is_lead ? M_MIN : M_MAX)) * (is_lead ? GAMMA_MAX : GAMMA_MIN);

    float dvdt = a + b * v + c * v * v;
    if (v <= 0.0f && dvdt < 0.0f) dvdt = 0.0f;
    if (v >= V_MAX && dvdt > 0.0f) dvdt = 0.0f;
    return dvdt;
}

inline void ode_rhs(const float* x, const float* u, const float* w, float* dxdt, const float* rt_params) {
    (void)rt_params;

    dxdt[0] = x[2] - x[1];
    dxdt[1] = compute_vehicle_accel(x[1], x[3], false);
    dxdt[2] = compute_vehicle_accel(x[2], x[4], true);
    dxdt[3] = (u[0] - x[3]) / TAU_EGO;
    dxdt[4] = (w[0] - x[4]) / TAU_LEAD;
}

inline void apply_state_constraints(float* x) {
    x[1] = fmax(V_MIN, fmin(x[1], V_MAX));
    x[2] = fmax(V_MIN, fmin(x[2], V_MAX));
    x[3] = fmax(T_EGO_MIN, fmin(x[3], T_MAX));
    x[4] = fmax(T_LEAD_MIN, fmin(x[4], T_MAX));
}
