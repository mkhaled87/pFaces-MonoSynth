/**
 * @file scenarios/acc.h
 * @brief Adaptive Cruise Control (ACC) scenario.
 *
 * 3D state: [h, v_ego, v_lead]  (headway, ego velocity, lead velocity)
 * 1D input: [T_ego]             (ego braking torque, range [-1, 1])
 *
 * Matches acc_dynamics.cl exactly.
 * No runtime-varying measured parameter — synthesis happens once at startup.
 * Lead vehicle brakes at worst case (T_BRAKE_MAX) in both synthesis and MPPI.
 */

#pragma once

#include "../dynamics.h"
#include "../safe_set.h"

#include <cmath>

namespace rt_ctrl {

class AccDynamics : public DynamicsModel {
public:
    // Vehicle parameters (from acc_dynamics.cl constants)
    double T_MAX       = 1200.0;
    double T_BRAKE_MIN = -1800.0;   // ego worst-case braking
    double T_BRAKE_MAX = -2400.0;   // lead worst-case braking
    double M_MIN       = 2000.0,  M_MAX  = 2500.0;
    double R_W_MIN     = 0.30,   R_W_MAX = 0.35;
    double ALPHA_MIN   = 300.0, ALPHA_MAX = 350.0;
    double BETA_MIN    = 0.10,  BETA_MAX  = 0.25;
    double GAMMA_MIN   = 0.30,  GAMMA_MAX = 0.65;
    double V_MIN       = 0.0,   V_MAX     = 20.0;

    // -----------------------------------------------------------------------
    // Metadata
    // -----------------------------------------------------------------------
    std::string name() const override { return "acc"; }
    std::vector<std::string> state_names() const override {
        return {"h", "v_ego", "v_lead"};
    }
    std::vector<std::string> input_names() const override {
        return {"T_ego"};
    }
    // No runtime measured parameter — fixed worst-case disturbance
    std::string measured_param_name() const override { return ""; }
    std::string synthesis_macro() const override { return ""; }

    // -----------------------------------------------------------------------
    // Dimensions
    // -----------------------------------------------------------------------
    int nx() const override { return 3; }
    int nu() const override { return 1; }
    int ny() const override { return 3; }

    // -----------------------------------------------------------------------
    // Parameters (for overriding vehicle constants from JSON if needed)
    // -----------------------------------------------------------------------
    void set_param(const std::string& name, double value) override {
        if      (name == "T_MAX")       T_MAX       = value;
        else if (name == "T_BRAKE_MIN") T_BRAKE_MIN = value;
        else if (name == "T_BRAKE_MAX") T_BRAKE_MAX = value;
        else if (name == "M_MIN")       M_MIN       = value;
        else if (name == "M_MAX")       M_MAX       = value;
        else if (name == "V_MAX")       V_MAX       = value;
    }

    // -----------------------------------------------------------------------
    // Vehicle acceleration (matches acc_dynamics.cl exactly)
    //
    // Uses interval arithmetic for uncertain parameters.
    // is_lead selects opposite parameter corners for worst-case.
    // -----------------------------------------------------------------------
    double compute_vehicle_accel(double v, double T, bool is_lead) const {
        double R_w = (T > 0.0) ? (is_lead ? R_W_MAX : R_W_MIN)
                                : (is_lead ? R_W_MIN : R_W_MAX);
        double alpha_ref = is_lead ? ALPHA_MAX : ALPHA_MIN;
        double M = ((T / R_w - alpha_ref) > 0.0)
                     ? (is_lead ? M_MAX : M_MIN)
                     : (is_lead ? M_MIN : M_MAX);
        double a = (1.0 / M) * (T / R_w - alpha_ref);
        double M_drag = is_lead ? M_MIN : M_MAX;
        double b = -(1.0 / M_drag) * (is_lead ? BETA_MAX  : BETA_MIN);
        double c = -(1.0 / M_drag) * (is_lead ? GAMMA_MAX : GAMMA_MIN);

        double dvdt = a + b * v + c * v * v;
        if (v <= 0.0 && dvdt < 0.0) dvdt = 0.0;
        if (is_lead && v >= V_MAX && dvdt > 0.0) dvdt = 0.0;
        return dvdt;
    }

    // -----------------------------------------------------------------------
    // ODE: dx/dt = f(x, u)
    //
    // Lead brakes at worst case (T_BRAKE_MAX) — matches synthesis assumption.
    // -----------------------------------------------------------------------
    void ode(const double* x, const double* u, double* dx) const override {
        double h = x[0], v_ego = x[1], v_lead = x[2];

        double a_ego  = compute_vehicle_accel(v_ego,  u[0],        false);
        double a_lead = compute_vehicle_accel(v_lead, T_BRAKE_MAX, true);

        dx[0] = v_lead - v_ego;   // dh/dt  = v_lead - v_ego
        dx[1] = a_ego;            // dv_ego/dt
        dx[2] = a_lead;           // dv_lead/dt (worst-case braking)
    }

    // -----------------------------------------------------------------------
    // Cost function
    //
    // Tracks a reference headway and speed while penalizing unsafe states
    // and control effort. No goal/unsafe_goal switching — ACC maintains
    // safe following indefinitely.
    // -----------------------------------------------------------------------
    CostParams default_cost_params() const override {
        return {
            {"w_headway", 1.0},     // weight: headway tracking
            {"w_speed",   0.5},     // weight: velocity tracking
            {"w_safety",  1e4},     // penalty per unsafe predicted state
            {"w_comfort", 0.1},     // weight: control effort (comfort)
            {"w_jerk",    0.01},    // weight: input smoothing
            {"h_ref",     40.0},    // desired headway (m)
            {"v_ref",     15.0}     // desired ego speed (m/s)
        };
    }

    MppiObjective make_objective(const SafeSet* ss,
                                 const CostParams& p) const override
    {
        const double w_h    = cparam(p, "w_headway", 1.0);
        const double w_v    = cparam(p, "w_speed",   0.5);
        const double w_safe = cparam(p, "w_safety",  1e4);
        const double w_com  = cparam(p, "w_comfort", 0.1);
        const double w_jerk = cparam(p, "w_jerk",    0.01);
        const double h_ref  = cparam(p, "h_ref",     40.0);
        const double v_ref  = cparam(p, "v_ref",     15.0);
        const int    nxd    = nx();
        const int    nud    = nu();

        return [=](const Eigen::MatrixXd& x, const Eigen::MatrixXd& /*y*/,
                   const Eigen::MatrixXd& u, const double& /*slack*/) -> double
        {
            double cost = 0.0;
            const int H = static_cast<int>(x.rows()) - 1;

            for (int k = 0; k <= H; ++k) {
                // Headway tracking
                double dh = x(k, 0) - h_ref;
                cost += w_h * dh * dh;

                // Speed tracking
                double dv = x(k, 1) - v_ref;
                cost += w_v * dv * dv;

                // Safety penalty
                double sk[MAX_DIM];
                row_to_buf(x, k, nxd, sk);
                if (!ss->is_safe(sk)) cost += w_safe;
            }

            // Control effort (comfort) + jerk
            for (int k = 0; k < H; ++k) {
                for (int j = 0; j < nud; ++j) {
                    cost += w_com * u(k, j) * u(k, j);
                    if (k > 0 && w_jerk > 0.0) {
                        double du = u(k, j) - u(k - 1, j);
                        cost += w_jerk * du * du;
                    }
                }
            }

            return cost;
        };
    }
};

}  // namespace rt_ctrl
