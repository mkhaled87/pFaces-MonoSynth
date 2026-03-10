/**
 * @file scenarios/turn_ego_first.h
 * @brief Turn Ego First scenario — ego crosses intersection before oncoming.
 *
 * 3D state: [s_ego, v_ego, s_oncoming]
 * 1D input: [T_ego] (torque)
 *
 * Matches turn_ego_first_dynamics.cl exactly.
 * v_oncoming is a SENSED parameter (not a state), updated at runtime.
 */

#pragma once

#include "../dynamics.h"
#include "../safe_set.h"

#include <cmath>

namespace rt_ctrl {

class TurnEgoFirstDynamics : public DynamicsModel {
public:
    // Vehicle parameters (from turn_ego_first_dynamics.cl constants)
    double T_MAX  = 1200.0,  T_MIN  = -1800.0;
    double M_MIN  = 2000.0,  M_MAX  = 2250.0;
    double R_W_MIN = 0.30,   R_W_MAX = 0.35;
    double ALPHA_MIN = 300.0, ALPHA_MAX = 350.0;
    double BETA_MIN  = 0.10,  BETA_MAX  = 0.25;
    double GAMMA_MIN = 0.30,  GAMMA_MAX = 0.65;
    double V_MIN = 0.0,      V_MAX = 12.0;
    double COLLISION_ZONE_WIDTH = 10.0;
    double S_MAX = 10.0;

    // Sensed oncoming velocity — updated at runtime
    double v_oncoming = 12.0;  // default: worst-case V0_MAX

    // -----------------------------------------------------------------------
    // Metadata
    // -----------------------------------------------------------------------
    std::string name() const override { return "turn_ego_first"; }
    std::vector<std::string> state_names() const override {
        return {"s_ego", "v_ego", "s_onc"};
    }
    std::vector<std::string> input_names() const override {
        return {"T_ego"};
    }
    std::string measured_param_name() const override { return "v_oncoming"; }
    std::string synthesis_macro() const override { return "V0_MAX"; }

    // -----------------------------------------------------------------------
    // Dimensions
    // -----------------------------------------------------------------------
    int nx() const override { return 3; }
    int nu() const override { return 1; }
    int ny() const override { return 3; }

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    void set_param(const std::string& name, double value) override {
        if (name == "v_oncoming" || name == "V0_MAX") v_oncoming = value;
        else if (name == "T_MAX")  T_MAX  = value;
        else if (name == "T_MIN")  T_MIN  = value;
        else if (name == "M_MIN")  M_MIN  = value;
        else if (name == "M_MAX")  M_MAX  = value;
        else if (name == "V_MAX")  V_MAX  = value;
        else if (name == "COLLISION_ZONE_WIDTH") COLLISION_ZONE_WIDTH = value;
    }

    // -----------------------------------------------------------------------
    // Zone logic (matches .cl exactly)
    // -----------------------------------------------------------------------
    int get_zone(double s) const {
        if (s < -COLLISION_ZONE_WIDTH) return 1;
        if (std::fabs(s) <= COLLISION_ZONE_WIDTH) return 2;
        return 3;
    }

    bool is_safe_condition(double s_ego, double s_oncoming) const {
        int ez = get_zone(s_ego);
        int oz = get_zone(s_oncoming);
        // Ego First: unsafe if ego behind oncoming (ez < oz) or both in zone 2
        return !((ez < oz) || (ez == 2 && oz == 2));
    }

    // -----------------------------------------------------------------------
    // Vehicle acceleration (matches .cl exactly — ego worst-case corners)
    // -----------------------------------------------------------------------
    double compute_vehicle_accel(double v, double T) const {
        double R_w = (T > 0.0) ? R_W_MAX : R_W_MIN;
        double M   = ((T / R_w - ALPHA_MAX) > 0.0) ? M_MAX : M_MIN;
        double a   = (1.0 / M) * (T / R_w - ALPHA_MAX);
        double b   = -(1.0 / M_MIN) * BETA_MAX;
        double c   = -(1.0 / M_MIN) * GAMMA_MAX;

        double dvdt = a + b * v + c * v * v;
        if (v <= 0.0 && dvdt < 0.0) dvdt = 0.0;
        if (v >= V_MAX && dvdt > 0.0) dvdt = 0.0;
        return dvdt;
    }

    // -----------------------------------------------------------------------
    // ODE: dx/dt = f(x, u)
    // -----------------------------------------------------------------------
    void ode(const double* x, const double* u, double* dx) const override {
        double s_ego = x[0], v_ego = x[1], s_onc = x[2];

        // Self-loop at goal (ego cleared intersection while safe)
        if (is_safe_condition(s_ego, s_onc) && s_ego >= S_MAX) {
            dx[0] = dx[1] = dx[2] = 0.0;
            return;
        }

        dx[0] = v_ego;                                  // ds/dt = v
        dx[1] = compute_vehicle_accel(v_ego, u[0]);     // dv/dt = accel
        dx[2] = v_oncoming;                              // ds0/dt = sensed v_onc
    }

    // -----------------------------------------------------------------------
    // Cost function
    // -----------------------------------------------------------------------
    CostParams default_cost_params() const override {
        return {
            {"w_progress",   1.0},
            {"w_safety",     1e6},
            {"w_terminal_v", 0.0},
            {"w_jerk",       0.0},
            {"goal",         10.0},
            {"unsafe_goal",  -10.0}
        };
    }

    MppiObjective make_objective(const SafeSet* ss,
                                 const CostParams& p) const override
    {
        const double w_prog = cparam(p, "w_progress",   1.0);
        const double w_safe = cparam(p, "w_safety",     1e6);
        const double w_tv   = cparam(p, "w_terminal_v", 0.0);
        const double w_jerk = cparam(p, "w_jerk",       0.0);
        const double goal   = cparam(p, "goal",         10.0);
        const double ugol   = cparam(p, "unsafe_goal",  -10.0);
        const int    nxd    = nx();
        const int    nud    = nu();

        return [=](const Eigen::MatrixXd& x, const Eigen::MatrixXd& /*y*/,
                   const Eigen::MatrixXd& u, const double& /*slack*/) -> double
        {
            double cost = 0.0;
            const int H = static_cast<int>(x.rows()) - 1;

            // Check initial state for safe/unsafe goal selection
            double s0[MAX_DIM];
            row_to_buf(x, 0, nxd, s0);
            bool safe0 = ss->is_safe(s0);
            double g = safe0 ? goal : ugol;

            for (int k = 0; k <= H; ++k) {
                // Progress: distance-squared to goal along dim 0
                double ds = x(k, 0) - g;
                cost += w_prog * ds * ds;

                // Jerk: consecutive control differences
                if (k > 0 && k < H && w_jerk > 0.0) {
                    for (int j = 0; j < nud; ++j) {
                        double du = u(k, j) - u(k - 1, j);
                        cost += w_jerk * du * du;
                    }
                }

                // Safety penalty
                double sk[MAX_DIM];
                row_to_buf(x, k, nxd, sk);
                if (!ss->is_safe(sk)) cost += w_safe;
            }

            // Terminal velocity penalty (unsafe only — encourage braking)
            if (!safe0 && w_tv > 0.0) {
                double vH = x(H, 1);
                cost += w_tv * vH * vH;
            }

            return cost;
        };
    }
};

}  // namespace rt_ctrl
