/**
 * @file scenarios/two_oncoming.h
 * @brief Two Oncoming Vehicles scenario — ego crosses gap between two
 *        independent oncoming vehicles with independently varying velocities.
 *
 * 4D state: [s_ego, v_ego, s_onc1, s_onc2]
 *   s_ego  — ego position along its path
 *   v_ego  — ego velocity
 *   s_onc1 — position of vehicle 1 (closer, ego WAITS for it to pass)
 *   s_onc2 — position of vehicle 2 (farther, ego crosses BEFORE it arrives)
 *
 * 1D input: [T_ego] (torque)
 *
 * Synthesis is DECOMPOSED into two independent 3D sub-problems:
 *   P_wait:  (s_ego, v_ego, s_onc1) via turn_oncoming_first (ego yields)
 *   P_go:    (s_ego, v_ego, s_onc2) via turn_ego_first      (ego goes first)
 *
 * The combined safe set is the conjunction:
 *   safe(x) = safe_wait(s_ego, v_ego, s_onc1) ∧ safe_go(s_ego, v_ego, s_onc2)
 *
 * Each sub-SafeSet is a standard 3D bitmap queried independently.
 *
 * MPPI goal selection: if (s_ego=+10, v_ego, s_onc1, s_onc2) is safe in BOTH
 * sub-SafeSets → goal = +10 (proceed); otherwise → goal = -10 (yield).
 */

#pragma once

#include "../dynamics.h"
#include "../safe_set.h"

#include <cmath>

namespace rt_ctrl {

class TwoOncomingDynamics : public DynamicsModel {
public:
    double T_MAX  = 1200.0,  T_MIN  = -1800.0;
    double M_MIN  = 2000.0,  M_MAX  = 2250.0;
    double R_W_MIN = 0.30,   R_W_MAX = 0.35;
    double ALPHA_MIN = 300.0, ALPHA_MAX = 350.0;
    double BETA_MIN  = 0.10,  BETA_MAX  = 0.25;
    double GAMMA_MIN = 0.30,  GAMMA_MAX = 0.65;
    double V_MIN = 0.0,      V_MAX = 12.0;
    double COLLISION_ZONE_WIDTH = 10.0;
    double S_MAX = 10.0;

    double v_vehicle_1 = 8.0;   ///< Sensed velocity of vehicle 1 (closer, wait)
    double v_vehicle_2 = 12.0;  ///< Sensed velocity of vehicle 2 (farther, go)

    // -----------------------------------------------------------------------
    // Metadata
    // -----------------------------------------------------------------------
    std::string name() const override { return "two_oncoming"; }
    std::vector<std::string> state_names() const override {
        return {"s_ego", "v_ego", "s_onc1", "s_onc2"};
    }
    std::vector<std::string> input_names() const override {
        return {"T_ego"};
    }
    std::string measured_param_name() const override { return "v_vehicle_1"; }
    std::string synthesis_macro() const override { return "V0_MIN"; }

    // -----------------------------------------------------------------------
    // Dimensions — 4D state, 1D input
    // -----------------------------------------------------------------------
    int nx() const override { return 4; }
    int nu() const override { return 1; }
    int ny() const override { return 4; }

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    void set_param(const std::string& name, double value) override {
        if (name == "v_vehicle_1" || name == "V0_MIN") v_vehicle_1 = value;
        else if (name == "v_vehicle_2" || name == "V0_MAX") v_vehicle_2 = value;
        else if (name == "T_MAX")  T_MAX  = value;
        else if (name == "T_MIN")  T_MIN  = value;
        else if (name == "M_MIN")  M_MIN  = value;
        else if (name == "M_MAX")  M_MAX  = value;
        else if (name == "V_MAX")  V_MAX  = value;
        else if (name == "COLLISION_ZONE_WIDTH") COLLISION_ZONE_WIDTH = value;
    }

    // -----------------------------------------------------------------------
    // Vehicle acceleration (worst-case corners, same as turn_ego_first)
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
    // ODE: state = [s_ego, v_ego, s_onc1, s_onc2]
    //
    // Self-loop: when s_ego >= S_MAX, only ego stops (ds_ego=0, dv_ego=0).
    // Oncoming vehicles ALWAYS continue at their sensed velocities.
    // -----------------------------------------------------------------------
    void ode(const double* x, const double* u, double* dx) const override {
        if (x[0] >= S_MAX) {
            // Ego has crossed — hold ego state, but let oncoming continue
            dx[0] = 0.0;
            dx[1] = 0.0;
        } else {
            dx[0] = x[1];                                // ds_ego/dt = v_ego
            dx[1] = compute_vehicle_accel(x[1], u[0]);   // dv_ego/dt = accel
        }
        dx[2] = v_vehicle_1;  // ds_onc1/dt (vehicle 1 always advances)
        dx[3] = v_vehicle_2;  // ds_onc2/dt (vehicle 2 always advances)
    }

    void apply_constraints(double* x, const GridDesc& grid) const override {
        if (grid.n_dim > 0) x[0] = std::max(grid.lb[0], std::min(x[0], grid.ub[0]));
        if (grid.n_dim > 1) x[1] = std::max(V_MIN, std::min(x[1], V_MAX));
        // Do NOT clamp oncoming vehicles: they continue past the grid boundary
        // in the runtime simulation. Safe-set queries use bypass once s_onc >= 10.
    }

    // -----------------------------------------------------------------------
    // Cost function with goal-state safety check
    //
    // Goal selection (three-way):
    //   1. If oncoming vehicle 1 has NOT cleared the intersection → yield
    //   2. Elif current ego state is safe → proceed
    //   3. Else → yield (stop and wait for safety)
    // This ensures: if motion is safe, ego stops first then moves;
    //               if not safe at all, ego just stops and waits.
    // -----------------------------------------------------------------------
    CostParams default_cost_params() const override {
        return {
            {"w_progress",   1.0},
            {"w_safety",     10000.0},
            {"w_terminal_v", 500.0},
            {"w_jerk",       1e-5},
            {"goal",         10.0},
            {"unsafe_goal",  -10.0}
        };
    }

    MppiObjective make_objective(const SafeSet* ss,
                                 const CostParams& p) const override
    {
        const double w_prog = cparam(p, "w_progress",   1.0);
        const double w_safe = cparam(p, "w_safety",     10000.0);
        const double w_tv   = cparam(p, "w_terminal_v", 500.0);
        const double w_jerk = cparam(p, "w_jerk",       1e-5);
        const double goal   = cparam(p, "goal",         10.0);
        const double ugol   = cparam(p, "unsafe_goal",  -10.0);
        const int    nxd    = nx();
        const int    nud    = nu();
        constexpr double CZ = 5.0;  // collision zone half-width

        return [=](const Eigen::MatrixXd& x, const Eigen::MatrixXd& /*y*/,
                   const Eigen::MatrixXd& u, const double& /*slack*/) -> double
        {
            double cost = 0.0;
            const int H = static_cast<int>(x.rows()) - 1;

            // Goal selection: check current ego state safety, not goal state.
            //   1. If onc1 hasn't cleared the collision zone → wait
            //   2. Elif current state is safe → proceed (goal = +10)
            //   3. Else → wait (unsafe_goal = -10)
            double cur_state[MAX_DIM];
            row_to_buf(x, 0, nxd, cur_state);
            bool onc1_cleared = (x(0, 2) >= CZ);  // s_onc1 past collision zone
            bool cur_safe = ss->is_safe(cur_state);

            double g;
            if (!onc1_cleared) {
                g = ugol;   // first vehicle hasn't passed → yield
            } else if (cur_safe) {
                g = goal;   // safe to proceed → go
            } else {
                g = ugol;   // not safe → stop and wait
            }

            for (int k = 0; k <= H; ++k) {
                double ds = x(k, 0) - g;
                cost += w_prog * ds * ds;

                if (k > 0 && k < H && w_jerk > 0.0) {
                    for (int j = 0; j < nud; ++j) {
                        double du = u(k, j) - u(k - 1, j);
                        cost += w_jerk * du * du;
                    }
                }

                double sk[MAX_DIM];
                row_to_buf(x, k, nxd, sk);
                if (!ss->is_safe(sk)) cost += w_safe;
            }

            // Terminal velocity penalty: decelerate when goal is yield
            bool goal_is_yield = (g == ugol);
            if (goal_is_yield && w_tv > 0.0) {
                double vH = x(H, 1);
                cost += w_tv * vH * vH;
            }

            return cost;
        };
    }
};

}  // namespace rt_ctrl
