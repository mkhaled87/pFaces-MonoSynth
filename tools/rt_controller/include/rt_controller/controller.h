/**
 * @file controller.h
 * @brief MPC and MPPI controllers via libmpc++ with safe set integration.
 *
 * Provides a unified SafeController interface that wraps libmpc++'s NLMPC
 * class with either NLopt (gradient-based MPC) or MPPI (sampling-based)
 * optimizer backend.
 *
 * Safe set integration strategy:
 *   - The objective function includes a large penalty for predicted states
 *     that leave the safe set (bitmap O(1) check per state).
 *   - The finite penalty informs optimization but does not enforce a hard
 *     invariant constraint or provide a formal closed-loop safety guarantee.
 *
 * Dependencies: Eigen3, libmpc++ (headers), NLopt
 */

#pragma once

#include "dynamics.h"
#include "safe_set.h"
#include "types.h"

#include <mpc/NLMPC.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

namespace rt_ctrl {

// ---------------------------------------------------------------------------
// Controller configuration (scenario-agnostic)
// ---------------------------------------------------------------------------
struct ControllerConfig {
    // --- horizon ---
    int    prediction_horizon = 10;
    int    control_horizon    = 5;

    // --- dynamics ---
    double sampling_time  = 0.5;   ///< Controller sampling period (s)
    int    ode_substeps   = 100;   ///< RK4 sub-steps per sampling period

    // --- MPPI ---
    int    mppi_rollouts = 512;
    int    mppi_iterations = 100;
    double mppi_sigma    = 300.0; ///< Control noise std (torque units)
};

// ---------------------------------------------------------------------------
// SafeController: unified MPC/MPPI + bitmap safety via libmpc++
// ---------------------------------------------------------------------------
class SafeController {
public:
    SafeController() = default;

    /**
     * Initialize the controller.
     * @param cost_params  Scenario-specific cost weights (merged defaults + JSON).
     */
    void init(const ControllerConfig& cfg,
              std::shared_ptr<DynamicsModel> dynamics,
              SafeSet& safe_set,
              const GridDesc& state_grid,
              const GridDesc& input_grid,
              const DynamicsModel::CostParams& cost_params = {})
    {
        cfg_        = cfg;
        dyn_        = dynamics;
        safe_set_   = &safe_set;
        state_grid_ = state_grid;
        input_grid_ = input_grid;

        const int nx   = dyn_->nx();
        const int nu   = dyn_->nu();
        const int ny   = dyn_->ny();
        const int ph   = cfg_.prediction_horizon;
        const int ch   = cfg_.control_horizon;
        const int ineq = 0;
        const int eq   = 0;

        nlmpc_ = std::make_unique<mpc::NLMPC<>>(
            nx, nu, ny, ph, ch, ineq, eq, mpc::OptimizerType::MPPI);

        nlmpc_->setLoggerLevel(mpc::Logger::LogLevel::NONE);
        nlmpc_->setDiscretizationSamplingTime(cfg_.sampling_time);

        // --- State space function: dx/dt = f(x, u) ---
        auto dyn_ptr = dyn_.get();
        nlmpc_->setStateSpaceFunction(
            [dyn_ptr](mpc::cvec<Eigen::Dynamic>& dx,
                      const mpc::cvec<Eigen::Dynamic>& x,
                      const mpc::cvec<Eigen::Dynamic>& u,
                      const unsigned int& /*step*/)
            {
                dx.resize(dyn_ptr->nx());
                dyn_ptr->ode(x.data(), u.data(), dx.data());
            }
        );

        // --- Output function: y = g(x, u) ---
        nlmpc_->setOutputFunction(
            [dyn_ptr](mpc::cvec<Eigen::Dynamic>& y,
                      const mpc::cvec<Eigen::Dynamic>& x,
                      const mpc::cvec<Eigen::Dynamic>& u,
                      const unsigned int& /*step*/)
            {
                y.resize(dyn_ptr->ny());
                dyn_ptr->output(x.data(), u.data(), y.data());
            }
        );

        // --- Objective function: built by the scenario's make_objective() ---
        nlmpc_->setObjectiveFunction(
            dyn_->make_objective(safe_set_, cost_params)
        );

        // --- State bounds ---
        mpc::cvec<Eigen::Dynamic> xmin(nx), xmax(nx);
        for (int d = 0; d < nx; ++d) {
            xmin(d) = state_grid_.lb[d];
            xmax(d) = state_grid_.ub[d];
        }
        nlmpc_->setStateBounds(xmin, xmax, {0, ph});

        // --- Input bounds ---
        mpc::cvec<Eigen::Dynamic> umin(nu), umax(nu);
        for (int d = 0; d < nu; ++d) {
            umin(d) = input_grid_.lb[d];
            umax(d) = input_grid_.ub[d];
        }
        nlmpc_->setInputBounds(umin, umax, {0, ch});

        mpc::MPPIParameters mppi_p;
        mppi_p.num_rollouts         = cfg_.mppi_rollouts;
        mppi_p.maximum_iteration    = cfg_.mppi_iterations;
        mppi_p.sigma.resize(nu);
        mppi_p.sigma.setConstant(cfg_.mppi_sigma);
        mppi_p.state_bound_penalty  = 100.0;
        mppi_p.ineq_penalty         = 10.0;
        mppi_p.barrier_steepness    = 2.0;
        mppi_p.integration_substeps = 1;
        nlmpc_->setOptimizerParameters(mppi_p);

        last_u_ = mpc::cvec<Eigen::Dynamic>::Zero(nu);
        initialized_ = true;
    }

    /**
     * Compute optimal control for current state.
     */
    ControlResult step(const Vec& state) {
        assert(initialized_ && "Controller not initialized");

        ControlResult result;
        auto t0 = std::chrono::high_resolution_clock::now();

        auto opt_result = nlmpc_->optimize(state, last_u_);

        auto t1 = std::chrono::high_resolution_clock::now();
        result.solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        result.cmd      = opt_result.cmd;
        result.cost     = opt_result.cost;
        result.feasible = (opt_result.status == mpc::ResultStatus::SUCCESS);
        result.status   = static_cast<int>(opt_result.status);

        last_u_ = result.cmd;
        return result;
    }

    const ControllerConfig& config() const { return cfg_; }

private:
    ControllerConfig                    cfg_;
    std::shared_ptr<DynamicsModel>      dyn_;
    SafeSet*                            safe_set_  = nullptr;
    GridDesc                            state_grid_;
    GridDesc                            input_grid_;
    std::unique_ptr<mpc::NLMPC<>>       nlmpc_;
    mpc::cvec<Eigen::Dynamic>           last_u_;
    bool                                initialized_ = false;
};

}  // namespace rt_ctrl
