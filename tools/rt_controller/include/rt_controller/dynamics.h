/**
 * @file dynamics.h
 * @brief Abstract dynamics interface for the real-time controller framework.
 *
 * Provides the DynamicsModel base class that every scenario must subclass.
 * Each scenario defines:
 *   - ode()               — continuous-time dynamics dx/dt = f(x,u)
 *   - make_objective()    — MPPI cost function (C++ lambda)
 *   - state_names()       — human-readable labels for console/plots
 *   - measured_param_name()— runtime-varying parameter (triggers re-synthesis)
 *
 * Concrete scenarios live in scenarios/ subdirectory.
 * Register via the factory in scenarios/all.h.
 */

#pragma once

#include "config.h"
#include "types.h"

#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace rt_ctrl {

// Forward declaration (full definition in safe_set.h)
class SafeSet;

// ---------------------------------------------------------------------------
// Abstract dynamics interface
// ---------------------------------------------------------------------------
class DynamicsModel {
public:
    virtual ~DynamicsModel() = default;

    // -----------------------------------------------------------------------
    // Scenario metadata — override in every subclass
    // -----------------------------------------------------------------------

    /// Unique scenario name (used for factory lookup)
    virtual std::string name() const = 0;

    /// Human-readable labels for each state variable
    virtual std::vector<std::string> state_names() const = 0;

    /// Human-readable labels for each input variable
    virtual std::vector<std::string> input_names() const = 0;

    /**
     * Name of the runtime-varying measured parameter (e.g. "v_oncoming").
     * Return "" if the scenario has no runtime parameter (e.g. ACC).
     * When non-empty, the simulation loop reads this from the sensor,
     * passes it to set_param(), and triggers re-synthesis as needed.
     */
    virtual std::string measured_param_name() const { return ""; }

    /**
     * OpenCL #define macro to patch in ExternalSynthesis (e.g. "V0_MAX").
     * Return "" if no patching is needed.
     */
    virtual std::string synthesis_macro() const { return ""; }

    // -----------------------------------------------------------------------
    // Dimensions
    // -----------------------------------------------------------------------

    /// State dimension
    virtual int nx() const = 0;
    /// Input dimension
    virtual int nu() const = 0;
    /// Output dimension (for libmpc; typically == nx)
    virtual int ny() const = 0;

    // -----------------------------------------------------------------------
    // Dynamics
    // -----------------------------------------------------------------------

    /**
     * Continuous-time ODE: dx/dt = f(x, u)
     * @param x   State vector (size nx)
     * @param u   Input vector (size nu)
     * @param dx  Output derivative (size nx)
     */
    virtual void ode(const double* x, const double* u, double* dx) const = 0;

    /**
     * Single discrete step via RK4 integration.
     * @param x       Current state (size nx)
     * @param u       Control input (size nu)
     * @param x_next  Next state (size nx)
     * @param dt      Sampling period (seconds)
     * @param steps   Number of RK4 sub-steps
     */
    virtual void step(const double* x, const double* u, double* x_next,
                      double dt, int steps) const {
        const int n = nx();
        const double h = dt / static_cast<double>(steps);

        // Work buffers (stack allocation for small dims)
        double state[MAX_DIM], k1[MAX_DIM], k2[MAX_DIM], k3[MAX_DIM], k4[MAX_DIM], tmp[MAX_DIM];

        for (int d = 0; d < n; ++d) state[d] = x[d];

        for (int s = 0; s < steps; ++s) {
            ode(state, u, k1);
            for (int d = 0; d < n; ++d) tmp[d] = state[d] + 0.5 * h * k1[d];
            ode(tmp, u, k2);
            for (int d = 0; d < n; ++d) tmp[d] = state[d] + 0.5 * h * k2[d];
            ode(tmp, u, k3);
            for (int d = 0; d < n; ++d) tmp[d] = state[d] + h * k3[d];
            ode(tmp, u, k4);
            for (int d = 0; d < n; ++d) {
                state[d] += (h / 6.0) * (k1[d] + 2.0 * k2[d] + 2.0 * k3[d] + k4[d]);
            }
        }

        for (int d = 0; d < n; ++d) x_next[d] = state[d];
    }

    /**
     * Apply state constraints / clamping after integration.
     * Default: clamp to grid bounds.  Override for custom logic.
     */
    virtual void apply_constraints(double* x, const GridDesc& grid) const {
        for (int d = 0; d < grid.n_dim; ++d) {
            x[d] = std::max(grid.lb[d], std::min(x[d], grid.ub[d]));
        }
    }

    /**
     * Output function y = g(x, u).  Default: y = x.
     */
    virtual void output(const double* x, const double* /*u*/, double* y) const {
        for (int d = 0; d < nx(); ++d) y[d] = x[d];
    }

    /**
     * Set a named parameter (e.g., sensed v_oncoming).
     * Subclasses override to intercept relevant parameters.
     */
    virtual void set_param(const std::string& /*name*/, double /*value*/) {}

    // -----------------------------------------------------------------------
    // MPPI cost function — override in every subclass
    // -----------------------------------------------------------------------

    /// Cost parameter map: name → value (read from JSON "cost" block)
    using CostParams = std::map<std::string, double>;

    /// MPPI objective function signature (matches libmpc++ setObjectiveFunction)
    using MppiObjective = std::function<double(
        const Eigen::MatrixXd& x,       // predicted states  [H+1 × nx]
        const Eigen::MatrixXd& y,       // predicted outputs [H+1 × ny]
        const Eigen::MatrixXd& u,       // control inputs    [H   × nu]
        const double& slack)>;

    /**
     * Build the MPPI objective lambda.
     *
     * Called once during controller init. The returned lambda captures
     * the SafeSet pointer (for O(1) bitmap queries) and cost weights.
     *
     * @param safe_set  Pointer to the safe set (bitmap updated in-place).
     * @param params    Cost weights merged from scenario defaults + JSON.
     */
    virtual MppiObjective make_objective(const SafeSet* safe_set,
                                         const CostParams& params) const = 0;

    /**
     * Default cost parameter names and values.
     * Scenarios override to document their expected weights.
     * These are filled first, then overridden by JSON values.
     */
    virtual CostParams default_cost_params() const { return {}; }

    // -----------------------------------------------------------------------
    // Utilities (available to all scenarios)
    // -----------------------------------------------------------------------

    /// Read a cost parameter with default fallback.
    static double cparam(const CostParams& p, const std::string& k, double def) {
        auto it = p.find(k);
        return it != p.end() ? it->second : def;
    }

    /// Copy row k of a column-major Eigen matrix into a contiguous buffer.
    /// Critical: Eigen stores matrices column-major, so row(k).data() is NOT
    /// contiguous. This helper ensures is_safe() reads the correct state.
    static void row_to_buf(const Eigen::MatrixXd& m, int k, int cols, double* buf) {
        for (int d = 0; d < cols; ++d) buf[d] = m(k, d);
    }
};

// ===========================================================================
// Dynamics factory — register new dynamics by name
// ===========================================================================
using DynamicsFactory = std::function<std::unique_ptr<DynamicsModel>(const Config&)>;

inline std::unordered_map<std::string, DynamicsFactory>& dynamics_registry() {
    static std::unordered_map<std::string, DynamicsFactory> reg;
    return reg;
}

inline void register_dynamics(const std::string& name, DynamicsFactory factory) {
    dynamics_registry()[name] = std::move(factory);
}

/**
 * Create a dynamics model by name.
 * @param name  Scenario name (from JSON "dynamics_class" or .cfg project_name).
 * @param cfg   Parsed .cfg (passed to factory, may be used for init).
 */
inline std::unique_ptr<DynamicsModel> create_dynamics(const std::string& name,
                                                       const Config& cfg) {
    auto& reg = dynamics_registry();
    auto it = reg.find(name);
    if (it != reg.end()) return it->second(cfg);

    // List available for error message
    std::string available;
    for (auto& [k, v] : reg) {
        if (!available.empty()) available += ", ";
        available += k;
    }
    throw std::runtime_error("Unknown dynamics class: '" + name +
                             "'. Available: [" + available + "]");
}

}  // namespace rt_ctrl
