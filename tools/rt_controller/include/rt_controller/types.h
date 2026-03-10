/**
 * @file types.h
 * @brief Common types and utilities for the real-time controller framework.
 *
 * This header defines shared types, constants, and inline utilities used
 * throughout the MonoSafe real-time controller framework.
 */

#pragma once

#include <Eigen/Dense>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace rt_ctrl {

// ---------------------------------------------------------------------------
// Compile-time limits
// ---------------------------------------------------------------------------
static constexpr int MAX_DIM = 8;

// ---------------------------------------------------------------------------
// Eigen convenience aliases (dynamic size — dimension set at runtime)
// ---------------------------------------------------------------------------
using Vec  = Eigen::VectorXd;
using Mat  = Eigen::MatrixXd;
using VecI = Eigen::VectorXi;

// ---------------------------------------------------------------------------
// Grid descriptor — captures the N-dimensional grid structure
// ---------------------------------------------------------------------------
struct GridDesc {
    int              n_dim   = 0;          ///< Number of dimensions
    std::vector<int> sizes;                ///< Number of cells per dimension
    std::vector<double> lb;                ///< Lower bound per dimension
    std::vector<double> ub;                ///< Upper bound per dimension
    std::vector<double> eta;               ///< Cell width per dimension
    int              total_cells = 0;      ///< Product of all sizes

    /// Compute flat index from multi-index (row-major: last dim varies fastest)
    inline int flatten(const int* idx) const {
        int flat = 0;
        int stride = 1;
        for (int d = n_dim - 1; d >= 0; --d) {
            flat += idx[d] * stride;
            stride *= sizes[d];
        }
        return flat;
    }

    /// Compute multi-index from flat index
    inline void unflatten(int flat, int* idx) const {
        for (int d = n_dim - 1; d >= 0; --d) {
            idx[d] = flat % sizes[d];
            flat /= sizes[d];
        }
    }

    /// Continuous state → grid index (clamp to bounds)
    inline void state_to_grid(const double* x, int* idx) const {
        for (int d = 0; d < n_dim; ++d) {
            double clamped = std::max(lb[d], std::min(x[d], ub[d]));
            idx[d] = static_cast<int>((clamped - lb[d]) / eta[d]);
            idx[d] = std::max(0, std::min(idx[d], sizes[d] - 1));
        }
    }

    /// Grid index → continuous state (cell center)
    inline void grid_to_state(const int* idx, double* x) const {
        for (int d = 0; d < n_dim; ++d) {
            x[d] = lb[d] + idx[d] * eta[d];
        }
    }

    /// Recompute total_cells from sizes
    void recompute_total() {
        total_cells = 1;
        for (int d = 0; d < n_dim; ++d) total_cells *= sizes[d];
    }
};

// ---------------------------------------------------------------------------
// Timer utility
// ---------------------------------------------------------------------------
struct ScopedTimer {
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point start;
    double* out_ms;

    ScopedTimer(double* target) : start(Clock::now()), out_ms(target) {}
    ~ScopedTimer() {
        auto end = Clock::now();
        *out_ms = std::chrono::duration<double, std::milli>(end - start).count();
    }
};

// ---------------------------------------------------------------------------
// Control result
// ---------------------------------------------------------------------------
struct ControlResult {
    Vec    cmd;             ///< Optimal control input
    double cost     = 0.0; ///< Cost of the optimal trajectory
    double solve_ms = 0.0; ///< Solver wall-clock time (ms)
    bool   feasible = false;///< Whether the optimizer found a feasible solution
    int    status   = -1;  ///< Solver status code
};

// ---------------------------------------------------------------------------
// Simulation log entry (one per time step)
// ---------------------------------------------------------------------------
struct LogEntry {
    double time       = 0.0;
    Vec    state;
    Vec    control;
    double param_value = 0.0;  ///< Measured parameter value (0 if none)
    bool   is_safe    = false;
    double safe_s_lo  = std::numeric_limits<double>::quiet_NaN(); ///< Safe range lower bound along s_ego / dim 0
    double safe_s_hi  = std::numeric_limits<double>::quiet_NaN(); ///< Safe range upper bound along s_ego / dim 0
    double query_ns   = 0.0;   ///< Safe set query time (nanoseconds)
    double synth_ms   = 0.0;   ///< Synthesis time (ms), 0 if not recomputed
    double ctrl_ms    = 0.0;   ///< Controller solve time (ms)
    int    basis_size = 0;     ///< Current basis size
    double safe_frac  = 0.0;   ///< Fraction of state space that is safe
};

}  // namespace rt_ctrl
