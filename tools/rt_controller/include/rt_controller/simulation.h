/**
 * @file simulation.h
 * @brief Closed-loop simulation with online synthesis, control, and logging.
 *
 * Orchestrates the full real-time pipeline:
 *   1. Sense → read measured parameter from sensor (if applicable)
 *   2. Synthesize → recompute safe set if parameter changed significantly
 *   3. Control → MPPI with bitmap safety checking
 *   4. Simulate → RK4 step with the exact same dynamics
 *   5. Log → write CSV for post-hoc visualization
 *
 * Fully generic: works with any scenario registered in the dynamics factory.
 * State names, parameter handling, and logging adapt automatically.
 */

#pragma once

#include "controller.h"
#include "dynamics.h"
#include "safe_set.h"
#include "synthesis.h"
#include "types.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace rt_ctrl {

// ---------------------------------------------------------------------------
// Sensor model: provides a time-varying measured parameter
// ---------------------------------------------------------------------------
/**
 * Abstract sensor interface for a single measured parameter.
 * For scenarios with no runtime parameter (e.g. ACC), sensor is nullptr.
 */
class Sensor {
public:
    virtual ~Sensor() = default;
    virtual double sense(double sim_time) const = 0;
};

/// Constant value sensor
class ConstantSensor : public Sensor {
public:
    ConstantSensor(double v) : v_(v) {}
    double sense(double /*t*/) const override { return v_; }
private:
    double v_;
};

/// Step change: v0 for t < t_step, v1 after
class StepSensor : public Sensor {
public:
    StepSensor(double v0, double v1, double t_step)
        : v0_(v0), v1_(v1), t_step_(t_step) {}
    double sense(double t) const override { return t < t_step_ ? v0_ : v1_; }
private:
    double v0_, v1_, t_step_;
};

/// Sinusoidal variation around a mean
class SineSensor : public Sensor {
public:
    SineSensor(double mean, double amp, double freq)
        : mean_(mean), amp_(amp), freq_(freq) {}
    double sense(double t) const override {
        return mean_ + amp_ * std::sin(2.0 * M_PI * freq_ * t);
    }
private:
    double mean_, amp_, freq_;
};

// ---------------------------------------------------------------------------
// Simulation configuration
// ---------------------------------------------------------------------------
struct SimConfig {
    Vec    x0;                        ///< Initial state
    double total_time     = 20.0;     ///< Simulation duration (seconds)
    double dt             = 0.5;      ///< Simulation time step (= control period)
    int    ode_steps      = 1000;     ///< RK4 sub-steps per dt
    double resynth_thresh = 0.5;      ///< Resynthesize if |Δparam| > threshold
    std::string resynth_policy = "threshold"; ///< "always", "never", or "threshold"
    std::string log_file  = "sim_log.csv";

    // Verbosity
    bool   verbose        = true;
};

// ---------------------------------------------------------------------------
// Simulation engine
// ---------------------------------------------------------------------------
class Simulation {
public:
    /**
     * @param sensor  Measured-parameter sensor. Pass nullptr if the scenario
     *                has no runtime parameter (e.g. ACC). In that case,
     *                synthesis must happen once before run() is called.
     */
    Simulation(std::shared_ptr<DynamicsModel> dynamics,
               SafeSet& safe_set,
               SafeController& controller,
               std::shared_ptr<SynthesisBackend> synth,
               std::shared_ptr<Sensor> sensor,
               const GridDesc& state_grid)
        : dyn_(dynamics)
        , safe_set_(safe_set)
        , ctrl_(controller)
        , synth_(synth)
        , sensor_(sensor)
        , state_grid_(state_grid)
    {}

    /**
     * Run the full closed-loop simulation.
     * @return  Vector of log entries (one per time step).
     */
    std::vector<LogEntry> run(const SimConfig& sim_cfg) {
        std::vector<LogEntry> log;
        Vec state = sim_cfg.x0;
        double t = 0.0;
        double last_param_val = std::numeric_limits<double>::quiet_NaN();
        int step = 0;
        const int total_steps = static_cast<int>(sim_cfg.total_time / sim_cfg.dt);

        const bool has_param = (sensor_ != nullptr);
        const std::string param_name = dyn_->measured_param_name();
        const auto snames = dyn_->state_names();
        const auto inames = dyn_->input_names();

        if (sim_cfg.verbose) {
            std::cout << "╔══════════════════════════════════════════════════╗\n"
                      << "║  MonoSafe Real-Time Controller Simulation       ║\n"
                      << "╠══════════════════════════════════════════════════╣\n"
                      << "║  Scenario:   " << dyn_->name()
                      << std::string(std::max(1, 34 - (int)dyn_->name().size()), ' ') << "║\n"
                      << "║  State dim:  " << dyn_->nx() << "    Input dim: " << dyn_->nu()
                      << std::string(std::max(1, 23 - (int)std::to_string(dyn_->nx()).size()
                                        - (int)std::to_string(dyn_->nu()).size()), ' ') << "║\n"
                      << "║  Grid cells: " << safe_set_.total_cells()
                      << std::string(std::max(1, 34 - (int)std::to_string(safe_set_.total_cells()).size()), ' ') << "║\n"
                      << "║  Duration:   " << sim_cfg.total_time << " s"
                      << "   Steps: " << total_steps
                      << std::string(std::max(1, 18 - (int)std::to_string(total_steps).size()), ' ') << "║\n";
            if (has_param)
                std::cout << "║  Param:      " << param_name
                          << std::string(std::max(1, 34 - (int)param_name.size()), ' ') << "║\n";
            std::cout << "╚══════════════════════════════════════════════════╝\n\n";
        }

        while (t <= sim_cfg.total_time + 1e-9) {
            LogEntry entry;
            entry.time  = t;
            entry.state = state;

            if (sim_cfg.verbose) {
                std::cout << "\n────── Step " << step
                          << "  t=" << std::fixed << std::setprecision(2) << t
                          << "s ──────\n";
                std::cout << "  State:";
                for (int d = 0; d < dyn_->nx(); ++d) {
                    std::cout << " " << (d < (int)snames.size() ? snames[d] : "x" + std::to_string(d))
                              << "=" << std::setprecision(2) << state[d];
                }
                std::cout << "\n";
            }

            // 1. Sense measured parameter (if applicable)
            double param_val = 0.0;
            if (has_param) {
                param_val = sensor_->sense(t);
                entry.param_value = param_val;
                dyn_->set_param(param_name, param_val);

                if (sim_cfg.verbose) {
                    std::cout << "  [1] Param:     " << param_name << "="
                              << std::setprecision(2) << param_val << "\n";
                }
            } else if (sim_cfg.verbose) {
                std::cout << "  [1] Param:     (none)\n";
            }

            // 2. Resynthesize based on policy
            bool do_resynth = false;
            if (has_param) {
                if (sim_cfg.resynth_policy == "always") {
                    do_resynth = true;
                } else if (sim_cfg.resynth_policy == "never") {
                    do_resynth = false;
                } else { // "threshold" (default)
                    do_resynth = std::isnan(last_param_val) ||
                                 std::fabs(param_val - last_param_val) > sim_cfg.resynth_thresh;
                }
            }
            if (do_resynth) {
                double synth_ms = synth_->synthesize(param_val, safe_set_);
                entry.synth_ms = synth_ms;
                last_param_val = param_val;

                if (sim_cfg.verbose) {
                    const auto& d = synth_->last_detail();
                    std::cout << "  [2] Synthesis: " << std::setprecision(1) << synth_ms << " ms  (RESYNTHESIZED)\n"
                              << "        GPU kernels : " << std::setprecision(1) << d.gpu_exec_ms << " ms"
                              << "  (" << d.iterations << " iters)\n"
                              << "        Bitmap xfer : " << std::setprecision(2) << d.transfer_ms << " ms\n"
                              << "        Basis: " << d.basis_size
                              << "  Safe: " << d.safe_cells << "/" << d.total_cells
                              << " (" << std::setprecision(1)
                              << (d.total_cells > 0 ? 100.0 * d.safe_cells / d.total_cells : 0.0) << "%)\n";
                }
            } else if (sim_cfg.verbose) {
                if (has_param) {
                    std::cout << "  [2] Synthesis: skipped (Δ="
                              << std::setprecision(2)
                              << (std::isnan(last_param_val) ? 0.0 : std::fabs(param_val - last_param_val))
                              << ")\n";
                } else {
                    std::cout << "  [2] Synthesis: one-shot (no param)\n";
                }
            }

            // 3. Check current safety
            auto q_t0 = std::chrono::high_resolution_clock::now();
            entry.is_safe = safe_set_.is_safe(state);
            auto q_t1 = std::chrono::high_resolution_clock::now();
            entry.query_ns = std::chrono::duration<double, std::nano>(q_t1 - q_t0).count();

            entry.basis_size = safe_set_.basis_size();
            entry.safe_frac  = safe_set_.safe_fraction();
            auto safe_range = safe_set_.safe_range_along_dim(0, state.data());
            entry.safe_s_lo = safe_range.first;
            entry.safe_s_hi = safe_range.second;

            if (sim_cfg.verbose) {
                std::cout << "  [3] SafeQuery: " << (entry.is_safe ? "SAFE ✓" : "UNSAFE ✗")
                          << "  (" << std::setprecision(0) << entry.query_ns << " ns)"
                          << "  basis=" << entry.basis_size
                          << "  safe=" << std::setprecision(1)
                          << (entry.safe_frac * 100.0) << "%\n";
            }

            // 4. Compute control
            ControlResult ctrl_result = ctrl_.step(state);
            entry.control = ctrl_result.cmd;
            entry.ctrl_ms = ctrl_result.solve_ms;

            if (sim_cfg.verbose) {
                std::cout << "  [4] Control:   u=[";
                for (int d = 0; d < dyn_->nu(); ++d) {
                    if (d > 0) std::cout << ", ";
                    std::cout << std::setprecision(1) << ctrl_result.cmd[d];
                }
                std::cout << "]  MPPI=" << std::setprecision(2) << ctrl_result.solve_ms << " ms\n";
            }

            // 5. Log
            log.push_back(entry);

            // 6. Simulate one step (same dynamics as synthesis)
            auto ts0 = std::chrono::high_resolution_clock::now();
            double x_next_arr[MAX_DIM];
            dyn_->step(state.data(), ctrl_result.cmd.data(), x_next_arr,
                       sim_cfg.dt, sim_cfg.ode_steps);
            dyn_->apply_constraints(x_next_arr, state_grid_);
            auto ts1 = std::chrono::high_resolution_clock::now();
            double sim_step_ms = std::chrono::duration<double, std::milli>(ts1 - ts0).count();

            // Update state
            for (int d = 0; d < dyn_->nx(); ++d) {
                state[d] = x_next_arr[d];
            }

            if (sim_cfg.verbose) {
                double total_step_ms = entry.synth_ms + ctrl_result.solve_ms + sim_step_ms;
                std::cout << "  [5] Simulate:  " << std::setprecision(3) << sim_step_ms << " ms →";
                for (int d = 0; d < std::min(dyn_->nx(), 3); ++d) {
                    std::cout << " " << (d < (int)snames.size() ? snames[d] : "x" + std::to_string(d))
                              << "=" << std::setprecision(2) << state[d];
                }
                std::cout << "\n  Total step: " << std::setprecision(1) << total_step_ms << " ms\n";
            }

            t += sim_cfg.dt;
            ++step;
        }

        // Write CSV log
        write_log(log, sim_cfg.log_file);

        // Print summary
        if (sim_cfg.verbose) {
            print_summary(log);
        }

        return log;
    }

private:
    void write_log(const std::vector<LogEntry>& log, const std::string& path) {
        std::ofstream ofs(path);
        if (!ofs.is_open()) {
            std::cerr << "[Simulation] Cannot open log file: " << path << "\n";
            return;
        }

        // Header
        ofs << "time";
        for (int d = 0; d < dyn_->nx(); ++d) ofs << ",x" << d;
        for (int d = 0; d < dyn_->nu(); ++d) ofs << ",u" << d;
        ofs << ",param_value,is_safe,safe_s_lo,safe_s_hi,query_ns,synth_ms,ctrl_ms,basis_size,safe_frac\n";

        // Data
        for (const auto& e : log) {
            ofs << std::fixed << std::setprecision(6) << e.time;
            for (int d = 0; d < dyn_->nx(); ++d) {
                ofs << "," << (d < e.state.size() ? e.state[d] : 0.0);
            }
            for (int d = 0; d < dyn_->nu(); ++d) {
                ofs << "," << (d < e.control.size() ? e.control[d] : 0.0);
            }
            ofs << "," << e.param_value
                << "," << (e.is_safe ? 1 : 0)
                << "," << e.safe_s_lo
                << "," << e.safe_s_hi
                << "," << e.query_ns
                << "," << e.synth_ms
                << "," << e.ctrl_ms
                << "," << e.basis_size
                << "," << e.safe_frac
                << "\n";
        }
    }

    void print_summary(const std::vector<LogEntry>& log) {
        int safe_count = 0, total = static_cast<int>(log.size());
        double total_ctrl_ms = 0.0, max_ctrl_ms = 0.0;
        double total_query_ns = 0.0;

        for (const auto& e : log) {
            if (e.is_safe) ++safe_count;
            total_ctrl_ms += e.ctrl_ms;
            max_ctrl_ms = std::max(max_ctrl_ms, e.ctrl_ms);
            total_query_ns += e.query_ns;
        }

        std::cout << "\n╔══════════════════════════════════════════════════╗\n"
                  << "║  Simulation Summary                              ║\n"
                  << "╠══════════════════════════════════════════════════╣\n"
                  << std::fixed
                  << "║  Scenario:        " << dyn_->name() << "\n"
                  << "║  Total steps:     " << total << "\n"
                  << "║  Safe steps:      " << safe_count << "/" << total
                  << " (" << std::setprecision(1)
                  << (100.0 * safe_count / std::max(1, total)) << "%)\n"
                  << "║  Avg ctrl time:   " << std::setprecision(2)
                  << (total_ctrl_ms / std::max(1, total)) << " ms\n"
                  << "║  Max ctrl time:   " << max_ctrl_ms << " ms\n"
                  << "║  Avg query time:  " << std::setprecision(0)
                  << (total_query_ns / std::max(1, total)) << " ns\n"
                  << "║  Final basis:     " << (log.empty() ? 0 : log.back().basis_size) << "\n"
                  << "╚══════════════════════════════════════════════════╝\n";
    }

    std::shared_ptr<DynamicsModel>      dyn_;
    SafeSet&                            safe_set_;
    SafeController&                     ctrl_;
    std::shared_ptr<SynthesisBackend>   synth_;
    std::shared_ptr<Sensor>             sensor_;
    GridDesc                            state_grid_;
};

}  // namespace rt_ctrl
