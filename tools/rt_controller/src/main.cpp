/**
 * @file main.cpp
 * @brief Entry point for the MonoSafe real-time controller framework.
 *
 * All runtime parameters are controlled via a single JSON file.
 * The JSON references a .cfg file for grid/dynamics definitions.
 *
 * Usage:
 *   rt_controller <config.json> [output_dir]
 *   rt_controller --help
 *
 * If output_dir is provided, all outputs (log CSV) are written there.
 * Otherwise defaults to current working directory.
 *
 * See scenarios/ directory for built-in scenarios.
 */

#include "rt_controller/config.h"
#include "rt_controller/controller.h"
#include "rt_controller/dynamics.h"
#include "rt_controller/safe_set.h"
#include "rt_controller/simulation.h"
#include "rt_controller/synthesis.h"
#include "rt_controller/types.h"

// Include all built-in scenarios (registers them in the dynamics factory)
#include "rt_controller/scenarios/all.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace rt_ctrl;
namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// JSON helpers
// ---------------------------------------------------------------------------
template <typename T>
T jget(const json& j, const std::string& key, const T& def) {
    return j.contains(key) ? j[key].get<T>() : def;
}

// ---------------------------------------------------------------------------
// Print help
// ---------------------------------------------------------------------------
void print_help() {
    std::cout << R"(
MonoSafe Real-Time Controller Framework
========================================

Usage:
  rt_controller <config.json> [output_dir]
  rt_controller --help

The JSON config file controls ALL runtime parameters.
Grid and dynamics definitions are read from the .cfg file
referenced by "cfg_file" in the JSON.

JSON structure:
  {
    "cfg_file":        "path/to/problem.cfg",
    "synthesis":       { "mode": "direct", "kernel_pack": "...", "device_id": 0 },
    "controller":      { "prediction_horizon": 10, "cost": { ... }, ... },
    "simulation":      { "x0": [...], "duration": 20, ... },
    "sensor":          { "type": "const", "value": 12.0 }
  }

Built-in scenarios:
  - turn_ego_first       (3D: s_ego, v_ego, s_onc)
  - turn_oncoming_first  (3D: s_ego, v_ego, s_onc)
  - acc                  (3D: h, v_ego, v_lead)
)";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
        print_help();
        return (argc < 2) ? 1 : 0;
    }

    std::string json_path = argv[1];
    std::string output_dir = (argc >= 3) ? argv[2] : ".";

    try {
        // Create output directory if needed
        if (output_dir != ".") {
            fs::create_directories(output_dir);
        }

        // =============================================================
        // 0. Load JSON config
        // =============================================================
        std::ifstream jf(json_path);
        if (!jf.is_open())
            throw std::runtime_error("Cannot open JSON config: " + json_path);
        json J = json::parse(jf);

        bool verbose = jget<bool>(J, "verbose", true);

        // =============================================================
        // 1. Parse the .cfg for grid / dynamics definitions
        // =============================================================
        std::string cfg_path = J.at("cfg_file").get<std::string>();
        Config cfg = parse_config(cfg_path);
        if (verbose)
            std::cout << "Loaded .cfg: " << cfg.project_name
                      << " (" << cfg.state_grid.n_dim << "D, "
                      << cfg.state_grid.total_cells << " cells)\n";

        // =============================================================
        // 2. Create dynamics model (from JSON or .cfg project_name)
        // =============================================================
        std::string dynamics_class = jget<std::string>(J, "dynamics_class",
                                                        cfg.project_name);
        auto dynamics = std::shared_ptr<DynamicsModel>(
            create_dynamics(dynamics_class, cfg));

        if (verbose) {
            std::cout << "Dynamics:    " << dynamics->name()
                      << " (nx=" << dynamics->nx()
                      << " nu=" << dynamics->nu() << ")\n";
            auto sn = dynamics->state_names();
            std::cout << "  States:    [";
            for (size_t i = 0; i < sn.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << sn[i];
            }
            std::cout << "]\n";
            if (!dynamics->measured_param_name().empty())
                std::cout << "  Param:     " << dynamics->measured_param_name() << "\n";
        }

        // Apply any dynamics overrides from JSON
        if (J.contains("dynamics_params")) {
            for (auto& [k, v] : J["dynamics_params"].items())
                dynamics->set_param(k, v.get<double>());
        }

        // =============================================================
        // 3. Initialize safe set
        // =============================================================
        SafeSet safe_set;
        safe_set.init(cfg.state_grid, cfg.priorities);

        // =============================================================
        // 4. Synthesis backend
        // =============================================================
        json js = J.value("synthesis", json::object());
        std::string synth_mode   = jget<std::string>(js, "mode", "external");
        std::string kernel_pack  = jget<std::string>(js, "kernel_pack", "../../kernel-pack");
        int device_id            = jget<int>(js, "device_id", 1);
        std::string basis_path   = jget<std::string>(js, "basis_file", "");

        bool has_runtime_param = !dynamics->measured_param_name().empty();

        std::unique_ptr<SynthesisBackend> synth;
        if (synth_mode == "file") {
            if (basis_path.empty())
                throw std::runtime_error("synthesis.basis_file required for mode=file");
            synth = std::make_unique<FileSynthesis>(basis_path);
        } else if (synth_mode == "external") {
            ExternalSynthesis::Params ep;
            ep.cfg_path         = cfg_path;
            ep.kernel_pack      = kernel_pack;
            ep.dynamics_file    = cfg.dynamics_file;
            ep.synthesis_macro  = dynamics->synthesis_macro();
            ep.device_id        = device_id;
            ep.output_dir       = output_dir;
            synth = std::make_unique<ExternalSynthesis>(ep, cfg);
        }
#if defined(HAS_PFACES_SDK) && HAS_PFACES_SDK
        else if (synth_mode == "direct") {
            DirectSynthesis::Params dp;
            dp.cfg_path          = cfg_path;
            dp.kernel_pack       = kernel_pack;
            dp.device_id         = device_id;
            dp.has_runtime_param = has_runtime_param;
            synth = std::make_unique<DirectSynthesis>(dp);
        }
#endif
        else {
            throw std::runtime_error("Unknown synthesis mode: " + synth_mode);
        }

        // =============================================================
        // 5. Sensor (only if scenario has a measured parameter)
        // =============================================================
        json jsen = J.value("sensor", json::object());
        std::shared_ptr<Sensor> sensor = nullptr;
        double initial_param_value = 0.0;

        if (has_runtime_param) {
            std::string sensor_type = jget<std::string>(jsen, "type", "const");
            initial_param_value = jget<double>(jsen, "value", 0.0);

            if (sensor_type == "step") {
                double step_to   = jget<double>(jsen, "step_to", 6.0);
                double step_time = jget<double>(jsen, "step_time", 5.0);
                sensor = std::make_shared<StepSensor>(initial_param_value, step_to, step_time);
            } else if (sensor_type == "sine") {
                double amp  = jget<double>(jsen, "amplitude", 3.0);
                double freq = jget<double>(jsen, "frequency", 0.1);
                sensor = std::make_shared<SineSensor>(initial_param_value, amp, freq);
            } else {
                sensor = std::make_shared<ConstantSensor>(initial_param_value);
            }

            dynamics->set_param(dynamics->measured_param_name(), initial_param_value);

            if (verbose)
                std::cout << "Sensor:      " << sensor_type
                          << "  " << dynamics->measured_param_name()
                          << "=" << initial_param_value << "\n";
        }

        // =============================================================
        // 6. Initial synthesis
        // =============================================================
        if (verbose) std::cout << "Running initial synthesis (mode=" << synth_mode << ")...\n";
        double synth_ms = synth->synthesize(initial_param_value, safe_set);

        if (verbose) {
            const auto& sd = synth->last_detail();
            std::cout << "  Synthesis: " << std::fixed << std::setprecision(1)
                      << synth_ms << " ms  (" << sd.iterations << " iters)"
                      << "  basis=" << sd.basis_size
                      << "  safe=" << sd.safe_cells << "/" << sd.total_cells
                      << " (" << std::setprecision(1)
                      << (sd.total_cells > 0 ? 100.0 * sd.safe_cells / sd.total_cells : 0.0)
                      << "%)\n";
        }

        if (safe_set.basis_size() == 0 && safe_set.count_safe_cells() == 0)
            throw std::runtime_error("Empty basis — no safe set found.");

        // =============================================================
        // 7. Build cost params (scenario defaults + JSON overrides)
        // =============================================================
        DynamicsModel::CostParams cost_params = dynamics->default_cost_params();

        json jc = J.value("controller", json::object());
        if (jc.contains("cost")) {
            for (auto& [k, v] : jc["cost"].items()) {
                if (v.is_number()) cost_params[k] = v.get<double>();
            }
        }

        if (verbose) {
            std::cout << "Cost:        {";
            bool first = true;
            for (auto& [k, v] : cost_params) {
                if (!first) std::cout << ", ";
                std::cout << k << "=" << v;
                first = false;
            }
            std::cout << "}\n";
        }

        // =============================================================
        // 8. Controller
        // =============================================================
        ControllerConfig ctrl_cfg;
        ctrl_cfg.prediction_horizon = jget<int>(jc, "prediction_horizon", 10);
        ctrl_cfg.control_horizon    = jget<int>(jc, "control_horizon", 5);
        ctrl_cfg.sampling_time      = jget<double>(jc, "sampling_time", cfg.sampling_period);
        ctrl_cfg.ode_substeps       = jget<int>(jc, "ode_substeps", 100);
        ctrl_cfg.mppi_rollouts      = jget<int>(jc, "mppi_rollouts", 512);
        ctrl_cfg.mppi_iterations    = jget<int>(jc, "mppi_iterations", 100);
        ctrl_cfg.mppi_sigma         = jget<double>(jc, "mppi_sigma", 300.0);

        SafeController controller;
        controller.init(ctrl_cfg, dynamics, safe_set,
                        cfg.state_grid, cfg.input_grid, cost_params);

        if (verbose)
            std::cout << "Controller:  MPPI"
                      << "  ph=" << ctrl_cfg.prediction_horizon
                      << "  ch=" << ctrl_cfg.control_horizon
                      << "  iters=" << ctrl_cfg.mppi_iterations
                      << "  rollouts=" << ctrl_cfg.mppi_rollouts
                      << "\n";

        // =============================================================
        // 9. Simulation config
        // =============================================================
        json jsim = J.value("simulation", json::object());

        SimConfig sim_cfg;
        if (jsim.contains("x0")) {
            auto x0_vec = jsim["x0"].get<std::vector<double>>();
            sim_cfg.x0 = Eigen::Map<Vec>(x0_vec.data(), x0_vec.size());
        } else {
            sim_cfg.x0 = Vec(dynamics->nx());
            for (int d = 0; d < dynamics->nx(); ++d)
                sim_cfg.x0[d] = cfg.state_grid.lb[d] + 0.1 *
                    (cfg.state_grid.ub[d] - cfg.state_grid.lb[d]);
        }

        sim_cfg.total_time     = jget<double>(jsim, "duration", 20.0);
        sim_cfg.dt             = jget<double>(jsim, "sampling_period", cfg.sampling_period);
        ctrl_cfg.sampling_time = sim_cfg.dt;
        sim_cfg.ode_steps      = cfg.ode_steps;
        sim_cfg.resynth_thresh = jget<double>(jsim, "resynth_threshold", 0.5);
        sim_cfg.resynth_policy = jget<std::string>(jsim, "resynth_policy",
                                    has_runtime_param ? "threshold" : "never");
        sim_cfg.log_file       = (fs::path(output_dir) / "sim_log.csv").string();
        sim_cfg.verbose        = verbose;

        // =============================================================
        // 10. Run closed-loop simulation
        // =============================================================
        auto synth_ptr = std::shared_ptr<SynthesisBackend>(std::move(synth));
        Simulation sim(dynamics, safe_set, controller, synth_ptr, sensor,
                       cfg.state_grid);

        auto log = sim.run(sim_cfg);

        std::cout << "\nLog written to: " << sim_cfg.log_file
                  << " (" << log.size() << " steps)\n";

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
