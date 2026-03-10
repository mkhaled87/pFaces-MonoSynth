# MonoSafe Real-Time Controller Framework

A general-purpose C++20 framework for **real-time safe control** using monotone safe sets computed by pFaces-MonoSynth. Combines parallel safe-set synthesis with online MPPI control to maintain safety guarantees while tracking performance objectives.

## Quick Start

```bash
# Build
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_PFACES_SDK=ON
make -j$(sysctl -n hw.logicalcpu 2>/dev/null || nproc)
cd ..

# Run an experiment (builds, runs, visualizes — all outputs in one folder)
./scripts/run_experiment.sh examples/turn_ego_first.json

# Or run manually:
./build/rt_controller examples/turn_ego_first.json experiments/my_run
python3 scripts/visualize.py experiments/my_run/sim_log.csv --save experiments/my_run
```

## Architecture

```
┌─────────────┐      ┌────────────┐      ┌──────────────┐
│  Synthesis   │─────▶│  Safe Set   │◀────│  Controller  │
│  Backend     │      │  (Bitmap)   │      │  (MPPI)      │
└──────┬──────┘      └──────┬──────┘      └──────┬───────┘
       │                     │                    │
       │    ┌────────────────┴─────────────┐     │
       └───▶│     Simulation Loop          │◀────┘
            │  sense → synth → ctrl → sim  │
            └──────────────┬───────────────┘
                           │
                    ┌──────▼──────┐
                    │  CSV Log    │──▶  visualize.py
                    └─────────────┘
```

## JSON Configuration

**Everything** is controlled by a single `.json` file. No CLI flags.

```jsonc
{
  "cfg_file": "../../pFaces-MonoSynth/examples/turn_ego_first/turn_ego_first.cfg",

  "synthesis": {
    "mode": "direct",                    // "file" | "external" | "direct"
    "kernel_pack": "../../kernel-pack",
    "device_id": 0                       // GPU=0, CPU=1 (platform-dependent)
  },

  "controller": {
    "prediction_horizon": 10,
    "control_horizon": 5,
    "mppi_iterations": 100,
    "mppi_rollouts": 512,
    "mppi_sigma": 300.0,
    "cost": {                            // Scenario-specific cost weights
      "w_goal": 50.0,
      "w_speed": 20.0,
      "goal": -5.0
    }
  },

  "simulation": {
    "x0": [-35.0, 0.0, -60.0],
    "duration": 6.0,
    "sampling_period": 0.1,
    "resynth_policy": "always"           // "never" | "threshold" | "always"
  },

  "sensor": {                            // Omit entirely if no runtime param
    "type": "sine",                      // "const" | "step" | "sine"
    "value": 8.0,
    "amplitude": 3.0,
    "frequency": 0.15
  }
}
```

**Key rules:**
- `cfg_file` must be a **relative path** from the rt_controller root to the pFaces `.cfg` file
- `controller.cost` is a flat dict of `key: value` — merged over scenario defaults
- The `sensor` block is only needed if the scenario has a measured runtime parameter
- No `output` path: the output directory is set by the experiment script or CLI arg

## Built-in Scenarios

| Scenario | States | Input | Runtime Param | `.cfg` Location |
|----------|--------|-------|---------------|-----------------|
| `turn_ego_first` | s_ego, v_ego, s_onc | Torque (N·m) | v_oncoming | `pFaces-MonoSynth/examples/turn_ego_first/` |
| `turn_oncoming_first` | s_ego, v_ego, s_onc | Torque (N·m) | v_oncoming | `pFaces-MonoSynth/examples/turn_oncoming_first/` |
| `acc` | h, v_ego, v_lead | Accel (m/s²) | — | `pFaces-MonoSynth/examples/acc/` |

## Running Experiments

The recommended workflow uses `run_experiment.sh`, which:

1. Builds the project
2. Creates `experiments/<scenario>_<YYYYMMDD_HHMMSS>/`
3. Copies JSON + CFG configs into the experiment folder
4. Runs the simulation → `sim_log.csv`
5. Generates all plots + optional animation

```bash
# Full pipeline (with animation)
./scripts/run_experiment.sh examples/turn_ego_first.json

# Skip animation (faster)
./scripts/run_experiment.sh examples/acc.json --no-animate
```

Each run produces a self-contained folder:
```
experiments/turn_ego_first_20250701_143022/
├── config.json           # Copy of the JSON config used
├── turn_ego_first.cfg    # Copy of the pFaces .cfg used
├── sim_log.csv           # Simulation log
├── run.log               # Terminal output
├── states.png            # State trajectories
├── controls.png          # Control inputs
├── phase.png             # Phase-plane projections
├── timing.png            # Computation time breakdown
├── param_safety.png      # Parameter & safety panel
├── summary.png           # Summary statistics table
└── intersection.mp4      # Animation (if --animate)
```

## Adding a New Scenario

Follow these 4 steps:

### Step 1: Create the dynamics header

Create `include/rt_controller/scenarios/my_scenario.h`:

```cpp
#pragma once
#include "rt_controller/dynamics.h"

namespace rt_ctrl {

class MyScenarioDynamics : public DynamicsModel {
public:
    MyScenarioDynamics(const Config& cfg) {
        nx_ = 3;  nu_ = 1;  ny_ = 3;
        // Read params from cfg.dyn_params if needed
    }

    std::string name() const override { return "my_scenario"; }

    std::vector<std::string> state_names() const override {
        return {"x1", "x2", "x3"};
    }

    std::vector<std::string> input_names() const override {
        return {"u"};
    }

    // Return "" if no runtime parameter, or the param name for resynthesis
    std::string measured_param_name() const override { return ""; }

    // Must match the pFaces kernel #define (leave empty if not applicable)
    std::string synthesis_macro() const override { return "MY_SCENARIO"; }

    void ode(const double* x, const double* u, double* dx) const override {
        dx[0] = /* ... */;
        dx[1] = /* ... */;
        dx[2] = /* ... */;
    }

    void apply_constraints(double* x, const GridDesc& grid) const override {
        for (int d = 0; d < nx_; ++d)
            x[d] = std::clamp(x[d], grid.lb[d], grid.ub[d]);
    }

    void output(const double* x, double* y) const override {
        for (int d = 0; d < nx_; ++d) y[d] = x[d];
    }

    CostParams default_cost_params() const override {
        return {{"w_goal", 50.0}, {"w_speed", 20.0}, {"goal", 0.0}};
    }

    // Build the MPPI objective function
    std::function<double(const Vec&, const Vec&)>
    make_objective(const SafeSet& ss, const CostParams& p) const override {
        double w_goal  = p.at("w_goal");
        double w_speed = p.at("w_speed");
        double goal    = p.at("goal");
        return [=, &ss](const Vec& x, const Vec& u) -> double {
            double cost = w_goal * (x[0] - goal) * (x[0] - goal);
            cost += w_speed * x[1] * x[1];
            if (!ss.is_safe_state(x)) cost += 1e6;
            return cost;
        };
    }
};

// Factory registration
static bool _reg_my_scenario = DynamicsFactory::instance().register_dynamics(
    "my_scenario", [](const Config& c) -> DynamicsModel* {
        return new MyScenarioDynamics(c);
    });

}  // namespace rt_ctrl
```

### Step 2: Register in all.h

Edit `include/rt_controller/scenarios/all.h` — add one include:

```cpp
#include "rt_controller/scenarios/my_scenario.h"
```

### Step 3: Create a JSON config

Create `examples/my_scenario.json`:

```json
{
  "cfg_file": "../../pFaces-MonoSynth/examples/my_scenario/my_scenario.cfg",
  "synthesis": {
    "mode": "direct",
    "kernel_pack": "../../kernel-pack",
    "device_id": 0
  },
  "controller": {
    "prediction_horizon": 10,
    "control_horizon": 5,
    "mppi_rollouts": 512,
    "mppi_sigma": 300.0,
    "cost": {}
  },
  "simulation": {
    "x0": [0.0, 0.0, 0.0],
    "duration": 20.0,
    "sampling_period": 0.5,
    "resynth_policy": "never"
  }
}
```

### Step 4: Rebuild and run

```bash
./scripts/run_experiment.sh examples/my_scenario.json
```

That's it. The framework auto-detects the scenario from `cfg_file`'s `project_name`.

## Visualization

```bash
# Interactive
python3 scripts/visualize.py sim_log.csv

# Save to directory
python3 scripts/visualize.py sim_log.csv --save figures/

# With animation (for turn scenarios)
python3 scripts/visualize.py sim_log.csv --save figures/ --animate

# Dark mode
python3 scripts/visualize.py sim_log.csv --dark
```

Generated plots:
1. **State trajectories** — time series with safe/unsafe shading
2. **Control inputs** — ZOH step plots
3. **Phase-plane** — 2D state-space projections colored by safety
4. **Timing** — per-step computation time breakdown
5. **Parameter & safety** — measured param, certification status, basis size
6. **Summary table** — statistics (safety rate, timing percentiles)
7. **Animation** — top-down intersection MP4 (optional, `--animate`)

## CSV Log Format

```
time,x0,x1,...,u0,...,param_value,is_safe,query_ns,synth_ms,ctrl_ms,basis_size,safe_frac
```

| Column | Description |
|--------|-------------|
| `time` | Simulation time (s) |
| `x0..x{n-1}` | State variables |
| `u0..u{m-1}` | Control inputs |
| `param_value` | Measured runtime parameter (0 if none) |
| `is_safe` | Safety query result (0/1) |
| `query_ns` | Safety query time (nanoseconds) |
| `synth_ms` | Synthesis time — 0 if no resynthesis this step |
| `ctrl_ms` | MPPI controller solve time (ms) |
| `basis_size` | Current basis cardinality |
| `safe_frac` | Fraction of state space certified safe |

## Dependencies

| Dependency | Required | Notes |
|-----------|----------|-------|
| C++20 compiler | Yes | GCC ≥10 / Clang ≥14 |
| Eigen3 | Yes | FetchContent (auto) |
| libmpc++ | Yes | FetchContent (auto) |
| NLopt | Yes | `brew install nlopt` |
| pFaces SDK | Optional | For `direct` synthesis mode |
| Python 3.8+ | For viz | `pip install matplotlib pandas numpy` |

## Project Structure

```
tools/rt_controller/
├── CMakeLists.txt
├── README.md
├── include/rt_controller/
│   ├── types.h                    # Grid, indexing, log entries
│   ├── config.h                   # .cfg parser
│   ├── dynamics.h                 # Abstract dynamics + factory
│   ├── safe_set.h                 # Bitmap safe set (O(1) queries)
│   ├── synthesis.h                # file / external / direct backends
│   ├── controller.h               # MPPI controller
│   ├── simulation.h               # Closed-loop simulation
│   └── scenarios/
│       ├── all.h                  # Includes all scenarios
│       ├── turn_ego_first.h
│       ├── turn_oncoming_first.h
│       └── acc.h
├── src/main.cpp                   # Entry point
├── scripts/
│   ├── run_experiment.sh          # Full experiment pipeline
│   └── visualize.py               # Post-hoc visualization
├── examples/
│   ├── turn_ego_first.json
│   ├── turn_oncoming_first.json
│   └── acc.json
└── experiments/                   # Created per-run (gitignored)
    └── <scenario>_<timestamp>/
```