/**
 * @file config.h
 * @brief Configuration parser for MonoSafe .cfg files.
 *
 * Parses the key=value format used by pFaces-MonoSynth configuration files
 * and exposes the state/input/disturbance spaces, dynamics parameters, and
 * solver settings in a structured C++ interface.
 */

#pragma once

#include "types.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace rt_ctrl {

// ---------------------------------------------------------------------------
// Config: parsed representation of a .cfg file
// ---------------------------------------------------------------------------
struct Config {
    // --- project ---
    std::string project_name;
    std::string dynamics_file;

    // --- state space ---
    GridDesc state_grid;
    std::vector<int>    priorities;  ///< Per-dimension priority for monotone order

    // --- input space ---
    GridDesc input_grid;

    // --- solver ---
    double sampling_period  = 0.5;
    int    ode_steps        = 1000;
    int    max_basis        = 10000;
    bool   save_controller  = true;
    bool   record_evolution = false;

    // --- dynamics parameters (user-defined, from .cl or inline) ---
    std::map<std::string, double> dyn_params;

    // --- raw key-value store for anything else ---
    std::map<std::string, std::string> raw;
};

// ---------------------------------------------------------------------------
// Helper: strip quotes and whitespace
// ---------------------------------------------------------------------------
namespace detail {

inline std::string strip(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n\"");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n\";");
    return s.substr(start, end - start + 1);
}

inline std::vector<double> parse_csv_doubles(const std::string& s) {
    std::vector<double> v;
    std::stringstream ss(strip(s));
    std::string token;
    while (std::getline(ss, token, ',')) {
        v.push_back(std::stod(strip(token)));
    }
    return v;
}

inline std::vector<int> parse_csv_ints(const std::string& s) {
    std::vector<int> v;
    std::stringstream ss(strip(s));
    std::string token;
    while (std::getline(ss, token, ',')) {
        v.push_back(std::stoi(strip(token)));
    }
    return v;
}

inline GridDesc build_grid(int dim,
                           const std::vector<double>& eta,
                           const std::vector<double>& lb,
                           const std::vector<double>& ub) {
    GridDesc g;
    g.n_dim = dim;
    g.eta   = eta;
    g.lb    = lb;
    g.ub    = ub;
    g.sizes.resize(dim);
    for (int d = 0; d < dim; ++d) {
        g.sizes[d] = static_cast<int>(std::llround((ub[d] - lb[d]) / eta[d])) + 1;
    }
    g.recompute_total();
    return g;
}

}  // namespace detail

// ---------------------------------------------------------------------------
// parse_config: load a .cfg file
// ---------------------------------------------------------------------------
inline Config parse_config(const std::string& path) {
    Config cfg;

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open config: " + path);
    }

    // Flat key-value map (handles nested blocks by prefixing with block name)
    std::map<std::string, std::string> kv;
    std::string line;
    std::string current_block;

    while (std::getline(ifs, line)) {
        // Remove comments
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);

        std::string trimmed = detail::strip(line);
        if (trimmed.empty()) continue;

        // Block open: "states{" or "inputs{"
        if (trimmed.back() == '{') {
            current_block = detail::strip(trimmed.substr(0, trimmed.size() - 1));
            continue;
        }
        // Block close
        if (trimmed.front() == '}') {
            current_block.clear();
            continue;
        }

        // Key = value
        auto eq = trimmed.find('=');
        if (eq == std::string::npos) continue;

        std::string key = detail::strip(trimmed.substr(0, eq));
        std::string val = detail::strip(trimmed.substr(eq + 1));

        std::string full_key = current_block.empty() ? key : current_block + "." + key;
        kv[full_key] = val;
    }

    // Store raw
    cfg.raw = kv;

    // --- project ---
    if (kv.count("project_name"))      cfg.project_name  = detail::strip(kv["project_name"]);
    if (kv.count("user_dynamics_file")) cfg.dynamics_file = detail::strip(kv["user_dynamics_file"]);

    // --- solver params ---
    if (kv.count("sampling_period"))       cfg.sampling_period  = std::stod(detail::strip(kv["sampling_period"]));
    if (kv.count("ode_steps"))             cfg.ode_steps        = std::stoi(detail::strip(kv["ode_steps"]));
    if (kv.count("max_basis_elements"))    cfg.max_basis        = std::stoi(detail::strip(kv["max_basis_elements"]));
    if (kv.count("save_controller"))       cfg.save_controller  = detail::strip(kv["save_controller"]) == "true";
    if (kv.count("record_basis_evolution")) cfg.record_evolution = detail::strip(kv["record_basis_evolution"]) == "true";

    // --- state space ---
    int s_dim = kv.count("states.dim") ? std::stoi(detail::strip(kv["states.dim"])) : 0;
    if (s_dim > 0) {
        auto eta = detail::parse_csv_doubles(kv["states.eta"]);
        auto lb  = detail::parse_csv_doubles(kv["states.lb"]);
        auto ub  = detail::parse_csv_doubles(kv["states.ub"]);
        cfg.state_grid = detail::build_grid(s_dim, eta, lb, ub);

        if (kv.count("states.priorities"))
            cfg.priorities = detail::parse_csv_ints(kv["states.priorities"]);
    }

    // --- input space ---
    int u_dim = kv.count("inputs.dim") ? std::stoi(detail::strip(kv["inputs.dim"])) : 0;
    if (u_dim > 0) {
        auto eta = detail::parse_csv_doubles(kv["inputs.eta"]);
        auto lb  = detail::parse_csv_doubles(kv["inputs.lb"]);
        auto ub  = detail::parse_csv_doubles(kv["inputs.ub"]);
        cfg.input_grid = detail::build_grid(u_dim, eta, lb, ub);
    }

    return cfg;
}

}  // namespace rt_ctrl
