/**
 * @file scenarios/all.h
 * @brief Include and register all built-in scenarios.
 *
 * Include this header in main.cpp to make all scenarios available via
 * the dynamics factory.  To add a new scenario:
 *   1. Create a new header in this directory (copy an existing one).
 *   2. Include it below.
 *   3. Register it in the ScenarioRegistrar constructor.
 */

#pragma once

#include "turn_ego_first.h"
#include "turn_oncoming_first.h"
#include "acc.h"

namespace rt_ctrl {
namespace {

struct ScenarioRegistrar {
    ScenarioRegistrar() {
        register_dynamics("turn_ego_first", [](const Config&) {
            return std::make_unique<TurnEgoFirstDynamics>();
        });
        register_dynamics("turn_oncoming_first", [](const Config&) {
            return std::make_unique<TurnOncFirstDynamics>();
        });
        register_dynamics("acc", [](const Config&) {
            return std::make_unique<AccDynamics>();
        });
    }
};
static ScenarioRegistrar _scenario_reg;

}  // anonymous namespace
}  // namespace rt_ctrl
