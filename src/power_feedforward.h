#pragma once

#include <optional>
#include <string>

namespace autorudder {

struct PowerFeedforwardConfig {
    std::string source = "collective";
    double fuel_flow_min = 0.055;
    double fuel_flow_max = 0.155;
    double rpm_nominal = 100.0;
    double rpm_drop_full_scale = 8.0;
    double rpm_power_gain = 0.40;
    double collective_lead_gain = 0.0;
    double collective_lead_invert = 0.0;
    double collective_lead_deadband = 0.0;
};

struct PowerFeedforwardInput {
    std::optional<double> collective;
    std::optional<double> fuel_flow;
    std::optional<double> rpm;
};

struct PowerFeedforwardOutput {
    std::optional<double> value;
    std::string source;
};

PowerFeedforwardOutput compute_power_feedforward_input(
    const PowerFeedforwardConfig& config,
    const PowerFeedforwardInput& input);

}  // namespace autorudder
