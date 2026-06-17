#include "power_feedforward.h"

#include <algorithm>

namespace autorudder {
namespace {

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double normalize_fuel_flow(const PowerFeedforwardConfig& config, double fuel_flow) {
    return clamp(
        (fuel_flow - config.fuel_flow_min) / (config.fuel_flow_max - config.fuel_flow_min),
        0.0,
        1.0);
}

double rpm_power_correction(const PowerFeedforwardConfig& config, double rpm) {
    const double rpm_drop = clamp(
        (config.rpm_nominal - rpm) / config.rpm_drop_full_scale,
        -1.0,
        1.0);
    return config.rpm_power_gain * rpm_drop;
}

}  // namespace

PowerFeedforwardOutput compute_power_feedforward_input(
    const PowerFeedforwardConfig& config,
    const PowerFeedforwardInput& input) {
    if (config.source == "off") {
        return {std::nullopt, "off"};
    }
    if (config.source == "collective") {
        return {input.collective, input.collective ? "collective" : "collective_missing"};
    }
    if (config.source == "fuel_flow") {
        if (!input.fuel_flow) {
            return {std::nullopt, "fuel_flow_missing"};
        }
        return {normalize_fuel_flow(config, *input.fuel_flow), "fuel_flow"};
    }
    if (config.source == "fuel_rpm") {
        if (!input.fuel_flow) {
            return {std::nullopt, "fuel_rpm_missing"};
        }
        double value = normalize_fuel_flow(config, *input.fuel_flow);
        if (input.rpm) {
            value += rpm_power_correction(config, *input.rpm);
        }
        return {clamp(value, 0.0, 1.0), "fuel_rpm"};
    }
    return {std::nullopt, "invalid"};
}

}  // namespace autorudder
