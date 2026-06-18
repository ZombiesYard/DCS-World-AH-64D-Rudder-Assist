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

double collective_power_demand(const PowerFeedforwardConfig& config, double collective) {
    double demand = clamp(collective, 0.0, 1.0);
    if (config.collective_lead_invert > 0.5) {
        demand = 1.0 - demand;
    }
    return demand;
}

double apply_collective_lead(
    const PowerFeedforwardConfig& config,
    double power_proxy,
    const PowerFeedforwardInput& input,
    bool& applied) {
    applied = false;
    if (config.collective_lead_gain <= 0.0 || !input.collective) {
        return power_proxy;
    }
    const double demand = collective_power_demand(config, *input.collective);
    const double lead_error = demand - power_proxy - config.collective_lead_deadband;
    if (lead_error <= 0.0) {
        return power_proxy;
    }
    applied = true;
    return clamp(power_proxy + config.collective_lead_gain * lead_error, 0.0, 1.0);
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
        bool collective_lead_applied = false;
        const double value = apply_collective_lead(
            config,
            normalize_fuel_flow(config, *input.fuel_flow),
            input,
            collective_lead_applied);
        return {value, collective_lead_applied ? "fuel_flow_collective_lead" : "fuel_flow"};
    }
    if (config.source == "fuel_rpm") {
        if (!input.fuel_flow) {
            return {std::nullopt, "fuel_rpm_missing"};
        }
        double value = normalize_fuel_flow(config, *input.fuel_flow);
        if (input.rpm) {
            value += rpm_power_correction(config, *input.rpm);
        }
        bool collective_lead_applied = false;
        value = apply_collective_lead(config, clamp(value, 0.0, 1.0), input, collective_lead_applied);
        return {value, collective_lead_applied ? "fuel_rpm_collective_lead" : "fuel_rpm"};
    }
    return {std::nullopt, "invalid"};
}

}  // namespace autorudder
