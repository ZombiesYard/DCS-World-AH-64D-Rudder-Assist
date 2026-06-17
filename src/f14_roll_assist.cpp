#include "f14_roll_assist.h"

#include "yaw_damper.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace autorudder {
namespace {

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double move_towards(double current, double target, double max_delta) {
    if (target > current) {
        return std::min(target, current + max_delta);
    }
    return std::max(target, current - max_delta);
}

double smoothstep(double edge0, double edge1, double value) {
    const double t = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double with_deadzone(double value, double deadzone) {
    return std::abs(value) < deadzone ? 0.0 : value;
}

double beta_limiter(double slip, double limit, double gain, double sign) {
    const double excess = std::abs(slip) - limit;
    if (excess <= 0.0) {
        return 0.0;
    }
    return -sign * gain * std::copysign(excess, slip);
}

}  // namespace

F14RollAssist::F14RollAssist(AppConfig config) : config_(std::move(config)) {}

void F14RollAssist::set_config(AppConfig config) {
    config_ = std::move(config);
}

void F14RollAssist::reset() {
    rudder_assist_ = 0.0;
    reversal_timer_ = 0.0;
}

F14RollAssistOutput F14RollAssist::update(const F14RollAssistInput& input) {
    const double dt = clamp(input.dt, 0.001, 0.2);
    const double physical_roll = clamp_unit(input.physical_roll);
    const double physical_rudder = clamp_unit(input.physical_rudder);

    const bool telemetry_allowed =
        input.telemetry_fresh &&
        input.aircraft_is_f14 &&
        input.aoa_valid &&
        input.roll_input_valid &&
        input.rudder_input_valid;

    F14RollAssistOutput output;
    output.final_roll = physical_roll;
    output.final_rudder = physical_rudder;
    output.flight_mode = "UNKNOWN";
    output.reason = "assist";

    if (!telemetry_allowed) {
        if (!input.telemetry_fresh) output.reason = "stale telemetry";
        else if (!input.aircraft_is_f14) output.reason = "not F-14";
        else if (!input.aoa_valid) output.reason = "aoa invalid";
        else if (!input.roll_input_valid || !input.rudder_input_valid) output.reason = "input invalid";
        output.final_rudder = 0.0;
        rudder_assist_ = 0.0;
        reversal_timer_ = 0.0;
        return output;
    }

    const double aoa_weight = smoothstep(
        config_.f14_aoa_onset_units,
        config_.f14_aoa_full_units,
        input.aoa_units);
    const double deep_aoa_weight = smoothstep(
        config_.f14_deep_aoa_onset_units,
        config_.f14_deep_aoa_full_units,
        input.aoa_units);
    const double roll_demand = with_deadzone(physical_roll, config_.f14_roll_deadzone);
    const bool on_ground =
        input.indicated_airspeed_valid &&
        input.radar_altitude_valid &&
        input.indicated_airspeed <= config_.f14_ground_ias_threshold &&
        input.radar_altitude <= config_.f14_ground_agl_threshold;
    const bool landing =
        !on_ground &&
        ((input.gear_valid && input.gear_position >= config_.f14_landing_gear_threshold) ||
         (input.flaps_valid && input.flaps_position >= config_.f14_landing_flaps_threshold));
    const bool high_aoa = !on_ground && !landing && aoa_weight > 0.0001;
    const char* flight_mode = on_ground ? "GROUND" : (landing ? "LANDING" : (high_aoa ? "HIGH_AOA_COMBAT" : "NORMAL_FLIGHT"));

    if (!input.assist_enabled) {
        rudder_assist_ = 0.0;
        reversal_timer_ = 0.0;
        output.final_roll = physical_roll;
        output.final_rudder = on_ground ? physical_rudder : 0.0;
        output.aoa_weight = aoa_weight;
        output.deep_aoa_weight = deep_aoa_weight;
        output.flight_mode = flight_mode;
        output.reason = "disabled";
        return output;
    }

    if (on_ground) {
        rudder_assist_ = 0.0;
        reversal_timer_ = 0.0;
        output.final_roll = physical_roll;
        output.final_rudder = physical_rudder;
        output.aoa_weight = aoa_weight;
        output.deep_aoa_weight = deep_aoa_weight;
        output.flight_mode = flight_mode;
        output.reason = "f14 ground";
        return output;
    }

    const double roll_scale = landing
        ? 1.0
        : clamp(
              1.0 - config_.f14_roll_washout * aoa_weight - config_.f14_deep_roll_washout * deep_aoa_weight,
              config_.f14_roll_min_scale,
              1.0);

    bool reversal_candidate = false;
    if (input.roll_rate_valid &&
        high_aoa &&
        aoa_weight >= 0.75 &&
        std::abs(roll_demand) >= config_.f14_reversal_roll_threshold) {
        const double effective_roll_rate = config_.f14_roll_rate_sign * input.roll_rate_x;
        reversal_candidate =
            std::abs(effective_roll_rate) >= config_.f14_reversal_roll_rate_threshold &&
            roll_demand * effective_roll_rate < 0.0;
    }
    if (reversal_candidate) {
        reversal_timer_ = std::min(config_.f14_reversal_guard_time, reversal_timer_ + dt);
    } else {
        reversal_timer_ = std::max(0.0, reversal_timer_ - dt);
    }
    const double reversal_guard =
        reversal_timer_ >= config_.f14_reversal_guard_time ? config_.f14_reversal_guard_scale : 1.0;

    const double scheduled_roll_mix =
        high_aoa
            ? (config_.f14_roll_to_rudder_gain * aoa_weight +
               config_.f14_deep_roll_to_rudder_gain * deep_aoa_weight)
            : config_.f14_low_aoa_roll_coordination_gain;
    const double rudder_from_roll =
        (landing ? 0.0 : 1.0) *
        reversal_guard *
        config_.f14_roll_to_rudder_sign *
        scheduled_roll_mix *
        roll_demand;
    const double roll_intent_weight = clamp(std::abs(roll_demand) / 0.50, 0.0, 1.0);
    const double yaw_damping_scale = high_aoa ? (1.0 - 0.70 * roll_intent_weight) : 1.0;
    const double yaw_damping =
        yaw_damping_scale * -config_.f14_yaw_rate_sign * config_.f14_yaw_rate_gain * input.yaw_rate_z;
    double slip_correction = 0.0;
    if (input.slip_valid) {
        if (high_aoa) {
            slip_correction = beta_limiter(
                input.slip_ball,
                config_.f14_beta_limit,
                config_.f14_beta_limiter_gain,
                config_.f14_slip_sign);
        } else {
            slip_correction = -config_.f14_slip_sign * config_.f14_slip_gain * input.slip_ball;
        }
    }
    const double target_assist = clamp(
        rudder_from_roll + yaw_damping + slip_correction,
        -config_.f14_rudder_max_assist,
        config_.f14_rudder_max_assist);

    if (config_.f14_rudder_rate_limit <= 0.0) {
        rudder_assist_ = target_assist;
    } else {
        rudder_assist_ = move_towards(
            rudder_assist_,
            target_assist,
            config_.f14_rudder_rate_limit * dt);
    }

    output.final_roll = clamp_unit(physical_roll * roll_scale);
    output.final_rudder = clamp_unit(rudder_assist_);
    output.roll_scale = roll_scale;
    output.aoa_weight = aoa_weight;
    output.deep_aoa_weight = deep_aoa_weight;
    output.rudder_assist = rudder_assist_;
    output.rudder_from_roll = rudder_from_roll;
    output.yaw_damping = yaw_damping;
    output.slip_correction = slip_correction;
    output.reversal_guard = reversal_guard;
    output.assist_active =
        aoa_weight > 0.0001 ||
        std::abs(rudder_assist_) > 0.0001 ||
        std::abs(yaw_damping) > 0.0001 ||
        std::abs(slip_correction) > 0.0001;
    output.flight_mode = flight_mode;
    if (landing) output.reason = "f14 landing";
    else if (high_aoa) output.reason = "f14 high-aoa mix";
    else output.reason = "f14 normal";
    return output;
}

}  // namespace autorudder
