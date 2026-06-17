#include "config.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace autorudder {
namespace {

std::string trim(std::string value) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

bool starts_with(std::string_view text, std::string_view prefix) {
    return text.size() >= prefix.size() && text.substr(0, prefix.size()) == prefix;
}

int parse_int(const std::string& value, const std::string& key) {
    try {
        return std::stoi(value);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid integer for key '" + key + "': " + value);
    }
}

double parse_double(const std::string& value, const std::string& key) {
    try {
        return std::stod(value);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid number for key '" + key + "': " + value);
    }
}

void apply_key(AppConfig& cfg, const std::string& key, const std::string& value) {
    if (key == "heading_hold_leak_time") {
        cfg.heading_hold_leak_time = parse_double(value, key);
        return;
    }
    if (key == "yaw_rate_error_exponent") {
        cfg.yaw_rate_error_exponent = parse_double(value, key);
        return;
    }
    if (key == "yaw_rate_integral_gain") {
        cfg.yaw_rate_integral_gain = parse_double(value, key);
        return;
    }
    if (key == "yaw_rate_integral_limit") {
        cfg.yaw_rate_integral_limit = parse_double(value, key);
        return;
    }
    if (key == "yaw_rate_integral_leak") {
        cfg.yaw_rate_integral_leak = parse_double(value, key);
        return;
    }
    if (key == "yaw_rate_integral_deadband") {
        cfg.yaw_rate_integral_deadband = parse_double(value, key);
        return;
    }
    if (key == "yaw_rate_source") {
        cfg.yaw_rate_source = value;
        return;
    }
    if (key == "power_feedforward_source") {
        cfg.power_feedforward_source = value;
        return;
    }
    if (key == "fuel_flow_min") {
        cfg.fuel_flow_min = parse_double(value, key);
        return;
    }
    if (key == "fuel_flow_max") {
        cfg.fuel_flow_max = parse_double(value, key);
        return;
    }
    if (key == "rpm_nominal") {
        cfg.rpm_nominal = parse_double(value, key);
        return;
    }
    if (key == "rpm_drop_full_scale") {
        cfg.rpm_drop_full_scale = parse_double(value, key);
        return;
    }
    if (key == "rpm_power_gain") {
        cfg.rpm_power_gain = parse_double(value, key);
        return;
    }

    if (key == "dcs_bios_path") cfg.dcs_bios_path = value;
    else if (key == "multicast_address") cfg.multicast_address = value;
    else if (key == "multicast_interface") cfg.multicast_interface = value;
    else if (key == "udp_port") cfg.udp_port = parse_int(value, key);
    else if (key == "telemetry_source") cfg.telemetry_source = value;
    else if (key == "fast_export_bind_address") cfg.fast_export_bind_address = value;
    else if (key == "fast_export_port") cfg.fast_export_port = parse_int(value, key);
    else if (key == "input_vjoy_id") cfg.input_vjoy_id = parse_int(value, key);
    else if (key == "output_vjoy_id") cfg.output_vjoy_id = parse_int(value, key);
    else if (key == "input_device_name_contains") cfg.input_device_name_contains = value;
    else if (key == "axis_name") cfg.axis_name = value;
    else if (key == "rudder_input_center") cfg.rudder_input_center = parse_double(value, key);
    else if (key == "rudder_input_deadzone") cfg.rudder_input_deadzone = parse_double(value, key);
    else if (key == "rudder_input_scale") cfg.rudder_input_scale = parse_double(value, key);
    else if (key == "control_mode") cfg.control_mode = value;
    else if (key == "assist_sign") cfg.assist_sign = parse_double(value, key);
    else if (key == "yaw_response_sign") cfg.yaw_response_sign = parse_double(value, key);
    else if (key == "yaw_rate_sign") cfg.yaw_rate_sign = parse_double(value, key);
    else if (key == "kp") cfg.kp = parse_double(value, key);
    else if (key == "ki") cfg.ki = parse_double(value, key);
    else if (key == "integral_limit") cfg.integral_limit = parse_double(value, key);
    else if (key == "max_assist") cfg.max_assist = parse_double(value, key);
    else if (key == "heading_hold_max_assist") cfg.heading_hold_max_assist = parse_double(value, key);
    else if (key == "release_brake_time") cfg.release_brake_time = parse_double(value, key);
    else if (key == "release_brake_kp") cfg.release_brake_kp = parse_double(value, key);
    else if (key == "release_brake_max_assist") cfg.release_brake_max_assist = parse_double(value, key);
    else if (key == "yaw_rate_deadband") cfg.yaw_rate_deadband = parse_double(value, key);
    else if (key == "yaw_accel_gain") cfg.yaw_accel_gain = parse_double(value, key);
    else if (key == "yaw_accel_filter_time") cfg.yaw_accel_filter_time = parse_double(value, key);
    else if (key == "yaw_accel_limit") cfg.yaw_accel_limit = parse_double(value, key);
    else if (key == "heading_error_deadband") cfg.heading_error_deadband = parse_double(value, key);
    else if (key == "heading_error_relock_threshold") cfg.heading_error_relock_threshold = parse_double(value, key);
    else if (key == "heading_kp") cfg.heading_kp = parse_double(value, key);
    else if (key == "heading_rate_limit") cfg.heading_rate_limit = parse_double(value, key);
    else if (key == "turn_rate_max") cfg.turn_rate_max = parse_double(value, key);
    else if (key == "pedal_command_sign") cfg.pedal_command_sign = parse_double(value, key);
    else if (key == "pedal_command_deadzone") cfg.pedal_command_deadzone = parse_double(value, key);
    else if (key == "pedal_command_exit_deadzone") cfg.pedal_command_exit_deadzone = parse_double(value, key);
    else if (key == "pedal_override_threshold") cfg.pedal_override_threshold = parse_double(value, key);
    else if (key == "pedal_rate_override_threshold") cfg.pedal_rate_override_threshold = parse_double(value, key);
    else if (key == "trim_capture_enabled") cfg.trim_capture_enabled = parse_double(value, key);
    else if (key == "trim_capture_min_pedal") cfg.trim_capture_min_pedal = parse_double(value, key);
    else if (key == "trim_capture_yaw_rate") cfg.trim_capture_yaw_rate = parse_double(value, key);
    else if (key == "trim_capture_pedal_rate") cfg.trim_capture_pedal_rate = parse_double(value, key);
    else if (key == "trim_guard_enabled") cfg.trim_guard_enabled = parse_double(value, key);
    else if (key == "trim_guard_input_id") cfg.trim_guard_input_id = parse_int(value, key);
    else if (key == "trim_guard_input_device_name_contains") cfg.trim_guard_input_device_name_contains = value;
    else if (key == "trim_guard_input_button") cfg.trim_guard_input_button = parse_int(value, key);
    else if (key == "trim_guard_output_button") cfg.trim_guard_output_button = parse_int(value, key);
    else if (key == "trim_guard_pre_time") cfg.trim_guard_pre_time = parse_double(value, key);
    else if (key == "trim_guard_press_time") cfg.trim_guard_press_time = parse_double(value, key);
    else if (key == "trim_guard_post_time") cfg.trim_guard_post_time = parse_double(value, key);
    else if (key == "collective_source") cfg.collective_source = value;
    else if (key == "collective_input_id") cfg.collective_input_id = parse_int(value, key);
    else if (key == "collective_device_name_contains") cfg.collective_device_name_contains = value;
    else if (key == "collective_axis_name") cfg.collective_axis_name = value;
    else if (key == "collective_invert") cfg.collective_invert = parse_double(value, key);
    else if (key == "collective_sign") cfg.collective_sign = parse_double(value, key);
    else if (key == "collective_feedforward_mode") cfg.collective_feedforward_mode = value;
    else if (key == "collective_gain") cfg.collective_gain = parse_double(value, key);
    else if (key == "collective_zero_thrust") cfg.collective_zero_thrust = parse_double(value, key);
    else if (key == "collective_power_exponent") cfg.collective_power_exponent = parse_double(value, key);
    else if (key == "collective_rate_gain") cfg.collective_rate_gain = parse_double(value, key);
    else if (key == "collective_rate_limit") cfg.collective_rate_limit = parse_double(value, key);
    else if (key == "collective_transient_rate_threshold") cfg.collective_transient_rate_threshold = parse_double(value, key);
    else if (key == "collective_rate_hold_time") cfg.collective_rate_hold_time = parse_double(value, key);
    else if (key == "collective_rate_reverse_threshold") cfg.collective_rate_reverse_threshold = parse_double(value, key);
    else if (key == "collective_rate_reverse_slew_time") cfg.collective_rate_reverse_slew_time = parse_double(value, key);
    else if (key == "collective_transient_fade_time") cfg.collective_transient_fade_time = parse_double(value, key);
    else if (key == "collective_output_axis_name") cfg.collective_output_axis_name = value;
    else if (key == "collective_output_invert") cfg.collective_output_invert = parse_double(value, key);
    else if (key == "auto_tune_collective_drive") cfg.auto_tune_collective_drive = parse_double(value, key);
    else if (key == "auto_tune_collective_amplitude") cfg.auto_tune_collective_amplitude = parse_double(value, key);
    else if (key == "auto_tune_collective_period") cfg.auto_tune_collective_period = parse_double(value, key);
    else if (key == "auto_tune_collective_settle_time") cfg.auto_tune_collective_settle_time = parse_double(value, key);
    else if (key == "auto_tune_collective_rate_limit") cfg.auto_tune_collective_rate_limit = parse_double(value, key);
    else if (key == "fade_in_time") cfg.fade_in_time = parse_double(value, key);
    else if (key == "fade_out_time") cfg.fade_out_time = parse_double(value, key);
    else if (key == "filter_time") cfg.filter_time = parse_double(value, key);
    else if (key == "stale_timeout") cfg.stale_timeout = parse_double(value, key);
    else if (key == "loop_hz") cfg.loop_hz = parse_int(value, key);
    else if (key == "f14_roll_axis_name") cfg.f14_roll_axis_name = value;
    else if (key == "f14_rudder_axis_name") cfg.f14_rudder_axis_name = value;
    else if (key == "f14_roll_input_id") cfg.f14_roll_input_id = parse_int(value, key);
    else if (key == "f14_roll_device_name_contains") cfg.f14_roll_device_name_contains = value;
    else if (key == "f14_rudder_input_id") cfg.f14_rudder_input_id = parse_int(value, key);
    else if (key == "f14_rudder_device_name_contains") cfg.f14_rudder_device_name_contains = value;
    else if (key == "f14_output_roll_axis_name") cfg.f14_output_roll_axis_name = value;
    else if (key == "f14_output_rudder_axis_name") cfg.f14_output_rudder_axis_name = value;
    else if (key == "f14_aoa_units_offset") cfg.f14_aoa_units_offset = parse_double(value, key);
    else if (key == "f14_aoa_units_per_radian") cfg.f14_aoa_units_per_radian = parse_double(value, key);
    else if (key == "f14_aoa_onset_units") cfg.f14_aoa_onset_units = parse_double(value, key);
    else if (key == "f14_aoa_full_units") cfg.f14_aoa_full_units = parse_double(value, key);
    else if (key == "f14_deep_aoa_onset_units") cfg.f14_deep_aoa_onset_units = parse_double(value, key);
    else if (key == "f14_deep_aoa_full_units") cfg.f14_deep_aoa_full_units = parse_double(value, key);
    else if (key == "f14_roll_deadzone") cfg.f14_roll_deadzone = parse_double(value, key);
    else if (key == "f14_roll_washout") cfg.f14_roll_washout = parse_double(value, key);
    else if (key == "f14_deep_roll_washout") cfg.f14_deep_roll_washout = parse_double(value, key);
    else if (key == "f14_roll_min_scale") cfg.f14_roll_min_scale = parse_double(value, key);
    else if (key == "f14_roll_to_rudder_gain") cfg.f14_roll_to_rudder_gain = parse_double(value, key);
    else if (key == "f14_deep_roll_to_rudder_gain") cfg.f14_deep_roll_to_rudder_gain = parse_double(value, key);
    else if (key == "f14_low_aoa_roll_coordination_gain") cfg.f14_low_aoa_roll_coordination_gain = parse_double(value, key);
    else if (key == "f14_roll_to_rudder_sign") cfg.f14_roll_to_rudder_sign = parse_double(value, key);
    else if (key == "f14_yaw_rate_gain") cfg.f14_yaw_rate_gain = parse_double(value, key);
    else if (key == "f14_yaw_rate_sign") cfg.f14_yaw_rate_sign = parse_double(value, key);
    else if (key == "f14_slip_gain") cfg.f14_slip_gain = parse_double(value, key);
    else if (key == "f14_slip_sign") cfg.f14_slip_sign = parse_double(value, key);
    else if (key == "f14_beta_limit") cfg.f14_beta_limit = parse_double(value, key);
    else if (key == "f14_beta_limiter_gain") cfg.f14_beta_limiter_gain = parse_double(value, key);
    else if (key == "f14_rudder_max_assist") cfg.f14_rudder_max_assist = parse_double(value, key);
    else if (key == "f14_rudder_rate_limit") cfg.f14_rudder_rate_limit = parse_double(value, key);
    else if (key == "f14_ground_ias_threshold") cfg.f14_ground_ias_threshold = parse_double(value, key);
    else if (key == "f14_ground_agl_threshold") cfg.f14_ground_agl_threshold = parse_double(value, key);
    else if (key == "f14_landing_gear_threshold") cfg.f14_landing_gear_threshold = parse_double(value, key);
    else if (key == "f14_landing_flaps_threshold") cfg.f14_landing_flaps_threshold = parse_double(value, key);
    else if (key == "f14_roll_rate_sign") cfg.f14_roll_rate_sign = parse_double(value, key);
    else if (key == "f14_reversal_guard_time") cfg.f14_reversal_guard_time = parse_double(value, key);
    else if (key == "f14_reversal_roll_threshold") cfg.f14_reversal_roll_threshold = parse_double(value, key);
    else if (key == "f14_reversal_roll_rate_threshold") cfg.f14_reversal_roll_rate_threshold = parse_double(value, key);
    else if (key == "f14_reversal_guard_scale") cfg.f14_reversal_guard_scale = parse_double(value, key);
    else if (key == "hotkey") cfg.hotkey = value;
    else if (key == "log_path") cfg.log_path = value;
    else if (key == "calibration_max_assist") cfg.calibration_max_assist = parse_double(value, key);
}

void validate(const AppConfig& cfg) {
    if (cfg.udp_port <= 0 || cfg.udp_port > 65535) throw std::runtime_error("udp_port out of range");
    if (cfg.fast_export_port <= 0 || cfg.fast_export_port > 65535) throw std::runtime_error("fast_export_port out of range");
    if (cfg.telemetry_source != "dcs_bios" && cfg.telemetry_source != "fast_export") {
        throw std::runtime_error("telemetry_source must be dcs_bios or fast_export");
    }
    if (cfg.yaw_rate_source != "heading" &&
        cfg.yaw_rate_source != "angular" &&
        cfg.yaw_rate_source != "auto") {
        throw std::runtime_error("yaw_rate_source must be heading, angular, or auto");
    }
    if (cfg.collective_source != "off" &&
        cfg.collective_source != "auto" &&
        cfg.collective_source != "fast_export" &&
        cfg.collective_source != "directinput" &&
        cfg.collective_source != "xinput") {
        throw std::runtime_error("collective_source must be off, auto, fast_export, directinput, or xinput");
    }
    if (cfg.control_mode != "yaw_damper" &&
        cfg.control_mode != "heading_hold" &&
        cfg.control_mode != "heading_command" &&
        cfg.control_mode != "f14_roll_assist") {
        throw std::runtime_error("control_mode must be yaw_damper, heading_hold, heading_command, or f14_roll_assist");
    }
    if (cfg.input_vjoy_id <= 0 || cfg.output_vjoy_id <= 0) throw std::runtime_error("vJoy IDs must be positive");
    if (cfg.rudder_input_center < -0.95 || cfg.rudder_input_center > 0.95) {
        throw std::runtime_error("rudder_input_center must be in [-0.95, 0.95]");
    }
    if (cfg.rudder_input_deadzone < 0.0 || cfg.rudder_input_deadzone > 0.95) {
        throw std::runtime_error("rudder_input_deadzone must be in [0, 0.95]");
    }
    if (cfg.rudder_input_scale <= 0.0 || cfg.rudder_input_scale > 4.0) {
        throw std::runtime_error("rudder_input_scale must be in (0, 4]");
    }
    if (cfg.collective_input_id <= 0) throw std::runtime_error("collective_input_id must be positive");
    if (cfg.loop_hz < 20 || cfg.loop_hz > 500) throw std::runtime_error("loop_hz must be between 20 and 500");
    if (cfg.ki < 0.0) throw std::runtime_error("ki must be non-negative");
    if (cfg.integral_limit < 0.0 || cfg.integral_limit > 1.0) {
        throw std::runtime_error("integral_limit must be in [0, 1]");
    }
    if (cfg.max_assist < 0.0 || cfg.max_assist > 1.0) throw std::runtime_error("max_assist must be in [0, 1]");
    if (cfg.heading_hold_max_assist < 0.0 || cfg.heading_hold_max_assist > cfg.max_assist) {
        throw std::runtime_error("heading_hold_max_assist must be in [0, max_assist]");
    }
    if (cfg.release_brake_time < 0.0) throw std::runtime_error("release_brake_time must be non-negative");
    if (cfg.release_brake_kp < 0.0) throw std::runtime_error("release_brake_kp must be non-negative");
    if (cfg.release_brake_max_assist < 0.0 || cfg.release_brake_max_assist > cfg.max_assist) {
        throw std::runtime_error("release_brake_max_assist must be in [0, max_assist]");
    }
    if (cfg.yaw_rate_error_exponent <= 0.0 || cfg.yaw_rate_error_exponent > 2.0) {
        throw std::runtime_error("yaw_rate_error_exponent must be in (0, 2]");
    }
    if (cfg.yaw_rate_integral_gain < 0.0) {
        throw std::runtime_error("yaw_rate_integral_gain must be non-negative");
    }
    if (cfg.yaw_rate_integral_limit < 0.0 || cfg.yaw_rate_integral_limit > 1.0) {
        throw std::runtime_error("yaw_rate_integral_limit must be in [0, 1]");
    }
    if (cfg.yaw_rate_integral_leak < 0.0) {
        throw std::runtime_error("yaw_rate_integral_leak must be non-negative");
    }
    if (cfg.yaw_rate_integral_deadband < 0.0) {
        throw std::runtime_error("yaw_rate_integral_deadband must be non-negative");
    }
    if (cfg.yaw_accel_gain < 0.0) throw std::runtime_error("yaw_accel_gain must be non-negative");
    if (cfg.yaw_accel_filter_time < 0.0) {
        throw std::runtime_error("yaw_accel_filter_time must be non-negative");
    }
    if (cfg.yaw_accel_limit < 0.0 || cfg.yaw_accel_limit > 1.0) {
        throw std::runtime_error("yaw_accel_limit must be in [0, 1]");
    }
    if (cfg.heading_error_deadband < 0.0) throw std::runtime_error("heading_error_deadband must be non-negative");
    if (cfg.heading_error_relock_threshold < 0.0) {
        throw std::runtime_error("heading_error_relock_threshold must be non-negative");
    }
    if (cfg.heading_hold_leak_time < 0.0) throw std::runtime_error("heading_hold_leak_time must be non-negative");
    if (cfg.heading_kp < 0.0) throw std::runtime_error("heading_kp must be non-negative");
    if (cfg.heading_rate_limit < 0.0) throw std::runtime_error("heading_rate_limit must be non-negative");
    if (cfg.turn_rate_max < 0.0) throw std::runtime_error("turn_rate_max must be non-negative");
    if (cfg.pedal_command_deadzone < 0.0 || cfg.pedal_command_deadzone > 1.0) {
        throw std::runtime_error("pedal_command_deadzone must be in [0, 1]");
    }
    if (cfg.pedal_command_exit_deadzone < 0.0 || cfg.pedal_command_exit_deadzone > cfg.pedal_command_deadzone) {
        throw std::runtime_error("pedal_command_exit_deadzone must be in [0, pedal_command_deadzone]");
    }
    if (cfg.trim_capture_enabled < 0.0 || cfg.trim_capture_enabled > 1.0) {
        throw std::runtime_error("trim_capture_enabled must be in [0, 1]");
    }
    if (cfg.trim_capture_min_pedal < 0.0 || cfg.trim_capture_min_pedal > 1.0) {
        throw std::runtime_error("trim_capture_min_pedal must be in [0, 1]");
    }
    if (cfg.trim_capture_yaw_rate < 0.0) throw std::runtime_error("trim_capture_yaw_rate must be non-negative");
    if (cfg.trim_capture_pedal_rate < 0.0) throw std::runtime_error("trim_capture_pedal_rate must be non-negative");
    if (cfg.trim_guard_enabled < 0.0 || cfg.trim_guard_enabled > 1.0) {
        throw std::runtime_error("trim_guard_enabled must be in [0, 1]");
    }
    if (cfg.trim_guard_input_id <= 0) throw std::runtime_error("trim_guard_input_id must be positive");
    if (cfg.trim_guard_input_button < 0 || cfg.trim_guard_input_button > 128) {
        throw std::runtime_error("trim_guard_input_button must be in [0, 128]");
    }
    if (cfg.trim_guard_output_button < 0 || cfg.trim_guard_output_button > 128) {
        throw std::runtime_error("trim_guard_output_button must be in [0, 128]");
    }
    if (cfg.trim_guard_pre_time < 0.0) throw std::runtime_error("trim_guard_pre_time must be non-negative");
    if (cfg.trim_guard_press_time < 0.0) throw std::runtime_error("trim_guard_press_time must be non-negative");
    if (cfg.trim_guard_post_time < 0.0) throw std::runtime_error("trim_guard_post_time must be non-negative");
    if (cfg.power_feedforward_source != "off" &&
        cfg.power_feedforward_source != "collective" &&
        cfg.power_feedforward_source != "fuel_flow" &&
        cfg.power_feedforward_source != "fuel_rpm") {
        throw std::runtime_error("power_feedforward_source must be off, collective, fuel_flow, or fuel_rpm");
    }
    if (cfg.fuel_flow_max <= cfg.fuel_flow_min) {
        throw std::runtime_error("fuel_flow_max must be greater than fuel_flow_min");
    }
    if (cfg.rpm_drop_full_scale <= 0.0) {
        throw std::runtime_error("rpm_drop_full_scale must be positive");
    }
    if (cfg.rpm_power_gain < 0.0 || cfg.rpm_power_gain > 2.0) {
        throw std::runtime_error("rpm_power_gain must be in [0, 2]");
    }
    if (cfg.collective_invert < 0.0 || cfg.collective_invert > 1.0) {
        throw std::runtime_error("collective_invert must be in [0, 1]");
    }
    if (cfg.collective_feedforward_mode != "linear" && cfg.collective_feedforward_mode != "power") {
        throw std::runtime_error("collective_feedforward_mode must be linear or power");
    }
    if (cfg.collective_gain < 0.0) throw std::runtime_error("collective_gain must be non-negative");
    if (cfg.collective_zero_thrust < 0.0 || cfg.collective_zero_thrust > 1.0) {
        throw std::runtime_error("collective_zero_thrust must be in [0, 1]");
    }
    if (cfg.collective_power_exponent <= 0.0 || cfg.collective_power_exponent > 4.0) {
        throw std::runtime_error("collective_power_exponent must be in (0, 4]");
    }
    if (cfg.collective_rate_gain < 0.0) throw std::runtime_error("collective_rate_gain must be non-negative");
    if (cfg.collective_rate_limit < 0.0 || cfg.collective_rate_limit > 1.0) {
        throw std::runtime_error("collective_rate_limit must be in [0, 1]");
    }
    if (cfg.collective_transient_rate_threshold < 0.0) {
        throw std::runtime_error("collective_transient_rate_threshold must be non-negative");
    }
    if (cfg.collective_rate_hold_time < 0.0) {
        throw std::runtime_error("collective_rate_hold_time must be non-negative");
    }
    if (cfg.collective_rate_reverse_threshold < 0.0 || cfg.collective_rate_reverse_threshold > 1.0) {
        throw std::runtime_error("collective_rate_reverse_threshold must be in [0, 1]");
    }
    if (cfg.collective_rate_reverse_slew_time < 0.0) {
        throw std::runtime_error("collective_rate_reverse_slew_time must be non-negative");
    }
    if (cfg.collective_transient_fade_time < 0.0) {
        throw std::runtime_error("collective_transient_fade_time must be non-negative");
    }
    if (cfg.collective_output_invert < 0.0 || cfg.collective_output_invert > 1.0) {
        throw std::runtime_error("collective_output_invert must be in [0, 1]");
    }
    if (cfg.auto_tune_collective_drive < 0.0 || cfg.auto_tune_collective_drive > 1.0) {
        throw std::runtime_error("auto_tune_collective_drive must be in [0, 1]");
    }
    if (cfg.auto_tune_collective_amplitude < 0.0 || cfg.auto_tune_collective_amplitude > 0.25) {
        throw std::runtime_error("auto_tune_collective_amplitude must be in [0, 0.25]");
    }
    if (cfg.auto_tune_collective_period <= 0.0) {
        throw std::runtime_error("auto_tune_collective_period must be positive");
    }
    if (cfg.auto_tune_collective_settle_time < 0.0) {
        throw std::runtime_error("auto_tune_collective_settle_time must be non-negative");
    }
    if (cfg.auto_tune_collective_rate_limit < 0.0) {
        throw std::runtime_error("auto_tune_collective_rate_limit must be non-negative");
    }
    if (cfg.calibration_max_assist < 0.0 || cfg.calibration_max_assist > 0.25) {
        throw std::runtime_error("calibration_max_assist must be in [0, 0.25]");
    }
    if (cfg.filter_time < 0.0) throw std::runtime_error("filter_time must be non-negative");
    if (cfg.stale_timeout <= 0.0) throw std::runtime_error("stale_timeout must be positive");
    if (cfg.f14_roll_input_id < 0) throw std::runtime_error("f14_roll_input_id must be non-negative");
    if (cfg.f14_rudder_input_id < 0) throw std::runtime_error("f14_rudder_input_id must be non-negative");
    if (cfg.f14_aoa_units_per_radian <= 0.0) throw std::runtime_error("f14_aoa_units_per_radian must be positive");
    if (cfg.f14_aoa_full_units <= cfg.f14_aoa_onset_units) {
        throw std::runtime_error("f14_aoa_full_units must be greater than f14_aoa_onset_units");
    }
    if (cfg.f14_deep_aoa_full_units <= cfg.f14_deep_aoa_onset_units) {
        throw std::runtime_error("f14_deep_aoa_full_units must be greater than f14_deep_aoa_onset_units");
    }
    if (cfg.f14_roll_deadzone < 0.0 || cfg.f14_roll_deadzone > 1.0) {
        throw std::runtime_error("f14_roll_deadzone must be in [0, 1]");
    }
    if (cfg.f14_roll_washout < 0.0 || cfg.f14_roll_washout > 1.0) {
        throw std::runtime_error("f14_roll_washout must be in [0, 1]");
    }
    if (cfg.f14_deep_roll_washout < 0.0 || cfg.f14_deep_roll_washout > 1.0) {
        throw std::runtime_error("f14_deep_roll_washout must be in [0, 1]");
    }
    if (cfg.f14_roll_min_scale < 0.0 || cfg.f14_roll_min_scale > 1.0) {
        throw std::runtime_error("f14_roll_min_scale must be in [0, 1]");
    }
    if (cfg.f14_roll_to_rudder_gain < 0.0) throw std::runtime_error("f14_roll_to_rudder_gain must be non-negative");
    if (cfg.f14_deep_roll_to_rudder_gain < 0.0) {
        throw std::runtime_error("f14_deep_roll_to_rudder_gain must be non-negative");
    }
    if (cfg.f14_low_aoa_roll_coordination_gain < 0.0) {
        throw std::runtime_error("f14_low_aoa_roll_coordination_gain must be non-negative");
    }
    if (cfg.f14_yaw_rate_gain < 0.0) throw std::runtime_error("f14_yaw_rate_gain must be non-negative");
    if (cfg.f14_slip_gain < 0.0) throw std::runtime_error("f14_slip_gain must be non-negative");
    if (cfg.f14_beta_limit < 0.0 || cfg.f14_beta_limit > 1.0) {
        throw std::runtime_error("f14_beta_limit must be in [0, 1]");
    }
    if (cfg.f14_beta_limiter_gain < 0.0) throw std::runtime_error("f14_beta_limiter_gain must be non-negative");
    if (cfg.f14_rudder_max_assist < 0.0 || cfg.f14_rudder_max_assist > 1.0) {
        throw std::runtime_error("f14_rudder_max_assist must be in [0, 1]");
    }
    if (cfg.f14_rudder_rate_limit < 0.0) throw std::runtime_error("f14_rudder_rate_limit must be non-negative");
    if (cfg.f14_ground_ias_threshold < 0.0) throw std::runtime_error("f14_ground_ias_threshold must be non-negative");
    if (cfg.f14_ground_agl_threshold < 0.0) throw std::runtime_error("f14_ground_agl_threshold must be non-negative");
    if (cfg.f14_landing_gear_threshold < 0.0 || cfg.f14_landing_gear_threshold > 1.0) {
        throw std::runtime_error("f14_landing_gear_threshold must be in [0, 1]");
    }
    if (cfg.f14_landing_flaps_threshold < 0.0 || cfg.f14_landing_flaps_threshold > 1.0) {
        throw std::runtime_error("f14_landing_flaps_threshold must be in [0, 1]");
    }
    if (cfg.f14_reversal_guard_time < 0.0) throw std::runtime_error("f14_reversal_guard_time must be non-negative");
    if (cfg.f14_reversal_roll_threshold < 0.0 || cfg.f14_reversal_roll_threshold > 1.0) {
        throw std::runtime_error("f14_reversal_roll_threshold must be in [0, 1]");
    }
    if (cfg.f14_reversal_roll_rate_threshold < 0.0) {
        throw std::runtime_error("f14_reversal_roll_rate_threshold must be non-negative");
    }
    if (cfg.f14_reversal_guard_scale < 0.0 || cfg.f14_reversal_guard_scale > 1.0) {
        throw std::runtime_error("f14_reversal_guard_scale must be in [0, 1]");
    }
}

}  // namespace

AppConfig load_config(const std::filesystem::path& path) {
    AppConfig cfg;
    std::ifstream in(path);
    if (!in) {
        validate(cfg);
        return cfg;
    }

    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || starts_with(line, "#") || starts_with(line, ";") || starts_with(line, "[")) {
            continue;
        }

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));
        if ((value.size() >= 2) &&
            ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
            value = value.substr(1, value.size() - 2);
        }
        apply_key(cfg, key, value);
    }

    validate(cfg);
    return cfg;
}

void write_default_config(const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        throw std::runtime_error("Could not write default config: " + path.string());
    }

    const AppConfig cfg;
    out << "# AH-64D external yaw FBW / auto rudder\n"
        << "dcs_bios_path=" << cfg.dcs_bios_path.string() << "\n"
        << "multicast_address=" << cfg.multicast_address << "\n"
        << "multicast_interface=" << cfg.multicast_interface << "\n"
        << "udp_port=" << cfg.udp_port << "\n\n"
        << "telemetry_source=" << cfg.telemetry_source << "\n"
        << "fast_export_bind_address=" << cfg.fast_export_bind_address << "\n"
        << "fast_export_port=" << cfg.fast_export_port << "\n\n"
        << "input_vjoy_id=" << cfg.input_vjoy_id << "\n"
        << "output_vjoy_id=" << cfg.output_vjoy_id << "\n"
        << "input_device_name_contains=" << cfg.input_device_name_contains << "\n"
        << "axis_name=" << cfg.axis_name << "\n\n"
        << "rudder_input_center=" << cfg.rudder_input_center << "\n"
        << "rudder_input_deadzone=" << cfg.rudder_input_deadzone << "\n"
        << "rudder_input_scale=" << cfg.rudder_input_scale << "\n\n"
        << "control_mode=" << cfg.control_mode << "\n"
        << "assist_sign=" << cfg.assist_sign << "\n"
        << "yaw_response_sign=" << cfg.yaw_response_sign << "\n"
        << "yaw_rate_sign=" << cfg.yaw_rate_sign << "\n"
        << "yaw_rate_source=" << cfg.yaw_rate_source << "\n"
        << "kp=" << cfg.kp << "\n"
        << "ki=" << cfg.ki << "\n"
        << "integral_limit=" << cfg.integral_limit << "\n"
        << "max_assist=" << cfg.max_assist << "\n"
        << "heading_hold_max_assist=" << cfg.heading_hold_max_assist << "\n"
        << "release_brake_time=" << cfg.release_brake_time << "\n"
        << "release_brake_kp=" << cfg.release_brake_kp << "\n"
        << "release_brake_max_assist=" << cfg.release_brake_max_assist << "\n"
        << "yaw_rate_deadband=" << cfg.yaw_rate_deadband << "\n"
        << "yaw_rate_error_exponent=" << cfg.yaw_rate_error_exponent << "\n"
        << "yaw_rate_integral_gain=" << cfg.yaw_rate_integral_gain << "\n"
        << "yaw_rate_integral_limit=" << cfg.yaw_rate_integral_limit << "\n"
        << "yaw_rate_integral_leak=" << cfg.yaw_rate_integral_leak << "\n"
        << "yaw_rate_integral_deadband=" << cfg.yaw_rate_integral_deadband << "\n"
        << "yaw_accel_gain=" << cfg.yaw_accel_gain << "\n"
        << "yaw_accel_filter_time=" << cfg.yaw_accel_filter_time << "\n"
        << "yaw_accel_limit=" << cfg.yaw_accel_limit << "\n"
        << "heading_error_deadband=" << cfg.heading_error_deadband << "\n"
        << "heading_error_relock_threshold=" << cfg.heading_error_relock_threshold << "\n"
        << "heading_hold_leak_time=" << cfg.heading_hold_leak_time << "\n"
        << "heading_kp=" << cfg.heading_kp << "\n"
        << "heading_rate_limit=" << cfg.heading_rate_limit << "\n"
        << "turn_rate_max=" << cfg.turn_rate_max << "\n"
        << "pedal_command_sign=" << cfg.pedal_command_sign << "\n"
        << "pedal_command_deadzone=" << cfg.pedal_command_deadzone << "\n"
        << "pedal_command_exit_deadzone=" << cfg.pedal_command_exit_deadzone << "\n"
        << "pedal_override_threshold=" << cfg.pedal_override_threshold << "\n"
        << "pedal_rate_override_threshold=" << cfg.pedal_rate_override_threshold << "\n"
        << "trim_capture_enabled=" << cfg.trim_capture_enabled << "\n"
        << "trim_capture_min_pedal=" << cfg.trim_capture_min_pedal << "\n"
        << "trim_capture_yaw_rate=" << cfg.trim_capture_yaw_rate << "\n"
        << "trim_capture_pedal_rate=" << cfg.trim_capture_pedal_rate << "\n"
        << "trim_guard_enabled=" << cfg.trim_guard_enabled << "\n"
        << "trim_guard_input_id=" << cfg.trim_guard_input_id << "\n"
        << "trim_guard_input_device_name_contains=" << cfg.trim_guard_input_device_name_contains << "\n"
        << "trim_guard_input_button=" << cfg.trim_guard_input_button << "\n"
        << "trim_guard_output_button=" << cfg.trim_guard_output_button << "\n"
        << "trim_guard_pre_time=" << cfg.trim_guard_pre_time << "\n"
        << "trim_guard_press_time=" << cfg.trim_guard_press_time << "\n"
        << "trim_guard_post_time=" << cfg.trim_guard_post_time << "\n"
        << "power_feedforward_source=" << cfg.power_feedforward_source << "\n"
        << "fuel_flow_min=" << cfg.fuel_flow_min << "\n"
        << "fuel_flow_max=" << cfg.fuel_flow_max << "\n"
        << "rpm_nominal=" << cfg.rpm_nominal << "\n"
        << "rpm_drop_full_scale=" << cfg.rpm_drop_full_scale << "\n"
        << "rpm_power_gain=" << cfg.rpm_power_gain << "\n"
        << "collective_source=" << cfg.collective_source << "\n"
        << "collective_input_id=" << cfg.collective_input_id << "\n"
        << "collective_device_name_contains=" << cfg.collective_device_name_contains << "\n"
        << "collective_axis_name=" << cfg.collective_axis_name << "\n"
        << "collective_invert=" << cfg.collective_invert << "\n"
        << "collective_sign=" << cfg.collective_sign << "\n"
        << "collective_feedforward_mode=" << cfg.collective_feedforward_mode << "\n"
        << "collective_gain=" << cfg.collective_gain << "\n"
        << "collective_zero_thrust=" << cfg.collective_zero_thrust << "\n"
        << "collective_power_exponent=" << cfg.collective_power_exponent << "\n"
        << "collective_rate_gain=" << cfg.collective_rate_gain << "\n"
        << "collective_rate_limit=" << cfg.collective_rate_limit << "\n"
        << "collective_transient_rate_threshold=" << cfg.collective_transient_rate_threshold << "\n"
        << "collective_rate_hold_time=" << cfg.collective_rate_hold_time << "\n"
        << "collective_rate_reverse_threshold=" << cfg.collective_rate_reverse_threshold << "\n"
        << "collective_rate_reverse_slew_time=" << cfg.collective_rate_reverse_slew_time << "\n"
        << "collective_transient_fade_time=" << cfg.collective_transient_fade_time << "\n"
        << "collective_output_axis_name=" << cfg.collective_output_axis_name << "\n"
        << "collective_output_invert=" << cfg.collective_output_invert << "\n"
        << "auto_tune_collective_drive=" << cfg.auto_tune_collective_drive << "\n"
        << "auto_tune_collective_amplitude=" << cfg.auto_tune_collective_amplitude << "\n"
        << "auto_tune_collective_period=" << cfg.auto_tune_collective_period << "\n"
        << "auto_tune_collective_settle_time=" << cfg.auto_tune_collective_settle_time << "\n"
        << "auto_tune_collective_rate_limit=" << cfg.auto_tune_collective_rate_limit << "\n"
        << "fade_in_time=" << cfg.fade_in_time << "\n"
        << "fade_out_time=" << cfg.fade_out_time << "\n"
        << "filter_time=" << cfg.filter_time << "\n"
        << "stale_timeout=" << cfg.stale_timeout << "\n"
        << "loop_hz=" << cfg.loop_hz << "\n\n"
        << "# F-14 high-AoA roll-to-rudder assist. Enable with control_mode=f14_roll_assist.\n"
        << "f14_roll_axis_name=" << cfg.f14_roll_axis_name << "\n"
        << "f14_rudder_axis_name=" << cfg.f14_rudder_axis_name << "\n"
        << "# Optional F-14 input overrides. Leave id=0/filter empty to use input_vjoy_id/input_device_name_contains.\n"
        << "f14_roll_input_id=" << cfg.f14_roll_input_id << "\n"
        << "f14_roll_device_name_contains=" << cfg.f14_roll_device_name_contains << "\n"
        << "f14_rudder_input_id=" << cfg.f14_rudder_input_id << "\n"
        << "f14_rudder_device_name_contains=" << cfg.f14_rudder_device_name_contains << "\n"
        << "f14_output_roll_axis_name=" << cfg.f14_output_roll_axis_name << "\n"
        << "f14_output_rudder_axis_name=" << cfg.f14_output_rudder_axis_name << "\n"
        << "# Multiplier is applied to the raw LoGetAngleOfAttack() export. Current DCS/F-14\n"
        << "# logs behave like degrees, so the default is 1.5 units per exported degree.\n"
        << "# If your export is radians, use 85.9436692696 instead.\n"
        << "f14_aoa_units_offset=" << cfg.f14_aoa_units_offset << "\n"
        << "f14_aoa_units_per_radian=" << cfg.f14_aoa_units_per_radian << "\n"
        << "f14_aoa_onset_units=" << cfg.f14_aoa_onset_units << "\n"
        << "f14_aoa_full_units=" << cfg.f14_aoa_full_units << "\n"
        << "f14_deep_aoa_onset_units=" << cfg.f14_deep_aoa_onset_units << "\n"
        << "f14_deep_aoa_full_units=" << cfg.f14_deep_aoa_full_units << "\n"
        << "f14_roll_deadzone=" << cfg.f14_roll_deadzone << "\n"
        << "f14_roll_washout=" << cfg.f14_roll_washout << "\n"
        << "f14_deep_roll_washout=" << cfg.f14_deep_roll_washout << "\n"
        << "f14_roll_min_scale=" << cfg.f14_roll_min_scale << "\n"
        << "f14_roll_to_rudder_gain=" << cfg.f14_roll_to_rudder_gain << "\n"
        << "f14_deep_roll_to_rudder_gain=" << cfg.f14_deep_roll_to_rudder_gain << "\n"
        << "f14_low_aoa_roll_coordination_gain=" << cfg.f14_low_aoa_roll_coordination_gain << "\n"
        << "f14_roll_to_rudder_sign=" << cfg.f14_roll_to_rudder_sign << "\n"
        << "f14_yaw_rate_gain=" << cfg.f14_yaw_rate_gain << "\n"
        << "f14_yaw_rate_sign=" << cfg.f14_yaw_rate_sign << "\n"
        << "f14_slip_gain=" << cfg.f14_slip_gain << "\n"
        << "f14_slip_sign=" << cfg.f14_slip_sign << "\n"
        << "f14_beta_limit=" << cfg.f14_beta_limit << "\n"
        << "f14_beta_limiter_gain=" << cfg.f14_beta_limiter_gain << "\n"
        << "f14_rudder_max_assist=" << cfg.f14_rudder_max_assist << "\n"
        << "f14_rudder_rate_limit=" << cfg.f14_rudder_rate_limit << "\n"
        << "f14_ground_ias_threshold=" << cfg.f14_ground_ias_threshold << "\n"
        << "f14_ground_agl_threshold=" << cfg.f14_ground_agl_threshold << "\n"
        << "f14_landing_gear_threshold=" << cfg.f14_landing_gear_threshold << "\n"
        << "f14_landing_flaps_threshold=" << cfg.f14_landing_flaps_threshold << "\n"
        << "f14_roll_rate_sign=" << cfg.f14_roll_rate_sign << "\n"
        << "f14_reversal_guard_time=" << cfg.f14_reversal_guard_time << "\n"
        << "f14_reversal_roll_threshold=" << cfg.f14_reversal_roll_threshold << "\n"
        << "f14_reversal_roll_rate_threshold=" << cfg.f14_reversal_roll_rate_threshold << "\n"
        << "f14_reversal_guard_scale=" << cfg.f14_reversal_guard_scale << "\n\n"
        << "hotkey=" << cfg.hotkey << "\n"
        << "log_path=" << cfg.log_path.string() << "\n"
        << "calibration_max_assist=" << cfg.calibration_max_assist << "\n";
}

}  // namespace autorudder
