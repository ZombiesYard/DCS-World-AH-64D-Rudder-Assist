#pragma once

#include <filesystem>
#include <string>

namespace autorudder {

struct AppConfig {
    std::filesystem::path dcs_bios_path =
        R"(C:\Users\15423\Saved Games\DCS\Scripts\DCS-BIOS)";
    std::string multicast_address = "239.255.50.10";
    std::string multicast_interface = "127.0.0.1";
    int udp_port = 5010;
    std::string telemetry_source = "fast_export";
    std::string fast_export_bind_address = "127.0.0.1";
    int fast_export_port = 34380;

    int input_vjoy_id = 1;
    int output_vjoy_id = 2;
    std::string input_device_name_contains = "vJoy";
    std::string axis_name = "X";
    double rudder_input_center = 0.0;
    double rudder_input_deadzone = 0.0;
    double rudder_input_scale = 1.0;

    double ah64_roll_enabled = 0.0;
    int ah64_roll_input_id = 1;
    std::string ah64_roll_device_name_contains = "Joystick - HOTAS Warthog";
    std::string ah64_roll_axis_name = "X";
    std::string ah64_roll_output_axis_name = "Y";
    double ah64_roll_input_center = 0.0;
    double ah64_roll_input_deadzone = 0.02;
    double ah64_roll_input_scale = 1.0;
    double ah64_roll_override_threshold = 0.30;
    double ah64_roll_counter_sign = -1.0;
    double ah64_roll_counter_gain = 0.18;
    double ah64_roll_counter_max = 0.08;
    double ah64_roll_counter_deadband = 0.04;
    double ah64_roll_counter_fade_time = 0.20;

    std::string control_mode = "heading_hold";
    double assist_sign = -1.0;
    double yaw_response_sign = 1.0;
    double yaw_rate_sign = -1.0;
    std::string yaw_rate_source = "heading";
    double kp = 1.45;
    double ki = 0.0;
    double integral_limit = 0.0;
    double max_assist = 0.85;
    double heading_hold_max_assist = 0.35;
    double release_brake_time = 1.80;
    double release_brake_kp = 3.20;
    double release_brake_max_assist = 0.85;
    double yaw_rate_deadband = 0.010;
    double yaw_rate_error_exponent = 1.0;
    double yaw_rate_integral_gain = 0.0;
    double yaw_rate_integral_limit = 0.0;
    double yaw_rate_integral_leak = 0.0;
    double yaw_rate_integral_deadband = 0.0;
    double yaw_accel_gain = 0.0;
    double yaw_accel_filter_time = 0.04;
    double yaw_accel_limit = 0.25;
    double heading_error_deadband = 0.015;
    double heading_error_relock_threshold = 0.80;
    double heading_hold_leak_time = 0.0;
    double heading_kp = 0.75;
    double heading_rate_limit = 0.30;
    double turn_rate_max = 0.45;
    double pedal_command_sign = 1.0;
    double pedal_command_deadzone = 0.06;
    double pedal_command_exit_deadzone = 0.03;
    double pedal_override_threshold = 0.12;
    double pedal_rate_override_threshold = 1.0;
    double trim_capture_enabled = 0.0;
    double trim_capture_min_pedal = 0.20;
    double trim_capture_yaw_rate = 0.025;
    double trim_capture_pedal_rate = 0.50;
    double trim_guard_enabled = 0.0;
    int trim_guard_input_id = 1;
    std::string trim_guard_input_device_name_contains;
    int trim_guard_input_button = 0;
    int trim_guard_output_button = 0;
    double trim_guard_pre_time = 0.08;
    double trim_guard_press_time = 0.08;
    double trim_guard_post_time = 0.35;
    std::string power_feedforward_source = "fuel_rpm";
    double fuel_flow_min = 0.055;
    double fuel_flow_max = 0.155;
    double rpm_nominal = 100.0;
    double rpm_drop_full_scale = 8.0;
    double rpm_power_gain = 0.40;
    double power_proxy_rise_rate_limit = 0.0;
    double power_proxy_fall_rate_limit = 0.0;
    double power_collective_lead_gain = 0.0;
    double power_collective_lead_invert = 0.0;
    double power_collective_lead_deadband = 0.02;
    std::string collective_source = "auto";
    int collective_input_id = 1;
    std::string collective_device_name_contains;
    std::string collective_axis_name = "Z";
    double collective_invert = 0.0;
    double collective_sign = -1.0;
    std::string collective_feedforward_mode = "linear";
    double collective_gain = 0.70;
    double collective_zero_thrust = 0.0;
    double collective_power_exponent = 1.5;
    double collective_rate_gain = 0.45;
    double collective_rate_limit = 0.40;
    double collective_transient_rate_threshold = 0.20;
    double collective_rate_hold_time = 0.18;
    double collective_rate_reverse_threshold = 0.35;
    double collective_rate_reverse_slew_time = 0.12;
    double collective_transient_fade_time = 0.015;
    std::string collective_output_axis_name = "Z";
    double collective_output_invert = 0.0;
    double auto_tune_collective_drive = 0.0;
    double auto_tune_collective_amplitude = 0.05;
    double auto_tune_collective_period = 4.0;
    double auto_tune_collective_settle_time = 6.0;
    double auto_tune_collective_rate_limit = 0.40;
    double fade_in_time = 0.06;
    double fade_out_time = 0.06;
    double filter_time = 0.05;
    double stale_timeout = 1.00;
    int loop_hz = 100;

    std::string f14_roll_axis_name = "X";
    std::string f14_rudder_axis_name = "RZ";
    int f14_roll_input_id = 0;
    std::string f14_roll_device_name_contains;
    int f14_rudder_input_id = 0;
    std::string f14_rudder_device_name_contains;
    std::string f14_output_roll_axis_name = "X";
    std::string f14_output_rudder_axis_name = "RZ";
    double f14_aoa_units_offset = 0.0;
    double f14_aoa_units_per_radian = 1.5;
    double f14_aoa_onset_units = 12.5;
    double f14_aoa_full_units = 16.5;
    double f14_deep_aoa_onset_units = 19.0;
    double f14_deep_aoa_full_units = 23.0;
    double f14_roll_deadzone = 0.03;
    double f14_roll_washout = 0.55;
    double f14_deep_roll_washout = 0.10;
    double f14_roll_min_scale = 0.35;
    double f14_roll_to_rudder_gain = 0.65;
    double f14_deep_roll_to_rudder_gain = 0.20;
    double f14_low_aoa_roll_coordination_gain = 0.04;
    double f14_roll_to_rudder_sign = 1.0;
    double f14_yaw_rate_gain = 0.08;
    double f14_yaw_rate_sign = 1.0;
    double f14_slip_gain = 0.0;
    double f14_slip_sign = 1.0;
    double f14_beta_limit = 0.35;
    double f14_beta_limiter_gain = 0.30;
    double f14_rudder_max_assist = 0.65;
    double f14_rudder_rate_limit = 2.0;
    double f14_ground_ias_threshold = 35.0;
    double f14_ground_agl_threshold = 3.0;
    double f14_landing_gear_threshold = 0.50;
    double f14_landing_flaps_threshold = 0.50;
    double f14_roll_rate_sign = 1.0;
    double f14_reversal_guard_time = 0.15;
    double f14_reversal_roll_threshold = 0.30;
    double f14_reversal_roll_rate_threshold = 0.10;
    double f14_reversal_guard_scale = 0.35;

    std::string hotkey = "PAUSE";
    std::filesystem::path log_path = "auto_rudder.log";
    double calibration_max_assist = 0.08;
};

AppConfig load_config(const std::filesystem::path& path);
void write_default_config(const std::filesystem::path& path);

}  // namespace autorudder
