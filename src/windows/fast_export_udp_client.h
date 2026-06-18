#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <winsock2.h>

namespace autorudder::windows {

struct FastExportTelemetry {
    std::string aircraft_name;
    double yaw_rate_z = 0.0;
    std::optional<double> yaw_rate_y;
    std::optional<double> slip_ball;
    std::optional<double> collective;
    std::optional<double> heading;
    std::optional<double> angle_of_attack;
    std::optional<double> roll_rate_x;
    std::optional<double> indicated_airspeed;
    std::optional<double> radar_altitude;
    std::optional<double> gear_position;
    std::optional<double> flaps_position;
    std::optional<double> engine_rpm_avg;
    std::optional<double> engine_fuel_flow_avg;
    std::optional<double> engine_torque_avg;
    std::optional<double> engine_torque_left;
    std::optional<double> engine_torque_right;
    std::optional<double> tail_rudder_left;
    std::optional<double> tail_rudder_right;
    std::optional<double> yaw_acceleration_z;
    std::optional<double> pitch;
    std::optional<double> bank;
    std::optional<double> attitude_yaw;
    std::optional<double> velocity_x;
    std::optional<double> velocity_y;
    std::optional<double> velocity_z;
    std::optional<double> speed_3d;
    std::optional<double> ground_speed;
    std::optional<double> vertical_velocity;
    std::optional<double> true_airspeed;
    std::optional<double> mach;
    std::optional<double> altitude_msl;
    std::optional<double> latitude;
    std::optional<double> longitude;
    std::optional<double> accel_x;
    std::optional<double> accel_y;
    std::optional<double> accel_z;
    std::optional<double> wind_x;
    std::optional<double> wind_y;
    std::optional<double> wind_z;
};

struct FastExportProbeValue {
    std::string aircraft_name;
    std::string source;
    std::string key;
    double value = 0.0;
};

class FastExportUdpClient {
public:
    FastExportUdpClient(const std::string& bind_address, int port);
    ~FastExportUdpClient();

    FastExportUdpClient(const FastExportUdpClient&) = delete;
    FastExportUdpClient& operator=(const FastExportUdpClient&) = delete;

    int pump();
    bool has_recent_frame(double stale_timeout_seconds) const;
    std::optional<FastExportTelemetry> latest() const;
    std::vector<FastExportProbeValue> drain_probe_values();

private:
    SOCKET socket_ = INVALID_SOCKET;
    std::optional<FastExportTelemetry> latest_;
    std::vector<FastExportProbeValue> probe_values_;
    std::chrono::steady_clock::time_point last_frame_time_{};
};

}  // namespace autorudder::windows
