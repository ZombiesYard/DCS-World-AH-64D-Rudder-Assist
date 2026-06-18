#include "windows/fast_export_udp_client.h"

#include <array>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ws2tcpip.h>

namespace autorudder::windows {
namespace {

std::string socket_error(const char* operation) {
    return std::string(operation) + " failed, WSA error " + std::to_string(WSAGetLastError());
}

void ensure_winsock_started() {
    static bool started = [] {
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
        return true;
    }();
    (void)started;
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        if (!field.empty() && (field.back() == '\n' || field.back() == '\r')) {
            field.pop_back();
        }
        fields.push_back(field);
    }
    return fields;
}

std::optional<FastExportTelemetry> parse_line(const std::string& line) {
    const auto fields = split_csv(line);
    if (fields.size() < 5 || fields[0] != "AR1") {
        return std::nullopt;
    }

    FastExportTelemetry telemetry;
    telemetry.aircraft_name = fields[2];
    try {
        telemetry.yaw_rate_z = std::stod(fields[3]);
        telemetry.slip_ball = std::stod(fields[4]);
        if (fields.size() >= 6 && !fields[5].empty()) {
            telemetry.collective = std::stod(fields[5]);
        }
        if (fields.size() >= 7 && !fields[6].empty()) {
            telemetry.heading = std::stod(fields[6]);
        }
        if (fields.size() >= 8 && !fields[7].empty()) {
            telemetry.angle_of_attack = std::stod(fields[7]);
        }
        if (fields.size() >= 9 && !fields[8].empty()) {
            telemetry.roll_rate_x = std::stod(fields[8]);
        }
        if (fields.size() >= 10 && !fields[9].empty()) {
            telemetry.indicated_airspeed = std::stod(fields[9]);
        }
        if (fields.size() >= 11 && !fields[10].empty()) {
            telemetry.radar_altitude = std::stod(fields[10]);
        }
        if (fields.size() >= 12 && !fields[11].empty()) {
            telemetry.gear_position = std::stod(fields[11]);
        }
        if (fields.size() >= 13 && !fields[12].empty()) {
            telemetry.flaps_position = std::stod(fields[12]);
        }
        if (fields.size() >= 14 && !fields[13].empty()) {
            telemetry.engine_rpm_avg = std::stod(fields[13]);
        }
        if (fields.size() >= 15 && !fields[14].empty()) {
            telemetry.engine_fuel_flow_avg = std::stod(fields[14]);
        }
        if (fields.size() >= 16 && !fields[15].empty()) {
            telemetry.tail_rudder_left = std::stod(fields[15]);
        }
        if (fields.size() >= 17 && !fields[16].empty()) {
            telemetry.tail_rudder_right = std::stod(fields[16]);
        }
        if (fields.size() >= 18 && !fields[17].empty()) {
            telemetry.yaw_acceleration_z = std::stod(fields[17]);
        }
        if (fields.size() >= 19 && !fields[18].empty()) {
            telemetry.engine_torque_avg = std::stod(fields[18]);
        }
        if (fields.size() >= 20 && !fields[19].empty()) {
            telemetry.engine_torque_left = std::stod(fields[19]);
        }
        if (fields.size() >= 21 && !fields[20].empty()) {
            telemetry.engine_torque_right = std::stod(fields[20]);
        }
        if (fields.size() >= 22 && !fields[21].empty()) {
            telemetry.yaw_rate_y = std::stod(fields[21]);
        }
        if (fields.size() >= 23 && !fields[22].empty()) {
            telemetry.pitch = std::stod(fields[22]);
        }
        if (fields.size() >= 24 && !fields[23].empty()) {
            telemetry.bank = std::stod(fields[23]);
        }
        if (fields.size() >= 25 && !fields[24].empty()) {
            telemetry.attitude_yaw = std::stod(fields[24]);
        }
        if (fields.size() >= 26 && !fields[25].empty()) {
            telemetry.velocity_x = std::stod(fields[25]);
        }
        if (fields.size() >= 27 && !fields[26].empty()) {
            telemetry.velocity_y = std::stod(fields[26]);
        }
        if (fields.size() >= 28 && !fields[27].empty()) {
            telemetry.velocity_z = std::stod(fields[27]);
        }
        if (fields.size() >= 29 && !fields[28].empty()) {
            telemetry.speed_3d = std::stod(fields[28]);
        }
        if (fields.size() >= 30 && !fields[29].empty()) {
            telemetry.ground_speed = std::stod(fields[29]);
        }
        if (fields.size() >= 31 && !fields[30].empty()) {
            telemetry.vertical_velocity = std::stod(fields[30]);
        }
        if (fields.size() >= 32 && !fields[31].empty()) {
            telemetry.true_airspeed = std::stod(fields[31]);
        }
        if (fields.size() >= 33 && !fields[32].empty()) {
            telemetry.mach = std::stod(fields[32]);
        }
        if (fields.size() >= 34 && !fields[33].empty()) {
            telemetry.altitude_msl = std::stod(fields[33]);
        }
        if (fields.size() >= 35 && !fields[34].empty()) {
            telemetry.latitude = std::stod(fields[34]);
        }
        if (fields.size() >= 36 && !fields[35].empty()) {
            telemetry.longitude = std::stod(fields[35]);
        }
        if (fields.size() >= 37 && !fields[36].empty()) {
            telemetry.accel_x = std::stod(fields[36]);
        }
        if (fields.size() >= 38 && !fields[37].empty()) {
            telemetry.accel_y = std::stod(fields[37]);
        }
        if (fields.size() >= 39 && !fields[38].empty()) {
            telemetry.accel_z = std::stod(fields[38]);
        }
        if (fields.size() >= 40 && !fields[39].empty()) {
            telemetry.wind_x = std::stod(fields[39]);
        }
        if (fields.size() >= 41 && !fields[40].empty()) {
            telemetry.wind_y = std::stod(fields[40]);
        }
        if (fields.size() >= 42 && !fields[41].empty()) {
            telemetry.wind_z = std::stod(fields[41]);
        }
    } catch (const std::exception&) {
        return std::nullopt;
    }
    return telemetry;
}

std::optional<FastExportProbeValue> parse_probe_line(const std::string& line) {
    const auto fields = split_csv(line);
    if (fields.size() < 6 || fields[0] != "ARP") {
        return std::nullopt;
    }

    FastExportProbeValue value;
    value.aircraft_name = fields[2];
    value.source = fields[3];
    value.key = fields[4];
    try {
        value.value = std::stod(fields[5]);
    } catch (const std::exception&) {
        return std::nullopt;
    }
    return value;
}

}  // namespace

FastExportUdpClient::FastExportUdpClient(const std::string& bind_address, int port) {
    ensure_winsock_started();

    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
        throw std::runtime_error(socket_error("socket"));
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = inet_addr(bind_address.c_str());
    local.sin_port = htons(static_cast<u_short>(port));
    if (bind(socket_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        throw std::runtime_error(socket_error("bind fast_export"));
    }

    u_long non_blocking = 1;
    if (ioctlsocket(socket_, FIONBIO, &non_blocking) != 0) {
        throw std::runtime_error(socket_error("ioctlsocket(FIONBIO)"));
    }
}

FastExportUdpClient::~FastExportUdpClient() {
    if (socket_ != INVALID_SOCKET) {
        closesocket(socket_);
    }
}

int FastExportUdpClient::pump() {
    std::array<char, 4096> buffer{};
    int packets = 0;
    for (;;) {
        const int received = recv(socket_, buffer.data(), static_cast<int>(buffer.size() - 1), 0);
        if (received > 0) {
            buffer[static_cast<std::size_t>(received)] = '\0';
            const std::string line(buffer.data(), static_cast<std::size_t>(received));
            if (auto parsed = parse_line(line)) {
                latest_ = *parsed;
                last_frame_time_ = std::chrono::steady_clock::now();
            } else if (auto probe = parse_probe_line(line)) {
                probe_values_.push_back(*probe);
            }
            ++packets;
            continue;
        }

        const int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK) {
            return packets;
        }
        throw std::runtime_error(socket_error("recv fast_export"));
    }
}

bool FastExportUdpClient::has_recent_frame(double stale_timeout_seconds) const {
    if (last_frame_time_ == std::chrono::steady_clock::time_point{}) {
        return false;
    }
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_frame_time_).count();
    return age <= stale_timeout_seconds;
}

std::optional<FastExportTelemetry> FastExportUdpClient::latest() const {
    return latest_;
}

std::vector<FastExportProbeValue> FastExportUdpClient::drain_probe_values() {
    std::vector<FastExportProbeValue> values = std::move(probe_values_);
    probe_values_.clear();
    return values;
}

}  // namespace autorudder::windows
