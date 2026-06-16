#include "windows/xinput_axis.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace autorudder::windows {
namespace {

std::string uppercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

double clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

}  // namespace

XInputAxisInput::XInputAxisInput(int controller_id, const std::string& axis_name)
    : axis_(parse_axis(axis_name)) {
    if (controller_id < 1 || controller_id > 4) {
        throw std::runtime_error("XInput controller id must be between 1 and 4");
    }
    user_index_ = static_cast<DWORD>(controller_id - 1);

    XINPUT_STATE state{};
    if (XInputGetState(user_index_, &state) != ERROR_SUCCESS) {
        throw std::runtime_error("XInput controller #" + std::to_string(controller_id) + " is not connected");
    }
}

std::optional<double> XInputAxisInput::read() const {
    XINPUT_STATE state{};
    if (XInputGetState(user_index_, &state) != ERROR_SUCCESS) {
        return std::nullopt;
    }

    const XINPUT_GAMEPAD& pad = state.Gamepad;
    switch (axis_) {
    case Axis::LX: return normalize_thumb(pad.sThumbLX);
    case Axis::LY: return normalize_thumb(pad.sThumbLY);
    case Axis::RX: return normalize_thumb(pad.sThumbRX);
    case Axis::RY: return normalize_thumb(pad.sThumbRY);
    case Axis::LT: return normalize_trigger(pad.bLeftTrigger);
    case Axis::RT: return normalize_trigger(pad.bRightTrigger);
    }
    return std::nullopt;
}

std::string XInputAxisInput::selected_name() const {
    return "XInput controller #" + std::to_string(user_index_ + 1);
}

std::vector<XInputDeviceInfo> XInputAxisInput::list_devices() {
    std::vector<XInputDeviceInfo> devices;
    for (DWORD index = 0; index < XUSER_MAX_COUNT; ++index) {
        XINPUT_STATE state{};
        if (XInputGetState(index, &state) == ERROR_SUCCESS) {
            XInputDeviceInfo info;
            info.index = static_cast<int>(index) + 1;
            info.packet_number = state.dwPacketNumber;
            devices.push_back(info);
        }
    }
    return devices;
}

XInputAxisInput::Axis XInputAxisInput::parse_axis(const std::string& axis_name) {
    const std::string axis = uppercase(axis_name);
    if (axis == "LX" || axis == "LEFTX") return Axis::LX;
    if (axis == "LY" || axis == "LEFTY") return Axis::LY;
    if (axis == "RX" || axis == "RIGHTX") return Axis::RX;
    if (axis == "RY" || axis == "RIGHTY") return Axis::RY;
    if (axis == "LT" || axis == "LEFTTRIGGER") return Axis::LT;
    if (axis == "RT" || axis == "RIGHTTRIGGER") return Axis::RT;
    throw std::runtime_error("Unsupported XInput axis_name: " + axis_name);
}

double XInputAxisInput::normalize_thumb(SHORT value) {
    const double scale = value < 0 ? 32768.0 : 32767.0;
    return clamp(static_cast<double>(value) / scale, -1.0, 1.0);
}

double XInputAxisInput::normalize_trigger(BYTE value) {
    return clamp((static_cast<double>(value) / 255.0) * 2.0 - 1.0, -1.0, 1.0);
}

}  // namespace autorudder::windows
