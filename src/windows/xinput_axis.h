#pragma once

#include <optional>
#include <string>
#include <vector>

#include <windows.h>
#include <Xinput.h>

namespace autorudder::windows {

struct XInputDeviceInfo {
    int index = 0;
    DWORD packet_number = 0;
};

class XInputAxisInput {
public:
    XInputAxisInput(int controller_id, const std::string& axis_name);

    std::optional<double> read() const;
    std::string selected_name() const;

    static std::vector<XInputDeviceInfo> list_devices();

private:
    enum class Axis {
        LX,
        LY,
        RX,
        RY,
        LT,
        RT,
    };

    static Axis parse_axis(const std::string& axis_name);
    static double normalize_thumb(SHORT value);
    static double normalize_trigger(BYTE value);

    DWORD user_index_ = 0;
    Axis axis_ = Axis::RT;
};

}  // namespace autorudder::windows
