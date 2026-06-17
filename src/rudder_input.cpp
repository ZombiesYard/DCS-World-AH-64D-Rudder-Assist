#include "rudder_input.h"

#include <algorithm>
#include <cmath>

namespace autorudder {
namespace {

double apply_deadzone(double value, double deadzone) {
    const double dz = std::clamp(deadzone, 0.0, 0.95);
    const double magnitude = std::abs(value);
    if (magnitude <= dz) {
        return 0.0;
    }
    const double scaled = (magnitude - dz) / (1.0 - dz);
    return std::copysign(scaled, value);
}

}  // namespace

double apply_rudder_input_calibration(double raw, const AppConfig& cfg) {
    const double center = std::clamp(cfg.rudder_input_center, -0.95, 0.95);
    const double span = raw >= center ? (1.0 - center) : (center + 1.0);
    const double centered = span > 0.000001 ? (raw - center) / span : 0.0;
    const double with_deadzone = apply_deadzone(std::clamp(centered, -1.0, 1.0), cfg.rudder_input_deadzone);
    return std::clamp(with_deadzone * cfg.rudder_input_scale, -1.0, 1.0);
}

}  // namespace autorudder
