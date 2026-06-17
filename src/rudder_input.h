#pragma once

#include "config.h"

namespace autorudder {

double apply_rudder_input_calibration(double raw, const AppConfig& cfg);

}  // namespace autorudder
