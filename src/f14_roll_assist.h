#pragma once

#include "config.h"

#include <string>

namespace autorudder {

struct F14RollAssistInput {
    double dt = 0.01;
    double physical_roll = 0.0;
    double physical_rudder = 0.0;
    double yaw_rate_z = 0.0;
    double roll_rate_x = 0.0;
    double slip_ball = 0.0;
    double aoa_units = 0.0;
    double indicated_airspeed = 0.0;
    double radar_altitude = 0.0;
    double gear_position = 0.0;
    double flaps_position = 0.0;
    bool roll_rate_valid = false;
    bool slip_valid = false;
    bool aoa_valid = false;
    bool indicated_airspeed_valid = false;
    bool radar_altitude_valid = false;
    bool gear_valid = false;
    bool flaps_valid = false;
    bool telemetry_fresh = false;
    bool aircraft_is_f14 = false;
    bool roll_input_valid = true;
    bool rudder_input_valid = true;
    bool assist_enabled = true;
};

struct F14RollAssistOutput {
    double final_roll = 0.0;
    double final_rudder = 0.0;
    double roll_scale = 1.0;
    double aoa_weight = 0.0;
    double deep_aoa_weight = 0.0;
    double rudder_assist = 0.0;
    double rudder_from_roll = 0.0;
    double yaw_damping = 0.0;
    double slip_correction = 0.0;
    double reversal_guard = 1.0;
    bool assist_active = false;
    std::string flight_mode;
    std::string reason;
};

class F14RollAssist {
public:
    explicit F14RollAssist(AppConfig config);

    F14RollAssistOutput update(const F14RollAssistInput& input);
    void set_config(AppConfig config);
    void reset();

private:
    AppConfig config_;
    double rudder_assist_ = 0.0;
    double reversal_timer_ = 0.0;
};

}  // namespace autorudder
