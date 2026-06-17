#include "collective_drive.h"

#include <algorithm>
#include <cmath>

namespace autorudder {
namespace {

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double move_towards(double current, double target, double max_delta) {
    if (current < target) {
        return std::min(target, current + max_delta);
    }
    return std::max(target, current - max_delta);
}

}  // namespace

CollectiveDrive::CollectiveDrive(CollectiveDriveConfig config) : config_(config) {}

void CollectiveDrive::reset() {
    has_output_ = false;
    elapsed_ = 0.0;
    output_collective_ = 0.0;
}

CollectiveDriveOutput CollectiveDrive::update(double dt, double physical_collective, bool enabled) {
    dt = clamp(dt, 0.001, 0.25);
    physical_collective = clamp(physical_collective, 0.0, 1.0);

    if (!enabled) {
        has_output_ = true;
        elapsed_ = 0.0;
        output_collective_ = physical_collective;
        return {output_collective_, 0.0, false};
    }

    if (!has_output_) {
        output_collective_ = physical_collective;
        has_output_ = true;
    }

    elapsed_ += dt;
    double target_offset = 0.0;
    bool driving = false;
    if (elapsed_ >= config_.settle_time && config_.amplitude > 0.0 && config_.period > 0.0) {
        const double phase = std::fmod(elapsed_ - config_.settle_time, config_.period);
        target_offset = phase < config_.period * 0.5 ? config_.amplitude : -config_.amplitude;
        driving = true;
    }

    const double target = clamp(physical_collective + target_offset, 0.0, 1.0);
    if (config_.rate_limit <= 0.0) {
        output_collective_ = target;
    } else {
        output_collective_ = move_towards(output_collective_, target, config_.rate_limit * dt);
    }

    return {output_collective_, output_collective_ - physical_collective, driving};
}

}  // namespace autorudder
