#include "retro_toggle.h"

#include <algorithm>

namespace autorudder {

RetroToggleController::RetroToggleController(std::int64_t release_cooldown_ms)
    : release_cooldown_ms_(std::max<std::int64_t>(0, release_cooldown_ms)) {}

RetroToggleAction RetroToggleController::click_profile(RetroProfile profile, std::int64_t now_ms) {
    if (profile == RetroProfile::None) {
        return {};
    }

    if (phase_ == Phase::Stopping) {
        return {RetroToggleActionKind::WaitForRelease, active_profile_};
    }

    if (phase_ == Phase::Starting || phase_ == Phase::Running) {
        if (active_profile_ == profile) {
            phase_ = Phase::Stopping;
            return {RetroToggleActionKind::Stop, profile};
        }
        return {RetroToggleActionKind::BlockedByActiveProfile, active_profile_};
    }

    if (now_ms < restart_allowed_at_ms_) {
        return {RetroToggleActionKind::WaitForRelease, profile};
    }

    phase_ = Phase::Starting;
    active_profile_ = profile;
    return {RetroToggleActionKind::Start, profile};
}

void RetroToggleController::mark_running(RetroProfile profile) {
    if (phase_ == Phase::Starting && active_profile_ == profile) {
        phase_ = Phase::Running;
    }
}

void RetroToggleController::mark_stopped(std::int64_t now_ms) {
    phase_ = Phase::Idle;
    active_profile_ = RetroProfile::None;
    restart_allowed_at_ms_ = std::max(restart_allowed_at_ms_, now_ms + release_cooldown_ms_);
}

RetroProfile RetroToggleController::active_profile() const {
    return active_profile_;
}

bool RetroToggleController::active() const {
    return phase_ == Phase::Starting || phase_ == Phase::Running || phase_ == Phase::Stopping;
}

bool RetroToggleController::stopping() const {
    return phase_ == Phase::Stopping;
}

}  // namespace autorudder
