#pragma once

#include <cstdint>

namespace autorudder {

enum class RetroProfile {
    None,
    Ah64d,
    F14,
};

enum class RetroToggleActionKind {
    None,
    Start,
    Stop,
    WaitForRelease,
    BlockedByActiveProfile,
};

struct RetroToggleAction {
    RetroToggleActionKind kind = RetroToggleActionKind::None;
    RetroProfile profile = RetroProfile::None;
};

class RetroToggleController {
public:
    explicit RetroToggleController(std::int64_t release_cooldown_ms = 1200);

    RetroToggleAction click_profile(RetroProfile profile, std::int64_t now_ms);
    void mark_running(RetroProfile profile);
    void mark_stopped(std::int64_t now_ms);

    RetroProfile active_profile() const;
    bool active() const;
    bool stopping() const;

private:
    enum class Phase {
        Idle,
        Starting,
        Running,
        Stopping,
    };

    std::int64_t release_cooldown_ms_ = 1200;
    std::int64_t restart_allowed_at_ms_ = 0;
    Phase phase_ = Phase::Idle;
    RetroProfile active_profile_ = RetroProfile::None;
};

}  // namespace autorudder
