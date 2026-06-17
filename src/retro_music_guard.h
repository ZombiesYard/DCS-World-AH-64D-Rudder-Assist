#pragma once

namespace autorudder {

enum class RetroMusicAction {
    None,
    StopForDcs,
    StopForProfileStart,
};

class RetroMusicGuard {
public:
    RetroMusicAction update(bool music_playing, bool dcs_running);
    RetroMusicAction profile_start(bool music_playing);

private:
    bool stopped_for_dcs_ = false;
    bool stopped_for_profile_ = false;
};

}  // namespace autorudder
