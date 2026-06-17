#include "retro_music_guard.h"

namespace autorudder {

RetroMusicAction RetroMusicGuard::update(bool music_playing, bool dcs_running) {
    if (music_playing && dcs_running && !stopped_for_dcs_) {
        stopped_for_dcs_ = true;
        return RetroMusicAction::StopForDcs;
    }
    return RetroMusicAction::None;
}

RetroMusicAction RetroMusicGuard::profile_start(bool music_playing) {
    if (music_playing && !stopped_for_profile_) {
        stopped_for_profile_ = true;
        return RetroMusicAction::StopForProfileStart;
    }
    return RetroMusicAction::None;
}

}  // namespace autorudder
