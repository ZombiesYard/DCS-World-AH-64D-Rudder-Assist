#include "retro_music_guard.h"

namespace autorudder {

RetroSfxAction retro_sfx_click(bool music_playing, bool dcs_running, bool profile_active) {
    if (music_playing) {
        return RetroSfxAction::Stop;
    }
    if (dcs_running || profile_active) {
        return RetroSfxAction::None;
    }
    return RetroSfxAction::Start;
}

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
