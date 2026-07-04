#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace autorudder {

struct XmModuleInfo {
    std::string title;
    int channels = 0;
    int patterns = 0;
    int instruments = 0;
    int song_length = 0;
    int default_tempo = 0;
    int default_bpm = 0;
};

XmModuleInfo parse_xm_module_info(std::span<const std::uint8_t> data);

std::vector<std::uint8_t> render_xm_to_wav(
    std::span<const std::uint8_t> data,
    int sample_rate,
    int max_seconds);

}  // namespace autorudder
