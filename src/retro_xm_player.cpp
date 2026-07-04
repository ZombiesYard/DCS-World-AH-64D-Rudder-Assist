#include "retro_xm_player.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string_view>

namespace autorudder {
namespace {

constexpr int kXmNoteKeyOff = 97;
constexpr double kXmBaseC4Rate = 8363.0;

struct Reader {
    std::span<const std::uint8_t> data;
    std::size_t pos = 0;

    void require(std::size_t count) const {
        if (pos + count > data.size()) {
            throw std::runtime_error("truncated XM data");
        }
    }

    std::uint8_t u8() {
        require(1);
        return data[pos++];
    }

    std::uint16_t u16() {
        require(2);
        const std::uint16_t value = static_cast<std::uint16_t>(data[pos]) |
            (static_cast<std::uint16_t>(data[pos + 1]) << 8);
        pos += 2;
        return value;
    }

    std::uint32_t u32() {
        require(4);
        const std::uint32_t value = static_cast<std::uint32_t>(data[pos]) |
            (static_cast<std::uint32_t>(data[pos + 1]) << 8) |
            (static_cast<std::uint32_t>(data[pos + 2]) << 16) |
            (static_cast<std::uint32_t>(data[pos + 3]) << 24);
        pos += 4;
        return value;
    }
};

std::string fixed_string(std::span<const std::uint8_t> data, std::size_t offset, std::size_t count) {
    if (offset + count > data.size()) {
        throw std::runtime_error("truncated XM string");
    }
    std::string text(reinterpret_cast<const char*>(data.data() + offset), count);
    while (!text.empty() && (text.back() == '\0' || text.back() == ' ')) {
        text.pop_back();
    }
    return text;
}

struct XmCell {
    std::uint8_t note = 0;
    std::uint8_t instrument = 0;
    std::uint8_t volume = 0;
    std::uint8_t effect = 0;
    std::uint8_t param = 0;
};

struct XmPattern {
    int rows = 0;
    std::vector<XmCell> cells;
};

struct XmSample {
    std::vector<float> pcm;
    int loop_start = 0;
    int loop_length = 0;
    int volume = 64;
    int finetune = 0;
    int loop_type = 0;
    int panning = 128;
    int relative_note = 0;
};

struct XmInstrument {
    std::array<std::uint8_t, 96> sample_map{};
    std::vector<XmSample> samples;
};

struct XmModule {
    XmModuleInfo info;
    int restart_position = 0;
    bool linear_frequency = true;
    std::vector<std::uint8_t> orders;
    std::vector<XmPattern> patterns;
    std::vector<XmInstrument> instruments;
};

struct XmSampleHeader {
    std::uint32_t length = 0;
    std::uint32_t loop_start = 0;
    std::uint32_t loop_length = 0;
    int volume = 64;
    int finetune = 0;
    int type = 0;
    int panning = 128;
    int relative_note = 0;
};

std::vector<float> decode_sample(std::span<const std::uint8_t> bytes, bool sixteen_bit) {
    std::vector<float> pcm;
    if (sixteen_bit) {
        pcm.reserve(bytes.size() / 2);
        std::uint16_t accumulator = 0;
        for (std::size_t i = 0; i + 1 < bytes.size(); i += 2) {
            const auto delta = static_cast<std::uint16_t>(
                static_cast<std::uint16_t>(bytes[i]) |
                (static_cast<std::uint16_t>(bytes[i + 1]) << 8));
            accumulator += delta;
            pcm.push_back(static_cast<float>(static_cast<std::int16_t>(accumulator) / 32768.0));
        }
    } else {
        pcm.reserve(bytes.size());
        std::uint8_t accumulator = 0;
        for (const std::uint8_t byte : bytes) {
            accumulator = static_cast<std::uint8_t>(accumulator + byte);
            pcm.push_back(static_cast<float>(static_cast<std::int8_t>(accumulator) / 128.0));
        }
    }
    return pcm;
}

XmModule parse_xm(std::span<const std::uint8_t> data) {
    if (data.size() < 80 ||
        std::memcmp(data.data(), "Extended Module: ", 17) != 0 ||
        data[37] != 0x1A) {
        throw std::runtime_error("not an XM module");
    }

    Reader reader{data, 60};
    const std::uint32_t header_size = reader.u32();
    if (header_size < 276 || 60 + header_size > data.size()) {
        throw std::runtime_error("invalid XM header size");
    }

    XmModule module;
    module.info.title = fixed_string(data, 17, 20);
    module.info.song_length = reader.u16();
    module.restart_position = reader.u16();
    module.info.channels = reader.u16();
    module.info.patterns = reader.u16();
    module.info.instruments = reader.u16();
    module.linear_frequency = (reader.u16() & 1U) != 0;
    module.info.default_tempo = reader.u16();
    module.info.default_bpm = reader.u16();

    if (module.info.song_length <= 0 ||
        module.info.channels <= 0 ||
        module.info.channels > 32 ||
        module.info.patterns < 0 ||
        module.info.instruments < 0) {
        throw std::runtime_error("invalid XM dimensions");
    }

    reader.require(256);
    module.orders.assign(data.begin() + reader.pos, data.begin() + reader.pos + module.info.song_length);
    reader.pos = 60 + header_size;

    module.patterns.reserve(module.info.patterns);
    for (int pattern_index = 0; pattern_index < module.info.patterns; ++pattern_index) {
        const std::size_t pattern_start = reader.pos;
        const std::uint32_t pattern_header_size = reader.u32();
        if (pattern_header_size < 9 || pattern_start + pattern_header_size > data.size()) {
            throw std::runtime_error("invalid XM pattern header");
        }
        (void)reader.u8();  // packing type, always 0 in ordinary XM files
        XmPattern pattern;
        pattern.rows = reader.u16();
        const std::uint16_t packed_size = reader.u16();
        reader.pos = pattern_start + pattern_header_size;

        if (pattern.rows <= 0 || pattern.rows > 256) {
            throw std::runtime_error("invalid XM pattern rows");
        }
        pattern.cells.resize(static_cast<std::size_t>(pattern.rows) * module.info.channels);
        const std::size_t packed_end = reader.pos + packed_size;
        if (packed_end > data.size()) {
            throw std::runtime_error("truncated XM pattern data");
        }
        if (packed_size > 0) {
            for (int row = 0; row < pattern.rows; ++row) {
                for (int channel = 0; channel < module.info.channels; ++channel) {
                    XmCell cell;
                    const std::uint8_t first = reader.u8();
                    if ((first & 0x80U) != 0) {
                        if ((first & 0x01U) != 0) cell.note = reader.u8();
                        if ((first & 0x02U) != 0) cell.instrument = reader.u8();
                        if ((first & 0x04U) != 0) cell.volume = reader.u8();
                        if ((first & 0x08U) != 0) cell.effect = reader.u8();
                        if ((first & 0x10U) != 0) cell.param = reader.u8();
                    } else {
                        cell.note = first;
                        cell.instrument = reader.u8();
                        cell.volume = reader.u8();
                        cell.effect = reader.u8();
                        cell.param = reader.u8();
                    }
                    pattern.cells[static_cast<std::size_t>(row) * module.info.channels + channel] = cell;
                }
            }
        }
        reader.pos = packed_end;
        module.patterns.push_back(std::move(pattern));
    }

    module.instruments.reserve(module.info.instruments);
    for (int instrument_index = 0; instrument_index < module.info.instruments; ++instrument_index) {
        const std::size_t instrument_start = reader.pos;
        const std::uint32_t instrument_header_size = reader.u32();
        if (instrument_header_size < 29 || instrument_start + instrument_header_size > data.size()) {
            throw std::runtime_error("invalid XM instrument header");
        }
        reader.pos = instrument_start + 27;
        const std::uint16_t sample_count = reader.u16();
        XmInstrument instrument;
        instrument.sample_map.fill(0);

        std::uint32_t sample_header_size = 0;
        if (sample_count > 0) {
            reader.pos = instrument_start + 29;
            sample_header_size = reader.u32();
            if (sample_header_size < 40) {
                throw std::runtime_error("invalid XM sample header size");
            }
            if (instrument_header_size >= 129) {
                std::copy_n(data.begin() + instrument_start + 33, 96, instrument.sample_map.begin());
            }
        }

        reader.pos = instrument_start + instrument_header_size;
        std::vector<XmSampleHeader> sample_headers;
        sample_headers.reserve(sample_count);
        for (std::uint16_t sample_index = 0; sample_index < sample_count; ++sample_index) {
            const std::size_t sample_header_start = reader.pos;
            XmSampleHeader header;
            header.length = reader.u32();
            header.loop_start = reader.u32();
            header.loop_length = reader.u32();
            header.volume = std::clamp<int>(reader.u8(), 0, 64);
            header.finetune = static_cast<std::int8_t>(reader.u8());
            header.type = reader.u8();
            header.panning = reader.u8();
            header.relative_note = static_cast<std::int8_t>(reader.u8());
            (void)reader.u8();  // reserved
            reader.pos = sample_header_start + sample_header_size;
            sample_headers.push_back(header);
        }

        for (const XmSampleHeader& header : sample_headers) {
            reader.require(header.length);
            const bool sixteen_bit = (header.type & 0x10) != 0;
            const int scale = sixteen_bit ? 2 : 1;
            XmSample sample;
            sample.pcm = decode_sample(data.subspan(reader.pos, header.length), sixteen_bit);
            sample.loop_start = static_cast<int>(header.loop_start / scale);
            sample.loop_length = static_cast<int>(header.loop_length / scale);
            sample.volume = header.volume;
            sample.finetune = header.finetune;
            sample.loop_type = header.type & 0x03;
            sample.panning = header.panning;
            sample.relative_note = header.relative_note;
            if (sample.loop_start < 0 ||
                sample.loop_start >= static_cast<int>(sample.pcm.size()) ||
                sample.loop_length <= 1 ||
                sample.loop_start + sample.loop_length > static_cast<int>(sample.pcm.size())) {
                sample.loop_type = 0;
                sample.loop_start = 0;
                sample.loop_length = 0;
            }
            instrument.samples.push_back(std::move(sample));
            reader.pos += header.length;
        }

        module.instruments.push_back(std::move(instrument));
    }

    return module;
}

double note_step(const XmSample& sample, int note, int sample_rate) {
    const double semitone = static_cast<double>(note - 49 + sample.relative_note) +
        static_cast<double>(sample.finetune) / 128.0;
    const double frequency = kXmBaseC4Rate * std::pow(2.0, semitone / 12.0);
    return frequency / static_cast<double>(sample_rate);
}

struct RenderChannel {
    const XmSample* sample = nullptr;
    int instrument = -1;
    int note = 0;
    double position = 0.0;
    double base_step = 0.0;
    double step = 0.0;
    int volume = 0;
    int panning = 128;
    bool active = false;
    std::uint8_t effect = 0;
    std::uint8_t param = 0;
    std::uint8_t volume_column = 0;
};

void apply_volume_slide(RenderChannel& channel, int up, int down) {
    channel.volume = std::clamp(channel.volume + up - down, 0, 64);
}

void process_tick_effects(RenderChannel& channel, int tick) {
    if (channel.effect == 0x00 && channel.param != 0 && channel.base_step > 0.0) {
        int semitone = 0;
        if (tick % 3 == 1) {
            semitone = channel.param >> 4;
        } else if (tick % 3 == 2) {
            semitone = channel.param & 0x0F;
        }
        channel.step = channel.base_step * std::pow(2.0, static_cast<double>(semitone) / 12.0);
    } else {
        channel.step = channel.base_step;
    }

    if (channel.effect == 0x0A || channel.effect == 0x06) {
        apply_volume_slide(channel, channel.param >> 4, channel.param & 0x0F);
    }
    if (channel.volume_column >= 0x60 && channel.volume_column <= 0x6F) {
        apply_volume_slide(channel, 0, channel.volume_column & 0x0F);
    } else if (channel.volume_column >= 0x70 && channel.volume_column <= 0x7F) {
        apply_volume_slide(channel, channel.volume_column & 0x0F, 0);
    }
}

void write_le16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

void write_le32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

std::vector<std::uint8_t> pcm_to_wav(const std::vector<std::int16_t>& pcm, int sample_rate) {
    const std::uint32_t data_size = static_cast<std::uint32_t>(pcm.size() * sizeof(std::int16_t));
    std::vector<std::uint8_t> wav;
    wav.reserve(44 + data_size);
    wav.insert(wav.end(), {'R', 'I', 'F', 'F'});
    write_le32(wav, 36 + data_size);
    wav.insert(wav.end(), {'W', 'A', 'V', 'E'});
    wav.insert(wav.end(), {'f', 'm', 't', ' '});
    write_le32(wav, 16);
    write_le16(wav, 1);
    write_le16(wav, 2);
    write_le32(wav, static_cast<std::uint32_t>(sample_rate));
    write_le32(wav, static_cast<std::uint32_t>(sample_rate * 2 * sizeof(std::int16_t)));
    write_le16(wav, static_cast<std::uint16_t>(2 * sizeof(std::int16_t)));
    write_le16(wav, 16);
    wav.insert(wav.end(), {'d', 'a', 't', 'a'});
    write_le32(wav, data_size);
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(pcm.data());
    wav.insert(wav.end(), bytes, bytes + data_size);
    return wav;
}

}  // namespace

XmModuleInfo parse_xm_module_info(std::span<const std::uint8_t> data) {
    return parse_xm(data).info;
}

std::vector<std::uint8_t> render_xm_to_wav(
    std::span<const std::uint8_t> data,
    int sample_rate,
    int max_seconds) {
    if (sample_rate < 8000 || sample_rate > 48000) {
        throw std::runtime_error("unsupported XM render sample rate");
    }
    if (max_seconds <= 0 || max_seconds > 600) {
        throw std::runtime_error("unsupported XM render duration");
    }

    const XmModule module = parse_xm(data);
    std::vector<RenderChannel> channels(static_cast<std::size_t>(module.info.channels));
    std::vector<std::int16_t> pcm;
    pcm.reserve(static_cast<std::size_t>(sample_rate) * 30U * 2U);

    int speed = std::max(1, module.info.default_tempo);
    int bpm = std::max(32, module.info.default_bpm);
    int order = 0;
    int row = 0;
    const std::size_t max_frames = static_cast<std::size_t>(sample_rate) * max_seconds;

    auto render_frames = [&](int frames) {
        const double gain = 0.45 / std::sqrt(std::max(1.0, static_cast<double>(channels.size())));
        for (int frame = 0; frame < frames && pcm.size() / 2 < max_frames; ++frame) {
            double left = 0.0;
            double right = 0.0;
            for (RenderChannel& channel : channels) {
                if (!channel.active || !channel.sample || channel.sample->pcm.empty()) {
                    continue;
                }
                const int index = static_cast<int>(channel.position);
                if (index < 0 || index >= static_cast<int>(channel.sample->pcm.size())) {
                    channel.active = false;
                    continue;
                }
                const int next_index = std::min(index + 1, static_cast<int>(channel.sample->pcm.size()) - 1);
                const double frac = channel.position - index;
                const double sample_value =
                    channel.sample->pcm[index] * (1.0 - frac) + channel.sample->pcm[next_index] * frac;
                const double volume = static_cast<double>(channel.volume) / 64.0;
                const double pan = static_cast<double>(channel.panning) / 255.0;
                left += sample_value * volume * (1.0 - pan);
                right += sample_value * volume * pan;

                channel.position += channel.step;
                if (channel.sample->loop_type != 0 && channel.sample->loop_length > 1) {
                    const double loop_start = channel.sample->loop_start;
                    const double loop_end = channel.sample->loop_start + channel.sample->loop_length;
                    while (channel.position >= loop_end) {
                        channel.position = loop_start + std::fmod(channel.position - loop_start, channel.sample->loop_length);
                    }
                } else if (channel.position >= static_cast<double>(channel.sample->pcm.size())) {
                    channel.active = false;
                }
            }
            const auto to_i16 = [](double value) {
                const int scaled = static_cast<int>(std::lround(std::clamp(value, -1.0, 1.0) * 32767.0));
                return static_cast<std::int16_t>(scaled);
            };
            pcm.push_back(to_i16(left * gain));
            pcm.push_back(to_i16(right * gain));
        }
    };

    while (order < module.info.song_length && pcm.size() / 2 < max_frames) {
        const int pattern_index = module.orders[static_cast<std::size_t>(order)];
        if (pattern_index < 0 || pattern_index >= static_cast<int>(module.patterns.size())) {
            ++order;
            row = 0;
            continue;
        }
        const XmPattern& pattern = module.patterns[static_cast<std::size_t>(pattern_index)];
        if (row < 0 || row >= pattern.rows) {
            ++order;
            row = 0;
            continue;
        }

        bool break_pattern = false;
        int break_row = 0;
        for (int channel_index = 0; channel_index < module.info.channels; ++channel_index) {
            const XmCell& cell = pattern.cells[static_cast<std::size_t>(row) * module.info.channels + channel_index];
            RenderChannel& channel = channels[static_cast<std::size_t>(channel_index)];
            if (cell.instrument > 0) {
                channel.instrument = cell.instrument - 1;
            }

            const bool valid_instrument =
                channel.instrument >= 0 &&
                channel.instrument < static_cast<int>(module.instruments.size());
            if (cell.note == kXmNoteKeyOff) {
                channel.active = false;
            } else if (cell.note > 0 && cell.note < kXmNoteKeyOff && valid_instrument) {
                const XmInstrument& instrument = module.instruments[static_cast<std::size_t>(channel.instrument)];
                int sample_index = 0;
                if (!instrument.sample_map.empty()) {
                    sample_index = instrument.sample_map[static_cast<std::size_t>(std::clamp<int>(cell.note - 1, 0, 95))];
                }
                if (sample_index >= 0 && sample_index < static_cast<int>(instrument.samples.size())) {
                    const XmSample& sample = instrument.samples[static_cast<std::size_t>(sample_index)];
                    if (!sample.pcm.empty()) {
                        channel.sample = &sample;
                        channel.note = cell.note;
                        channel.position = 0.0;
                        channel.base_step = note_step(sample, cell.note, sample_rate);
                        channel.step = channel.base_step;
                        channel.volume = sample.volume;
                        channel.panning = sample.panning;
                        channel.active = true;
                    }
                }
            }

            if (cell.volume >= 0x10 && cell.volume <= 0x50) {
                channel.volume = std::clamp<int>(cell.volume - 0x10, 0, 64);
            }
            if (cell.effect == 0x0C) {
                channel.volume = std::clamp<int>(cell.param, 0, 64);
            } else if (cell.effect == 0x0F && cell.param > 0) {
                if (cell.param <= 0x1F) {
                    speed = std::max(1, static_cast<int>(cell.param));
                } else {
                    bpm = std::max(32, static_cast<int>(cell.param));
                }
            } else if (cell.effect == 0x0D) {
                break_pattern = true;
                break_row = std::clamp<int>(((cell.param >> 4) * 10) + (cell.param & 0x0F), 0, 255);
            }

            channel.effect = cell.effect;
            channel.param = cell.param;
            channel.volume_column = cell.volume;
        }

        const int frames_per_tick = std::max(1, static_cast<int>(std::lround(sample_rate * 2.5 / bpm)));
        for (int tick = 0; tick < speed; ++tick) {
            if (tick > 0) {
                for (RenderChannel& channel : channels) {
                    process_tick_effects(channel, tick);
                }
            } else {
                for (RenderChannel& channel : channels) {
                    channel.step = channel.base_step;
                }
            }
            render_frames(frames_per_tick);
        }

        if (break_pattern) {
            ++order;
            row = break_row;
        } else {
            ++row;
            if (row >= pattern.rows) {
                ++order;
                row = 0;
            }
        }
    }

    if (pcm.empty()) {
        pcm.assign(static_cast<std::size_t>(sample_rate / 10) * 2U, 0);
    }
    return pcm_to_wav(pcm, sample_rate);
}

}  // namespace autorudder
