#include "windows/directinput_axis.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include <windows.h>

namespace autorudder::windows {
namespace {

std::string uppercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

bool contains_case_insensitive(const std::string& text, const std::string& needle) {
    if (needle.empty()) {
        return true;
    }
    return uppercase(text).find(uppercase(needle)) != std::string::npos;
}

std::string guid_to_string(const GUID& guid) {
    char buffer[64]{};
    std::snprintf(
        buffer,
        sizeof(buffer),
        "{%08lX-%04hX-%04hX-%02hhX%02hhX-%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX}",
        guid.Data1,
        guid.Data2,
        guid.Data3,
        guid.Data4[0],
        guid.Data4[1],
        guid.Data4[2],
        guid.Data4[3],
        guid.Data4[4],
        guid.Data4[5],
        guid.Data4[6],
        guid.Data4[7]);
    return buffer;
}

void check_hr(HRESULT hr, const char* operation) {
    if (FAILED(hr)) {
        throw std::runtime_error(std::string(operation) + " failed, HRESULT 0x" + std::to_string(static_cast<unsigned long>(hr)));
    }
}

struct ListContext {
    std::vector<DirectInputDeviceInfo> devices;
};

BOOL CALLBACK list_callback(const DIDEVICEINSTANCEA* instance, VOID* user) {
    auto* ctx = static_cast<ListContext*>(user);
    DirectInputDeviceInfo info;
    info.index = static_cast<int>(ctx->devices.size()) + 1;
    info.instance_name = instance->tszInstanceName;
    info.product_name = instance->tszProductName;
    info.instance_guid = guid_to_string(instance->guidInstance);
    info.product_guid = guid_to_string(instance->guidProduct);
    ctx->devices.push_back(std::move(info));
    return DIENUM_CONTINUE;
}

struct SelectContext {
    IDirectInput8A* direct_input = nullptr;
    std::string filter;
    int target_occurrence = 1;
    int seen = 0;
    IDirectInputDevice8A* selected = nullptr;
    std::string selected_name;
};

BOOL CALLBACK select_callback(const DIDEVICEINSTANCEA* instance, VOID* user) {
    auto* ctx = static_cast<SelectContext*>(user);
    const std::string product = instance->tszProductName;
    const std::string instance_name = instance->tszInstanceName;
    const std::string instance_guid = guid_to_string(instance->guidInstance);
    const std::string product_guid = guid_to_string(instance->guidProduct);
    if (!contains_case_insensitive(product, ctx->filter) &&
        !contains_case_insensitive(instance_name, ctx->filter) &&
        !contains_case_insensitive(instance_guid, ctx->filter) &&
        !contains_case_insensitive(product_guid, ctx->filter)) {
        return DIENUM_CONTINUE;
    }

    ++ctx->seen;
    if (ctx->seen != ctx->target_occurrence) {
        return DIENUM_CONTINUE;
    }

    if (SUCCEEDED(ctx->direct_input->CreateDevice(instance->guidInstance, &ctx->selected, nullptr))) {
        const std::string display_name = product.empty() ? instance_name : product;
        ctx->selected_name =
            display_name + " instance=" + instance_guid + " product=" + product_guid;
        return DIENUM_STOP;
    }
    return DIENUM_CONTINUE;
}

BOOL CALLBACK set_axis_range_callback(const DIDEVICEOBJECTINSTANCEA* object, VOID* user) {
    auto* device = static_cast<IDirectInputDevice8A*>(user);
    if ((object->dwType & DIDFT_AXIS) == 0) {
        return DIENUM_CONTINUE;
    }

    DIPROPRANGE range{};
    range.diph.dwSize = sizeof(range);
    range.diph.dwHeaderSize = sizeof(range.diph);
    range.diph.dwObj = object->dwType;
    range.diph.dwHow = DIPH_BYID;
    range.lMin = -32768;
    range.lMax = 32767;
    device->SetProperty(DIPROP_RANGE, &range.diph);
    return DIENUM_CONTINUE;
}

}  // namespace

DirectInputAxisInput::DirectInputAxisInput(
    int occurrence,
    const std::string& name_filter,
    const std::string& axis_name)
    : axis_(parse_axis(axis_name)) {
    check_hr(
        DirectInput8Create(GetModuleHandle(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8A, reinterpret_cast<void**>(&direct_input_), nullptr),
        "DirectInput8Create");

    SelectContext ctx;
    ctx.direct_input = direct_input_;
    ctx.filter = name_filter;
    ctx.target_occurrence = std::max(1, occurrence);
    check_hr(
        direct_input_->EnumDevices(DI8DEVCLASS_GAMECTRL, select_callback, &ctx, DIEDFL_ATTACHEDONLY),
        "EnumDevices");

    if (!ctx.selected) {
        throw std::runtime_error("Could not find DirectInput device matching '" + name_filter + "' occurrence " + std::to_string(occurrence));
    }

    device_ = ctx.selected;
    selected_name_ = ctx.selected_name;
    check_hr(device_->SetDataFormat(&c_dfDIJoystick2), "SetDataFormat");

    HWND hwnd = GetConsoleWindow();
    if (!hwnd) {
        hwnd = GetDesktopWindow();
    }
    check_hr(device_->SetCooperativeLevel(hwnd, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE), "SetCooperativeLevel");
    device_->EnumObjects(set_axis_range_callback, device_, DIDFT_AXIS);
    device_->Acquire();
}

DirectInputAxisInput::~DirectInputAxisInput() {
    if (device_) {
        device_->Unacquire();
        device_->Release();
    }
    if (direct_input_) {
        direct_input_->Release();
    }
}

std::optional<double> DirectInputAxisInput::read() {
    if (!device_) {
        return std::nullopt;
    }

    HRESULT hr = device_->Poll();
    if (FAILED(hr)) {
        device_->Acquire();
    }

    DIJOYSTATE2 state{};
    hr = device_->GetDeviceState(sizeof(state), &state);
    if (FAILED(hr)) {
        device_->Acquire();
        return std::nullopt;
    }

    switch (axis_) {
    case Axis::X: return normalize_long(state.lX);
    case Axis::Y: return normalize_long(state.lY);
    case Axis::Z: return normalize_long(state.lZ);
    case Axis::RX: return normalize_long(state.lRx);
    case Axis::RY: return normalize_long(state.lRy);
    case Axis::RZ: return normalize_long(state.lRz);
    case Axis::Slider0: return normalize_long(state.rglSlider[0]);
    case Axis::Slider1: return normalize_long(state.rglSlider[1]);
    }
    return std::nullopt;
}

std::string DirectInputAxisInput::selected_name() const {
    return selected_name_;
}

std::vector<DirectInputDeviceInfo> DirectInputAxisInput::list_devices() {
    IDirectInput8A* di = nullptr;
    check_hr(
        DirectInput8Create(GetModuleHandle(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8A, reinterpret_cast<void**>(&di), nullptr),
        "DirectInput8Create");
    ListContext ctx;
    const HRESULT hr = di->EnumDevices(DI8DEVCLASS_GAMECTRL, list_callback, &ctx, DIEDFL_ATTACHEDONLY);
    di->Release();
    check_hr(hr, "EnumDevices");
    return ctx.devices;
}

DirectInputAxisInput::Axis DirectInputAxisInput::parse_axis(const std::string& axis_name) {
    const std::string axis = uppercase(axis_name);
    if (axis == "X") return Axis::X;
    if (axis == "Y") return Axis::Y;
    if (axis == "Z") return Axis::Z;
    if (axis == "RX") return Axis::RX;
    if (axis == "RY") return Axis::RY;
    if (axis == "RZ") return Axis::RZ;
    if (axis == "SLIDER0" || axis == "SL0") return Axis::Slider0;
    if (axis == "SLIDER1" || axis == "SL1") return Axis::Slider1;
    throw std::runtime_error("Unsupported axis_name: " + axis_name);
}

double DirectInputAxisInput::normalize_long(long value) {
    const double normalized = static_cast<double>(value) / 32767.0;
    return std::max(-1.0, std::min(1.0, normalized));
}

DirectInputButtonInput::DirectInputButtonInput(
    int occurrence,
    const std::string& name_filter,
    int button_number)
    : button_index_(button_number - 1) {
    if (button_number < 1 || button_number > 128) {
        throw std::runtime_error("button_number must be in [1, 128]");
    }

    check_hr(
        DirectInput8Create(GetModuleHandle(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8A, reinterpret_cast<void**>(&direct_input_), nullptr),
        "DirectInput8Create");

    SelectContext ctx;
    ctx.direct_input = direct_input_;
    ctx.filter = name_filter;
    ctx.target_occurrence = std::max(1, occurrence);
    check_hr(
        direct_input_->EnumDevices(DI8DEVCLASS_GAMECTRL, select_callback, &ctx, DIEDFL_ATTACHEDONLY),
        "EnumDevices");

    if (!ctx.selected) {
        throw std::runtime_error("Could not find DirectInput device matching '" + name_filter + "' occurrence " + std::to_string(occurrence));
    }

    device_ = ctx.selected;
    selected_name_ = ctx.selected_name;
    check_hr(device_->SetDataFormat(&c_dfDIJoystick2), "SetDataFormat");

    HWND hwnd = GetConsoleWindow();
    if (!hwnd) {
        hwnd = GetDesktopWindow();
    }
    check_hr(device_->SetCooperativeLevel(hwnd, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE), "SetCooperativeLevel");
    device_->Acquire();
}

DirectInputButtonInput::~DirectInputButtonInput() {
    if (device_) {
        device_->Unacquire();
        device_->Release();
    }
    if (direct_input_) {
        direct_input_->Release();
    }
}

std::optional<bool> DirectInputButtonInput::read() {
    if (!device_) {
        return std::nullopt;
    }

    HRESULT hr = device_->Poll();
    if (FAILED(hr)) {
        device_->Acquire();
    }

    DIJOYSTATE2 state{};
    hr = device_->GetDeviceState(sizeof(state), &state);
    if (FAILED(hr)) {
        device_->Acquire();
        return std::nullopt;
    }
    return (state.rgbButtons[button_index_] & 0x80) != 0;
}

std::string DirectInputButtonInput::selected_name() const {
    return selected_name_;
}

std::vector<int> DirectInputButtonInput::pressed_buttons(
    int occurrence,
    const std::string& name_filter,
    int max_buttons) {
    max_buttons = std::max(1, std::min(128, max_buttons));

    IDirectInput8A* direct_input = nullptr;
    check_hr(
        DirectInput8Create(GetModuleHandle(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8A, reinterpret_cast<void**>(&direct_input), nullptr),
        "DirectInput8Create");

    SelectContext ctx;
    ctx.direct_input = direct_input;
    ctx.filter = name_filter;
    ctx.target_occurrence = std::max(1, occurrence);
    const HRESULT enum_hr = direct_input->EnumDevices(DI8DEVCLASS_GAMECTRL, select_callback, &ctx, DIEDFL_ATTACHEDONLY);
    if (FAILED(enum_hr)) {
        direct_input->Release();
        check_hr(enum_hr, "EnumDevices");
    }
    if (!ctx.selected) {
        direct_input->Release();
        throw std::runtime_error("Could not find DirectInput device matching '" + name_filter + "' occurrence " + std::to_string(occurrence));
    }

    IDirectInputDevice8A* device = ctx.selected;
    HRESULT hr = device->SetDataFormat(&c_dfDIJoystick2);
    if (SUCCEEDED(hr)) {
        HWND hwnd = GetConsoleWindow();
        if (!hwnd) {
            hwnd = GetDesktopWindow();
        }
        hr = device->SetCooperativeLevel(hwnd, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE);
    }
    if (SUCCEEDED(hr)) {
        device->Acquire();
        device->Poll();
    }

    std::vector<int> buttons;
    DIJOYSTATE2 state{};
    if (SUCCEEDED(hr) && SUCCEEDED(device->GetDeviceState(sizeof(state), &state))) {
        for (int button = 1; button <= max_buttons; ++button) {
            if ((state.rgbButtons[button - 1] & 0x80) != 0) {
                buttons.push_back(button);
            }
        }
    }

    device->Unacquire();
    device->Release();
    direct_input->Release();
    return buttons;
}

}  // namespace autorudder::windows
