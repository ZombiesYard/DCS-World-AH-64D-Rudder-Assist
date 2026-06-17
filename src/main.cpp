#include "app.h"

#include <cstdio>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>

namespace {

std::string narrow_utf8(const std::wstring& value) {
    if (value.empty()) {
        return {};
    }
    const int size = WideCharToMultiByte(
        CP_UTF8, 0, value.c_str(), static_cast<int>(value.size()), nullptr, 0, nullptr, nullptr);
    std::string result(size, '\0');
    WideCharToMultiByte(
        CP_UTF8, 0, value.c_str(), static_cast<int>(value.size()), result.data(), size, nullptr, nullptr);
    return result;
}

bool has_std_handle(DWORD id) {
    HANDLE handle = GetStdHandle(id);
    if (handle == nullptr || handle == INVALID_HANDLE_VALUE) {
        return false;
    }
    SetLastError(ERROR_SUCCESS);
    const DWORD type = GetFileType(handle);
    return type != FILE_TYPE_UNKNOWN || GetLastError() == ERROR_SUCCESS;
}

void attach_parent_console() {
    const bool has_stdout = has_std_handle(STD_OUTPUT_HANDLE);
    const bool has_stderr = has_std_handle(STD_ERROR_HANDLE);
    const bool has_stdin = has_std_handle(STD_INPUT_HANDLE);
    if (AttachConsole(ATTACH_PARENT_PROCESS) || GetLastError() == ERROR_ACCESS_DENIED) {
        FILE* stream = nullptr;
        if (!has_stdout) {
            freopen_s(&stream, "CONOUT$", "w", stdout);
        }
        if (!has_stderr) {
            freopen_s(&stream, "CONOUT$", "w", stderr);
        }
        if (!has_stdin) {
            freopen_s(&stream, "CONIN$", "r", stdin);
        }
    }
}

}  // namespace
#endif

int main(int argc, char** argv) {
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }
    return autorudder::run_app(args);
}

#ifdef _WIN32
int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR, int) {
    int argc = 0;
    LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    if (argc > 1) {
        attach_parent_console();
    }
    std::vector<std::string> args;
    if (argv) {
        for (int i = 1; i < argc; ++i) {
            args.emplace_back(narrow_utf8(argv[i]));
        }
        LocalFree(argv);
    }
    return autorudder::run_app(args);
}
#endif
