cmake_minimum_required(VERSION 3.20)

project(ah64d_auto_rudder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(autorudder_core
    src/collective_drive.cpp
    src/config.cpp
    src/dcs_bios_protocol.cpp
    src/dcs_bios_refs.cpp
    src/dcs_bios_state.cpp
    src/f14_roll_assist.cpp
    src/logger.cpp
    src/power_feedforward.cpp
    src/retro_music_guard.cpp
    src/retro_xm_player.cpp
    src/retro_toggle.cpp
    src/rudder_input.cpp
    src/tune_session.cpp
    src/yaw_damper.cpp
)

target_include_directories(autorudder_core PUBLIC src)
target_compile_definitions(autorudder_core PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)

if(MSVC)
    target_compile_options(autorudder_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(autorudder_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(WIN32)
    set(AUTORUDDER_APP_SOURCES
        src/app.cpp
        src/main.cpp
        src/windows/dcs_bios_udp_client.cpp
        src/windows/directinput_axis.cpp
        src/windows/fast_export_udp_client.cpp
        src/windows/hotkey.cpp
        src/windows/retro_resources.rc
        src/windows/vjoy_device.cpp
        src/windows/xinput_axis.cpp
    )

    add_executable(ah64d_auto_rudder WIN32 ${AUTORUDDER_APP_SOURCES})
    add_executable(ah64d_auto_rudder_cli ${AUTORUDDER_APP_SOURCES})
    add_executable(f14_rudder_roll_assist WIN32 ${AUTORUDDER_APP_SOURCES})

    foreach(target ah64d_auto_rudder ah64d_auto_rudder_cli f14_rudder_roll_assist)
        target_include_directories(${target} PRIVATE src)
        target_compile_definitions(${target} PRIVATE DIRECTINPUT_VERSION=0x0800)
        target_link_libraries(${target} PRIVATE autorudder_core ws2_32 dinput8 dxguid xinput shell32 user32 gdi32 winmm)
    endforeach()

    target_compile_definitions(ah64d_auto_rudder PRIVATE AUTORUDDER_DEFAULT_PROFILE=\"ah64d\")
    target_compile_definitions(ah64d_auto_rudder_cli PRIVATE AUTORUDDER_DEFAULT_PROFILE=\"ah64d\" AUTORUDDER_DISABLE_GUI_MENU)
    target_compile_definitions(f14_rudder_roll_assist PRIVATE AUTORUDDER_DEFAULT_PROFILE=\"f14\")

    if(MSVC)
        target_compile_options(ah64d_auto_rudder PRIVATE /W4 /permissive-)
        target_compile_options(ah64d_auto_rudder_cli PRIVATE /W4 /permissive-)
        target_compile_options(f14_rudder_roll_assist PRIVATE /W4 /permissive-)
    endif()
endif()

add_executable(autorudder_tests
    tests/test_main.cpp
)

target_link_libraries(autorudder_tests PRIVATE autorudder_core)

enable_testing()
add_test(NAME autorudder_tests COMMAND autorudder_tests)
