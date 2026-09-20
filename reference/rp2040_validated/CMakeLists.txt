# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# The build script sets PICO_SDK_PATH and places the ARM toolchain in PATH.
include(pico_sdk_import.cmake)

project(glove_ukf_shadow C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_executable(glove_ukf_shadow
    main.c
)

target_include_directories(glove_ukf_shadow PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(glove_ukf_shadow
    pico_stdlib
    hardware_adc
    hardware_i2c
)

# Serial telemetry over USB.
pico_enable_stdio_usb(glove_ukf_shadow 1)
pico_enable_stdio_uart(glove_ukf_shadow 0)

pico_add_extra_outputs(glove_ukf_shadow)
