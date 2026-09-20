cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(pico_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(pico_firmware
  src/main.c
  src/crc16_ccitt.c
  src/pico_protocol.c
  src/pico_frame_parser.c
  src/pico_sensors.c
  src/pico_sensor_math.c
  src/pico_actuator.c
  src/pico_watchdog.c)
target_include_directories(pico_firmware PRIVATE include)
target_link_libraries(pico_firmware pico_stdlib hardware_adc hardware_i2c)
pico_enable_stdio_usb(pico_firmware 1)
pico_enable_stdio_uart(pico_firmware 0)
pico_add_extra_outputs(pico_firmware)

