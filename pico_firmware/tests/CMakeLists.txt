cmake_minimum_required(VERSION 3.16)
project(pico_host_tests C)
enable_testing()
set(CMAKE_C_STANDARD 11)
set(FW ${CMAKE_CURRENT_LIST_DIR}/..)
set(STUBS ${FW}/../tests/equivalence/stubs)
add_library(pico_host STATIC
  ${FW}/src/crc16_ccitt.c ${FW}/src/pico_protocol.c ${FW}/src/pico_frame_parser.c
  ${FW}/src/pico_sensor_math.c ${FW}/src/pico_actuator.c ${FW}/src/pico_watchdog.c)
target_include_directories(pico_host PUBLIC ${FW}/include)
target_compile_definitions(pico_host PUBLIC PICO_ACTUATOR_OUTPUT_ENABLE=0)
target_link_libraries(pico_host PUBLIC m)
foreach(name test_pico_protocol test_final_firmware)
  add_executable(${name} ${name}.c)
  target_link_libraries(${name} pico_host)
  add_test(NAME ${name} COMMAND ${name})
endforeach()
# Exercise the real GPIO mapping against host spies, never the physical GPIO backend.
add_executable(test_pressure_hardware test_pressure_hardware.c
  ${FW}/src/pico_actuator.c ${FW}/src/pico_sensors.c)
target_compile_definitions(test_pressure_hardware PRIVATE PICO_ACTUATOR_TEST_GPIO)
target_include_directories(test_pressure_hardware PRIVATE ${STUBS})
target_link_libraries(test_pressure_hardware pico_host)
add_test(NAME test_pressure_hardware COMMAND test_pressure_hardware)

# Execute production main with bounded host time and captured USB output.
add_executable(test_main_loop test_main_loop.c
  ${FW}/src/pico_actuator.c ${FW}/src/pico_sensors.c)
target_compile_definitions(test_main_loop PRIVATE PICO_ACTUATOR_TEST_GPIO LIB_PICO_STDIO_USB=1
  PICO_FIRMWARE_FIXTURE_DIR="${CMAKE_CURRENT_LIST_DIR}/fixtures")
target_include_directories(test_main_loop PRIVATE ${CMAKE_CURRENT_LIST_DIR}/stubs ${STUBS})
target_link_libraries(test_main_loop pico_host)
add_test(NAME test_main_loop COMMAND test_main_loop)
