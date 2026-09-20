cmake_minimum_required(VERSION 3.16)
project(pico_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glove_interfaces REQUIRED)

add_executable(pico_bridge_node src/pico_bridge_node.cpp)
set_target_properties(pico_bridge_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO)
target_compile_options(pico_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(pico_bridge_node rclcpp)

install(TARGETS pico_bridge_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright lint awaits the unresolved repository licensing decision.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

find_package(glove_interfaces REQUIRED)
ament_target_dependencies(pico_bridge_node glove_interfaces)

if(BUILD_TESTING)
  add_executable(test_pico_bridge_node test/test_pico_bridge_node.cpp)
  target_compile_definitions(test_pico_bridge_node PRIVATE
    PICO_FIRMWARE_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../pico_firmware/tests/fixtures")
  ament_target_dependencies(test_pico_bridge_node rclcpp glove_interfaces)
  add_test(NAME pico_bridge_transport COMMAND test_pico_bridge_node)
endif()

if(BUILD_TESTING)
  # Link actual Pico parser/actuator code to host GPIO spies for end-to-end safety.
  set(FW ${CMAKE_CURRENT_SOURCE_DIR}/../../../pico_firmware)
  add_library(pico_safety_test STATIC
    ${FW}/src/pico_actuator.c ${FW}/src/pico_frame_parser.c
    ${FW}/src/pico_protocol.c ${FW}/src/crc16_ccitt.c)
  target_include_directories(pico_safety_test PUBLIC ${FW}/include)
  target_include_directories(pico_safety_test PRIVATE ${FW}/../tests/equivalence/stubs)
  target_compile_definitions(pico_safety_test PRIVATE
    PICO_ACTUATOR_OUTPUT_ENABLE=0 PICO_ACTUATOR_TEST_GPIO)
  add_executable(test_rx_failsafe test/test_rx_failsafe.cpp)
  target_link_libraries(test_rx_failsafe pico_safety_test)
  ament_target_dependencies(test_rx_failsafe rclcpp glove_interfaces)
  add_test(NAME pico_bridge_rx_failsafe COMMAND test_rx_failsafe)
endif()
