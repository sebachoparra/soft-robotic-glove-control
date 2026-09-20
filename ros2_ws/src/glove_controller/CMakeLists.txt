cmake_minimum_required(VERSION 3.16)
project(glove_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glove_interfaces REQUIRED)
find_package(soft_glove_core REQUIRED)

add_executable(glove_control_node src/glove_control_node.cpp)
set_target_properties(glove_control_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO)
target_compile_options(glove_control_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(glove_control_node rclcpp)
target_link_libraries(glove_control_node soft_glove_core::soft_glove_core)

install(TARGETS glove_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright lint awaits the unresolved repository licensing decision.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

find_package(glove_interfaces REQUIRED)
ament_target_dependencies(glove_control_node glove_interfaces)

if(BUILD_TESTING)
  add_executable(test_glove_control_node test/test_glove_control_node.cpp)
  ament_target_dependencies(test_glove_control_node rclcpp glove_interfaces soft_glove_core)
  add_test(NAME glove_control_wrapper COMMAND test_glove_control_node)
  add_executable(test_stage5_integration test/test_stage5_integration.cpp)
  ament_target_dependencies(test_stage5_integration rclcpp glove_interfaces soft_glove_core)
  add_test(NAME stage5_integration COMMAND test_stage5_integration)
  add_executable(test_pressure_control test/test_pressure_control.cpp)
  ament_target_dependencies(test_pressure_control rclcpp glove_interfaces soft_glove_core)
  add_test(NAME pressure_control COMMAND test_pressure_control)
endif()
