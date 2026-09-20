cmake_minimum_required(VERSION 3.16)
project(glove_estimator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glove_interfaces REQUIRED)
find_package(soft_glove_core REQUIRED)

add_executable(ukf_node src/ukf_node.cpp)
set_target_properties(ukf_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO)
target_compile_options(ukf_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(ukf_node rclcpp)
target_link_libraries(ukf_node soft_glove_core::soft_glove_core)

install(TARGETS ukf_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright lint awaits the unresolved repository licensing decision.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

find_package(glove_interfaces REQUIRED)
ament_target_dependencies(ukf_node glove_interfaces)

if(BUILD_TESTING)
  add_executable(test_ukf_node test/test_ukf_node.cpp)
  ament_target_dependencies(test_ukf_node rclcpp glove_interfaces soft_glove_core)
  add_test(NAME ukf_node_wrapper COMMAND test_ukf_node)
endif()
