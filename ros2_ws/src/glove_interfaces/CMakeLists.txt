cmake_minimum_required(VERSION 3.16)
project(glove_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorFrame.msg"
  "msg/EstimatorInputFrame.msg"
  "msg/EstimatorState.msg"
  "msg/ActuatorCommand.msg"
  "msg/ControlCommand.msg"
  "msg/ControllerStatus.msg"
  "msg/PicoStatus.msg"
  "srv/CalibrateSensors.srv"
  "srv/CalibrateUkfReference.srv"
  "srv/SelectMode.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright lint awaits the unresolved repository licensing decision.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
