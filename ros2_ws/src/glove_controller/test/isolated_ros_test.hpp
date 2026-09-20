// Copyright 2026 Sebastian Parra

#ifndef ISOLATED_ROS_TEST_HPP_
#define ISOLATED_ROS_TEST_HPP_

#include <unistd.h>

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

// Absolute production endpoints must not connect tests to a running glove or
// another test. All communicating nodes share these process-specific remaps.
inline void init_isolated_test(int argc, char ** argv)
{
  const std::string prefix = "/glove_controller_test_" + std::to_string(getpid());
  std::vector<std::string> storage;
  for (int i = 0; i < argc; ++i) {
    storage.emplace_back(argv[i]);
  }
  storage.emplace_back("--ros-args");
  for (const std::string topic : {
    "/glove_control/actuator_command", "/glove_control/controller_status",
    "/glove_control/control_command", "/ukf/estimator_state",
    "/pico_bridge/sensor_frame", "/pico_bridge/estimator_input",
    "/pico_bridge/pico_status", "/pico_bridge/calibrate_sensors"})
  {
    storage.emplace_back("-r");
    storage.push_back(topic + ":=" + prefix + topic);
  }
  std::vector<const char *> args;
  for (const auto & arg : storage) {
    args.push_back(arg.c_str());
  }
  rclcpp::init(static_cast<int>(args.size()), args.data());
}

#endif  // ISOLATED_ROS_TEST_HPP_
