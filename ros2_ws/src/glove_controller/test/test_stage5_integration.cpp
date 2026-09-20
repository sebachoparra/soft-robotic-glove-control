// Copyright 2026 Sebastian Parra

#include <cassert>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "isolated_ros_test.hpp"

#define PICO_BRIDGE_TESTING
#include "../../pico_bridge/src/pico_bridge_node.cpp"  // NOLINT(build/include)
#define main ukf_node_program_main
#include "../../glove_estimator/src/ukf_node.cpp"  // NOLINT(build/include)
#undef main
#define main glove_control_program_main
#include "../src/glove_control_node.cpp"  // NOLINT(build/include)
#undef main

int main(int argc, char ** argv)
{
  init_isolated_test(argc, argv);
  rclcpp::NodeOptions loopback;
  loopback.append_parameter_override("transport", "loopback");
  auto bridge = std::make_shared<PicoBridgeNode>(loopback);
  auto estimator = std::make_shared<UkfNode>();
  auto control = std::make_shared<GloveControlNode>();
  auto observer = std::make_shared<rclcpp::Node>("stage5_integration_observer");
  std::vector<glove_interfaces::msg::SensorFrame> sensors;
  std::vector<glove_interfaces::msg::EstimatorState> estimates;
  std::vector<glove_interfaces::msg::ControllerStatus> controls;
  std::vector<std::string> order;
  auto sensor_sub = observer->create_subscription<glove_interfaces::msg::SensorFrame>(
    "/pico_bridge/sensor_frame", 10,
    [&](glove_interfaces::msg::SensorFrame::ConstSharedPtr msg) {
      sensors.push_back(*msg);
      if (msg->seq == 10u) {order.push_back("sensor");}
    });
  auto estimate_sub = observer->create_subscription<glove_interfaces::msg::EstimatorState>(
    "/ukf/estimator_state", 10,
    [&](glove_interfaces::msg::EstimatorState::ConstSharedPtr msg) {
      estimates.push_back(*msg);
      if (msg->source_seq == 10u) {order.push_back("estimate");}
    });
  auto control_sub = observer->create_subscription<glove_interfaces::msg::ControllerStatus>(
    "/glove_control/controller_status", 10,
    [&](glove_interfaces::msg::ControllerStatus::ConstSharedPtr msg) {
      controls.push_back(*msg);
      if (msg->estimator_source_seq == 10u) {order.push_back("control");}
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(estimator);
  executor.add_node(control);
  executor.add_node(observer);
  auto spin_for = [&](int ms) {
      auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
      while (std::chrono::steady_clock::now() < until) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };

  spin_for(20);
  assert(!controls.empty());
  assert(controls.back().sensor_stale);
  assert(controls.back().estimator_stale);
  const std::string sensor_prefix =
    "S,10,123400,0x1p+0,0x1p-1,0x1p+3,0x1p+3,0x1p+8,0x1p+8,0,0,0,0,0,0,0,";
  const std::string estimator_prefix =
    "E,10,123400,0x1p+8,0,0x1p+0,0x0p+0,0x0p+0,0x0p+0,"
    "0,0x1p+0,0x0p+0,0x0p+0,0x0p+0,";
  bridge->inject_loopback_line(with_crc(sensor_prefix));
  bridge->inject_loopback_line(with_crc(estimator_prefix));
  spin_for(50);
  assert(!sensors.empty() && sensors.back().pico_timestamp_us == 123400u);
  assert(!estimates.empty() && estimates.back().source_timestamp_us == 123400u);
  assert(!controls.empty() && controls.back().estimator_source_seq == 10u);
  assert(!controls.back().sensor_stale);
  assert(!controls.back().control_enabled);
  assert(!estimates.back().theta_valid);  // Both scripted IMUs are unavailable.
  assert(!estimates.back().reference_valid);  // No reference calibration was supplied.
  auto first = [&](const std::string & name) {
      for (size_t i = 0; i < order.size(); ++i) {
        if (order[i] == name) {return i;}
      }
      return order.size();
    };
  assert(first("sensor") < first("estimate"));
  assert(first("estimate") < first("control"));

  const size_t sensor_count = sensors.size();
  std::string malformed = with_crc(sensor_prefix);
  malformed[malformed.size() - 2u] = malformed[malformed.size() - 2u] == '0' ? '1' : '0';
  bridge->inject_loopback_line(malformed);
  spin_for(15);
  assert(bridge->crc_errors() == 1u && sensors.size() == sensor_count);

  // Sensor expiry precedes estimator expiry; neither path fabricates a frame.
  spin_for(55);
  assert(controls.back().sensor_stale);
  assert(!controls.back().estimator_stale);
  spin_for(105);
  assert(controls.back().estimator_stale);

  bridge->simulate_drop();
  assert(!bridge->link_up());
  bridge->inject_loopback_line(with_crc(sensor_prefix));
  assert(sensors.size() == sensor_count);
  bridge->simulate_reconnect();
  assert(!bridge->link_up() && bridge->reconnect_count() == 1u);
  std::string recovered = sensor_prefix;
  recovered.replace(2u, 2u, "11");
  bridge->inject_loopback_line(with_crc(recovered));
  spin_for(20);
  assert(bridge->link_up());
  assert(sensors.back().seq == 11u && !controls.back().sensor_stale);

  // A real core overpressure disposition must traverse the actuator topic.
  std::string high = recovered;
  const std::string low_pressure = "0x1p+3,0x1p+3";
  const size_t pressure_at = high.find(low_pressure);
  assert(pressure_at != std::string::npos);
  high.replace(pressure_at, low_pressure.size(), "0x1.9p+7,0x1.9p+7");
  high.replace(2u, 2u, "12");
  bridge->inject_loopback_line(with_crc(high));
  spin_for(30);
  assert(controls.back().overpressure_latched);
  bool saw_all_off = false;
  for (const auto & line : bridge->captured_lines()) {
    if (line.find(",5,") != std::string::npos) {saw_all_off = true;}
  }
  assert(saw_all_off);

  const size_t shutdown_count = bridge->captured_lines().size();
  recovered.replace(2u, 2u, "13");
  bridge->inject_loopback_line(with_crc(recovered));
  spin_for(20);
  auto command_pub = observer->create_publisher<glove_interfaces::msg::ControlCommand>(
    "/glove_control/control_command", 10);
  glove_interfaces::msg::ControlCommand abort;
  abort.kind = 4u;
  command_pub->publish(abort);
  spin_for(30);
  assert(controls.back().overpressure_latched);
  assert(!controls.back().control_enabled);
  assert(bridge->captured_lines().size() == shutdown_count);
  assert(!bridge->captured_lines().empty());

  executor.remove_node(observer);
  executor.remove_node(control);
  executor.remove_node(estimator);
  executor.remove_node(bridge);
  rclcpp::shutdown();
  return 0;
}
