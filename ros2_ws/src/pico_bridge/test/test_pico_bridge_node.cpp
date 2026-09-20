// Copyright 2026 Sebastian Parra

#include <cassert>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#define PICO_BRIDGE_TESTING
#include "../src/pico_bridge_node.cpp"  // NOLINT(build/include)

int main(int argc, char ** argv)
{
  assert(crc16("123456789") == 0x29b1u);
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.append_parameter_override("transport", "loopback");
  auto clock = std::chrono::steady_clock::time_point{};
  auto bridge = std::make_shared<PicoBridgeNode>(options, [&]() {return clock;});
  auto observer = std::make_shared<rclcpp::Node>("bridge_test_observer");
  std::vector<glove_interfaces::msg::SensorFrame> sensors;
  std::vector<glove_interfaces::msg::EstimatorInputFrame> estimators;
  std::vector<glove_interfaces::msg::PicoStatus> statuses;
  auto sensor_sub = observer->create_subscription<glove_interfaces::msg::SensorFrame>(
    "/pico_bridge/sensor_frame", 10,
    [&](glove_interfaces::msg::SensorFrame::ConstSharedPtr m) {sensors.push_back(*m);});
  auto estimator_sub = observer->create_subscription<glove_interfaces::msg::EstimatorInputFrame>(
    "/pico_bridge/estimator_input", 10,
    [&](glove_interfaces::msg::EstimatorInputFrame::ConstSharedPtr m) {
      estimators.push_back(*m);
    });
  auto status_sub = observer->create_subscription<glove_interfaces::msg::PicoStatus>(
    "/pico_bridge/pico_status", 10,
    [&](glove_interfaces::msg::PicoStatus::ConstSharedPtr m) {statuses.push_back(*m);});
  auto command_pub = observer->create_publisher<glove_interfaces::msg::ActuatorCommand>(
    "/glove_control/actuator_command", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(bridge);
  auto drain = [&]() {
      for (int i = 0; i < 15; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };
  assert(!bridge->link_up());
  const std::string sensor_fields =
    "S,8,123456,0x1p+0,0x1p-1,0x1.8p+3,0x1.6p+3,0x1.f4p+8,0x1.ep+8,"
    "-1,1,31,1,0,2,3,";
  bridge->inject_loopback_line(with_crc(sensor_fields));
  drain();
  assert(sensors.size() == 1u && bridge->rx_frames() == 1u);
  assert(sensors[0].seq == 8u && sensors[0].pico_timestamp_us == 123456u);
  assert(sensors[0].pressure_kpa == 12.0f && sensors[0].pneumatic_state == -1);
  assert(sensors[0].pulse_active && sensors[0].pulse_ms_remaining == 31u);
  assert(sensors[0].header.stamp.sec != 0);
  assert(!statuses.empty() && statuses.back().last_rx_pico_us == 123456u);

  const std::string estimator_fields =
    "E,13,123457,0x1.ep+8,1,0x1p+0,0x0p+0,0x0p+0,0x0p+0,"
    "0,0x1p+0,0x0p+0,0x0p+0,0x0p+0,";
  bridge->inject_loopback_line(with_crc(estimator_fields));
  drain();
  assert(estimators.size() == 1u && bridge->rx_frames() == 2u);
  assert(estimators[0].seq == 13u && estimators[0].pico_timestamp_us == 123457u);
  assert(estimators[0].bno1_ok && !estimators[0].bno2_ok);
  assert(estimators[0].bno1_quat[0] == 1.0f);

  glove_interfaces::msg::ActuatorCommand acknowledge;
  acknowledge.command = 5u;
  bridge->on_actuator(acknowledge);
  glove_interfaces::msg::ActuatorCommand command;
  command.seq = 71u;
  command.command = 2u;
  command.pulse_ms = 19u;
  command.deadline_us = 999u;
  command_pub->publish(command);
  drain();
  assert(bridge->tx_frames() == 2u && bridge->captured_lines().size() == 2u);
  assert(bridge->captured_lines()[1] == with_crc("A,71,2,19,999,"));

  const size_t applied_s = sensors.size();
  const size_t applied_e = estimators.size();
  const uint32_t applied = bridge->rx_frames();
  std::string bad_crc = with_crc(sensor_fields);
  bad_crc[bad_crc.size() - 2u] = bad_crc[bad_crc.size() - 2u] == '0' ? '1' : '0';
  bridge->inject_loopback_line(bad_crc);
  assert(bridge->crc_errors() == 1u);
  bridge->inject_loopback_line(with_crc("X,8,123456,"));
  assert(bridge->framing_errors() == 1u);
  bridge->inject_loopback_line(with_crc(sensor_fields).substr(0u,
    with_crc(sensor_fields).size() - 1u));
  assert(bridge->truncated_errors() == 1u);
  bridge->inject_loopback_line(with_crc("S,8,123456,0x1p+0,"));
  assert(bridge->short_fields() == 1u);
  bridge->inject_loopback_line(with_crc(sensor_fields + "4,"));
  assert(bridge->long_fields() == 1u);
  std::string invalid_number = sensor_fields;
  invalid_number.replace(invalid_number.find("0x1.8p+3"), 8u, "wrongnum");
  bridge->inject_loopback_line(with_crc(invalid_number));
  assert(bridge->numeric_errors() == 1u);
  drain();
  assert(sensors.size() == applied_s && estimators.size() == applied_e);
  assert(bridge->rx_frames() == applied);

  bridge->inject_loopback_line(with_crc(sensor_fields));
  assert(bridge->seq_repeats() == 1u);
  std::string gap = sensor_fields;
  gap.replace(2u, 1u, "12");
  bridge->inject_loopback_line(with_crc(gap));
  assert(bridge->seq_gaps() == 1u);
  drain();
  assert(sensors.size() == applied_s + 2u);
  assert(sensors.back().seq == 12u);

  bridge->simulate_drop();
  assert(!bridge->link_up());
  const uint32_t before_drop = bridge->rx_frames();
  bridge->inject_loopback_line(with_crc(sensor_fields));
  assert(bridge->rx_frames() == before_drop);
  bridge->simulate_reconnect();
  assert(!bridge->link_up() && bridge->reconnect_count() == 1u);
  bridge->inject_loopback_line(with_crc(sensor_fields));
  assert(bridge->rx_frames() == before_drop + 1u);
  clock += std::chrono::milliseconds(100);
  bridge->test_poll_transport();
  assert(!bridge->link_up());
  const auto stale_tx = bridge->tx_frames();
  bridge->on_actuator(command);
  assert(bridge->tx_frames() == stale_tx);
  drain();
  assert(!statuses.back().link_up);

  // The service must wait for the matching CRC-checked Pico result.
  bridge->simulate_reconnect();
  bridge->inject_loopback_line(with_crc(sensor_fields));
  auto client = observer->create_client<glove_interfaces::srv::CalibrateSensors>(
    "/pico_bridge/calibrate_sensors");
  assert(client->wait_for_service(std::chrono::seconds(1)));
  auto future = client->async_send_request(
    std::make_shared<glove_interfaces::srv::CalibrateSensors::Request>());
  drain();
  assert(bridge->captured_lines().back() == with_crc("C,1,"));
  bridge->inject_loopback_line(with_crc("Z,999,1,0x1p-1,0x1p+10,"));
  drain();
  assert(future.wait_for(std::chrono::seconds(0)) != std::future_status::ready);
  bridge->inject_loopback_line(with_crc("Z,1,1,0x1p-1,0x1p+10,"));
  drain();
  assert(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
  auto calibrated = future.get();
  assert(calibrated->success && calibrated->pressure_zero_vout == 0.5f);
  assert(calibrated->flex_zero_raw == 1024.0f);
  auto rejected = client->async_send_request(
    std::make_shared<glove_interfaces::srv::CalibrateSensors::Request>());
  drain();
  bridge->inject_loopback_line(with_crc("Z,2,0,0x0p+0,0x0p+0,"));
  drain();
  assert(rejected.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
  assert(!rejected.get()->success);
  auto disconnected = client->async_send_request(
    std::make_shared<glove_interfaces::srv::CalibrateSensors::Request>());
  drain();
  bridge->simulate_drop();
  drain();
  assert(disconnected.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
  assert(!disconnected.get()->success);

  // Decode the exact bytes emitted by the real firmware main-loop test (T6/T7).
  bridge->simulate_reconnect();
  for (const auto & name : {"estimator-valid.txt", "estimator-invalid.txt"}) {
    std::ifstream file(std::string(PICO_FIRMWARE_FIXTURE_DIR) + "/" + name);
    std::string wire;
    assert(std::getline(file, wire));
    bridge->inject_loopback_line(wire + "\n");
    drain();
    const auto & e = estimators.back();
    const bool valid = std::string(name) == "estimator-valid.txt";
    assert(e.bno1_ok == valid && e.bno2_ok == valid);
    assert(e.seq == (valid ? 0u : 1u));
    assert(e.pico_timestamp_us == (valid ? 0u : 20000u));
    assert(e.flex_filtered_raw == 2000);
    assert(e.bno1_quat[0] == 1 && e.bno1_quat[1] == 0);
    assert(e.bno2_quat[0] == 0 && e.bno2_quat[1] == 1);
  }

  // Serial RX freshness and Pico GPIO consequences are covered by test_rx_failsafe.

  executor.remove_node(observer);
  executor.remove_node(bridge);
  bridge.reset();
  observer.reset();
  rclcpp::shutdown();
  return 0;
}
