// Copyright 2026 Sebastian Parra

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "isolated_ros_test.hpp"

#define GLOVE_CONTROL_TESTING
#define main glove_control_program_main
#include "../src/glove_control_node.cpp"  // NOLINT(build/include)
#undef main

int main(int argc, char ** argv)
{
  init_isolated_test(argc, argv);
  auto control = std::make_shared<GloveControlNode>();
  auto observer = std::make_shared<rclcpp::Node>("glove_control_test_observer");
  std::vector<glove_interfaces::msg::ActuatorCommand> commands;
  std::vector<glove_interfaces::msg::ControllerStatus> statuses;
  auto command_sub = observer->create_subscription<glove_interfaces::msg::ActuatorCommand>(
    "/glove_control/actuator_command", 10,
    [&](glove_interfaces::msg::ActuatorCommand::ConstSharedPtr msg) {
      commands.push_back(*msg);
    });
  auto status_sub = observer->create_subscription<glove_interfaces::msg::ControllerStatus>(
    "/glove_control/controller_status", 10,
    [&](glove_interfaces::msg::ControllerStatus::ConstSharedPtr msg) {
      statuses.push_back(*msg);
    });
  rclcpp::executors::SingleThreadedExecutor observer_executor;
  observer_executor.add_node(observer);
  auto drain = [&]() {
      for (int i = 0; i < 20; ++i) {
        observer_executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };

  glove_interfaces::msg::SensorFrame sensor;
  sensor.pressure_kpa = 12.5f;
  sensor.pressure_filtered_kpa = 11.25f;
  sensor.flex_filtered_raw = 500.0f;
  sensor.pico_timestamp_us = 123456u;
  control->on_sensor(sensor);
  glove_interfaces::msg::EstimatorState estimator;
  estimator.source_seq = 7u;
  estimator.source_timestamp_us = 123400u;
  estimator.valid = true;
  estimator.reference_valid = true;
  estimator.theta_valid = true;
  estimator.s_hat = 24.0f;
  control->on_estimator(estimator);

  rclcpp::Clock clock(RCL_STEADY_TIME);
  const uint32_t before = static_cast<uint32_t>(clock.now().nanoseconds() / 1000000);
  const uint64_t calls_before = control->tick_calls();
  control->on_tick();
  const uint32_t after = static_cast<uint32_t>(clock.now().nanoseconds() / 1000000);
  const SgcTickInput & first = control->last_input();
  assert(control->tick_calls() == calls_before + 1u);
  assert(static_cast<int32_t>(first.now_ms - before) >= 0);
  assert(static_cast<int32_t>(after - first.now_ms) >= 0);
  assert(first.pressure_kpa == sensor.pressure_kpa);
  assert(first.pressure_filtered == sensor.pressure_filtered_kpa);
  assert(first.flex_filtered_raw == sensor.flex_filtered_raw);
  assert(first.latest_position_pct == sgc_clampf(
    sgc_flex_raw_to_position_unclipped(sensor.flex_filtered_raw, 0.0f), 0.0f, 100.0f));
  assert(!first.cmd_abort && !first.set_enable && !first.set_mode &&
    !first.set_reset && !first.set_logging && !first.set_q);
  assert(control->ukf_state().telem.valid);
  drain();
  assert(!statuses.empty());
  assert(statuses.back().estimator_source_seq == 7u);
  assert(statuses.back().estimator_acquisition_age_us == 56u);

  // All five frozen command kinds are queued and passed to the next core tick.
  glove_interfaces::msg::ControlCommand command;
  command.kind = 0u;
  command.q_ref_pct = 37.5f;
  control->on_command(command);
  command.kind = 1u;
  command.mode = static_cast<uint8_t>(SGC_CTRL_PI);
  control->on_command(command);
  command.kind = 2u;
  command.enable = true;
  control->on_command(command);
  command.kind = 3u;
  control->on_command(command);
  const uint64_t before_events = control->tick_calls();
  control->on_tick();
  assert(control->tick_calls() == before_events + 1u);
  const SgcTickInput & events = control->last_input();
  assert(events.set_q && events.q_value == 37.5f);
  assert(events.set_mode && events.mode_value == SGC_CTRL_PI);
  assert(events.set_logging && events.logging_value);
  assert(events.set_reset);
  assert(!events.cmd_abort && !events.set_enable);
  control->on_tick();
  const SgcTickInput & cleared = control->last_input();
  assert(!cleared.set_q && !cleared.set_mode && !cleared.set_logging && !cleared.set_reset);

  SgcActions actions{};
  actions.set_valves = true;
  actions.valve_cmd = SGC_STATE_VENT;
  actions.arm_pulse = true;
  actions.arm_pulse_ms = 42u;
  actions.pump_off = true;
  actions.outputs_off = true;
  drain();
  const size_t command_start = commands.size();
  control->test_publish_actions(actions);
  drain();
  assert(commands.size() == command_start + 4u);
  assert(commands[command_start].command == 2u && commands[command_start].pulse_ms == 0u);
  assert(commands[command_start + 1u].command == 2u &&
    commands[command_start + 1u].pulse_ms == 42u);
  assert(commands[command_start + 2u].command == 4u);
  assert(commands[command_start + 3u].command == 5u);
  for (size_t i = command_start + 1u; i < commands.size(); ++i) {
    assert(commands[i].seq == commands[i - 1u].seq + 1u);
  }

  // Expired estimator telemetry must enter the validated core's unavailable route.
  // Only the observer is spun: no timer or estimator subscription can refresh
  // the controller. Wait for expiry and its actual status delivery, not a sleep.
  const auto expiry_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  const size_t expiry_start = statuses.size();
  do {
    control->on_sensor(sensor);
    control->on_tick();
    observer_executor.spin_some();
    if (statuses.size() > expiry_start && statuses.back().estimator_stale) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < expiry_deadline);
  assert(statuses.size() > expiry_start);
  assert(!control->ukf_state().telem.valid);
  assert(statuses.back().estimator_stale);
  assert(!statuses.back().sensor_stale);
  assert(!sgc_ctrl_ukf_feedback_available(&control->ukf_state().telem));

  command.kind = 4u;
  control->on_command(command);
  const uint64_t before_abort = control->tick_calls();
  const size_t abort_start = commands.size();
  control->on_tick();
  drain();
  assert(control->tick_calls() == before_abort + 1u);
  assert(control->last_input().cmd_abort);
  assert(statuses.back().abort_active);
  assert(!control->last_actions().arm_pulse);

  assert(commands.size() == abort_start + 3u);
  assert(commands[abort_start].command == 2u);
  assert(commands[abort_start + 1u].command == 4u);
  assert(commands[abort_start + 2u].command == 5u);

  auto check_terminal = [&](bool aborted) {
      const size_t count = commands.size();
      const uint64_t ticks = control->tick_calls();
      for (int i = 0; i < 128; ++i) {
        // Try every command, including reset, reference and each active mode.
        command.kind = static_cast<uint8_t>(i % 5);
        command.mode = static_cast<uint8_t>(1 + (i / 5) % 5);
        command.q_ref_pct = 100.0f;
        command.enable = true;
        control->on_command(command);
        control->on_sensor(sensor);
        control->on_tick();
        observer_executor.spin_some();
      }
      drain();
      assert(commands.size() == count);
      assert(control->tick_calls() == ticks);
      assert(statuses.back().abort_active == aborted);
      assert(statuses.back().overpressure_latched == !aborted);
      assert(!statuses.back().control_enabled);
      assert(control->sup_state().controller_mode == SGC_CTRL_NONE);
      assert(!control->sup_state().pulse_active);
      assert(!statuses.back().ran_pressure && !statuses.back().ran_outer);
    };
  check_terminal(true);

  control = std::make_shared<GloveControlNode>();
  drain();
  sensor.pressure_kpa = SGC_HARD_PRESSURE_KPA;
  control->on_sensor(sensor);
  const size_t pressure_start = commands.size();
  control->on_tick();
  drain();
  assert(commands.size() == pressure_start + 3u);
  assert(commands[pressure_start].command == 2u);
  assert(commands[pressure_start + 1u].command == 4u);
  assert(commands[pressure_start + 2u].command == 5u);
  sensor.pressure_kpa = 0.0f;
  check_terminal(false);

  // Compare normal PI and PI_UKF actuator traffic with the untouched scheduler.
  for (auto mode : {SGC_CTRL_PI, SGC_CTRL_PI_UKF}) {
    control = std::make_shared<GloveControlNode>();
    drain();
    sensor.pressure_filtered_kpa = 0.0f;
    sensor.flex_filtered_raw = 0.0f;
    estimator.s_hat = 0.0f;
    SgcSupState expected_sup;
    SgcCtrlState expected_ctrl;
    SgcSchedState expected_sched{};
    sgc_sup_init(&expected_sup);
    sgc_ctrl_state_init(&expected_ctrl);
    command.kind = 1u;
    command.mode = static_cast<uint8_t>(mode);
    control->on_command(command);
    command.kind = 0u;
    command.q_ref_pct = 100.0f;
    control->on_command(command);
    bool saw_pulse = false;
    for (int i = 0; i < 12; ++i) {
      control->on_sensor(sensor);
      control->on_estimator(estimator);
      auto expected_ukf = control->ukf_state();
      const size_t start = commands.size();
      control->on_tick();
      SgcActions expected{};
      assert(sgc_sched_tick(&expected_sched, &expected_sup, &expected_ukf,
        &expected_ctrl, &control->last_input(), &expected) == 0);
      drain();
      size_t index = start;
      const uint8_t valve = expected.valve_cmd == SGC_STATE_FILL ? 1u :
        (expected.valve_cmd == SGC_STATE_VENT ? 2u : 0u);
      if (i == 0) {
        assert(index + 2u <= commands.size());
        assert(commands[index].command == 3u && commands[index].pulse_ms == 0u);
        assert(commands[index + 1u].command == 0u && commands[index + 1u].pulse_ms == 0u);
        assert(commands[index + 1u].seq == commands[index].seq + 1u);
        index += 2u;
        expected_sup.pneumatic_state = SGC_STATE_HOLD;
        if (expected.set_valves && expected.valve_cmd == SGC_STATE_HOLD) {
          expected.set_valves = false;
        }
      }
      if (expected.set_valves) {
        assert(index < commands.size());
        assert(commands[index].command == valve && commands[index].pulse_ms == 0u);
        ++index;
      }
      if (expected.arm_pulse) {
        assert(index < commands.size());
        assert(commands[index].command == valve);
        assert(commands[index].pulse_ms == expected.arm_pulse_ms);
        ++index;
        saw_pulse = true;
      }
      assert(index == commands.size());
    }
    assert(saw_pulse);
    // Reach an active pulse before requesting NONE.
    for (int i = 0; i < 50 && !control->sup_state().pulse_active; ++i) {
      control->on_sensor(sensor);
      control->on_estimator(estimator);
      control->on_tick();
      drain();
    }
    assert(control->sup_state().pulse_active);
    command.kind = 1u;
    command.mode = static_cast<uint8_t>(SGC_CTRL_NONE);
    control->on_command(command);
    const size_t none_start = commands.size();
    control->on_tick();
    drain();
    assert(commands.size() == none_start + 1u);
    assert(commands.back().command == 5u && commands.back().pulse_ms == 0u);
    assert(control->last_actions().cancel_pulse);
    assert(!control->sup_state().pulse_active);
    assert(!statuses.back().control_enabled);
    for (int i = 0; i < 128; ++i) {
      control->on_tick();
      observer_executor.spin_some();
    }
    drain();
    assert(commands.size() == none_start + 1u);
  }

  observer_executor.remove_node(observer);
  control.reset();
  observer.reset();
  rclcpp::shutdown();
  return 0;
}
