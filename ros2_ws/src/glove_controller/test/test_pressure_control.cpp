// Copyright 2026 Sebastian Parra

#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
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
  // Deterministic witnesses for the unchanged inner pressure controller.
  assert(SGC_KP_PRESSURE == 0.060f && SGC_KI_PRESSURE == 0.025f);
  assert(SGC_PRESSURE_DEADBAND_KPA == 1.0f);
  assert(SGC_PRESSURE_REF_MIN_KPA == 0.0f && SGC_PRESSURE_REF_MAX_KPA == 170.0f);
  assert(SGC_PRESSURE_LOOP_MS == 100 && SGC_REVERSAL_LOCKOUT_MS == 200);
  assert(SGC_FILL_MIN_MS == 5 && SGC_FILL_MAX_MS == 80);
  assert(SGC_VENT_MIN_MS == 10 && SGC_VENT_MAX_MS == 100);
  assert(SGC_HARD_PRESSURE_KPA == 200.0f);
  SgcSupState sup;
  SgcCtrlState ctrl;
  sgc_sup_init(&sup);
  sgc_ctrl_state_init(&ctrl);
  SgcActions out{};
  sgc_sup_execute_pressure_control(&sup, &ctrl, 20.0f, 0.0f, 0u, &out);
  assert(out.arm_pulse && out.valve_cmd == SGC_STATE_FILL);
  assert(out.arm_pulse_ms >= 5u && out.arm_pulse_ms <= 80u);
  sgc_sup_update_active_pulse(&sup, 100u, &out);
  out = {};
  sgc_sup_execute_pressure_control(&sup, &ctrl, 20.0f, 30.0f, 100u, &out);
  assert(!out.arm_pulse && out.valve_cmd == SGC_STATE_HOLD);
  assert(sup.reversal_block_until_ms == 300u);
  out = {};
  sgc_sup_execute_pressure_control(&sup, &ctrl, 20.0f, 30.0f, 299u, &out);
  assert(!out.arm_pulse && out.valve_cmd == SGC_STATE_HOLD);
  out = {};
  sgc_sup_execute_pressure_control(&sup, &ctrl, 20.0f, 30.0f, 300u, &out);
  assert(out.arm_pulse && out.valve_cmd == SGC_STATE_VENT);
  assert(out.arm_pulse_ms >= 10u && out.arm_pulse_ms <= 100u);
  for (float pressure : {19.0f, 20.0f, 21.0f}) {
    sgc_sup_init(&sup);
    sgc_ctrl_state_init(&ctrl);
    out = {};
    sgc_sup_execute_pressure_control(&sup, &ctrl, 20.0f, pressure, 0u, &out);
    assert(!out.arm_pulse && out.set_valves && out.valve_cmd == SGC_STATE_HOLD);
  }

  init_isolated_test(argc, argv);
  auto control = std::make_shared<GloveControlNode>();
  auto observer = std::make_shared<rclcpp::Node>("pressure_control_test_observer");
  std::vector<glove_interfaces::msg::ActuatorCommand> commands;
  auto subscription = observer->create_subscription<glove_interfaces::msg::ActuatorCommand>(
    "/glove_control/actuator_command", 100,
    [&](glove_interfaces::msg::ActuatorCommand::ConstSharedPtr msg) {commands.push_back(*msg);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  auto drain = [&]() {
      for (int i = 0; i < 20; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };
  drain();
  glove_interfaces::msg::SensorFrame sensor;
  sensor.pressure_kpa = 12.0f;
  sensor.pressure_filtered_kpa = 12.0f;
  control->on_sensor(sensor);
  glove_interfaces::msg::ControlCommand command;
  command.kind = glove_interfaces::msg::ControlCommand::SET_P;
  command.pressure_ref_kpa = 20.0f;
  const float initial = control->ctrl_state().pressure_ref_kpa;
  control->on_command(command);
  assert(control->ctrl_state().pressure_ref_kpa == initial);
  command.kind = 1u;
  command.mode = SGC_CTRL_PRESSURE;
  control->on_command(command);
  control->on_tick();
  drain();
  assert(control->ctrl_state().pressure_ref_kpa == 12.0f);
  assert(commands.size() == 2u && commands[0].command == 3u && commands[1].command == 0u);
  assert(commands[0].pulse_ms == 0u && commands[1].pulse_ms == 0u);
  assert(commands[1].seq == commands[0].seq + 1u);
  assert(control->sup_state().control_enabled);

  command.kind = glove_interfaces::msg::ControlCommand::SET_P;
  for (float pressure : {0.0f, 170.0f, 20.0f, 30.0f, 20.0f}) {
    command.pressure_ref_kpa = pressure;
    control->on_command(command);
    control->on_tick();
    assert(control->ctrl_state().pressure_ref_kpa == pressure);
  }
  for (float pressure : {-0.01f, 170.01f, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()})
  {
    command.pressure_ref_kpa = pressure;
    control->on_command(command);
    assert(control->ctrl_state().pressure_ref_kpa == 20.0f);
  }
  // Leave and re-enter: pressure mode always reseeds from filtered pressure.
  for (auto mode : {SGC_CTRL_NONE, SGC_CTRL_PI, SGC_CTRL_ADRC,
      SGC_CTRL_PI_UKF, SGC_CTRL_ADRC_UKF})
  {
    command.kind = 1u;
    command.mode = mode;
    control->on_command(command);
    control->on_sensor(sensor);
    drain();
    const size_t start = commands.size();
    control->on_tick();
    drain();
    if (mode == SGC_CTRL_NONE) {
      assert(commands.size() == start + 1u);
      assert(commands[start].command == 5u);
    } else {
      assert(commands.size() >= start + 2u);
      assert(commands[start].command == 3u && commands[start].pulse_ms == 0u);
      assert(commands[start + 1u].command == 0u && commands[start + 1u].pulse_ms == 0u);
      assert(commands[start + 1u].seq == commands[start].seq + 1u);
    }
    const float previous = control->ctrl_state().pressure_ref_kpa;
    command.kind = glove_interfaces::msg::ControlCommand::SET_P;
    command.pressure_ref_kpa = 99.0f;
    control->on_command(command);
    assert(control->ctrl_state().pressure_ref_kpa == previous);
  }
  command.kind = 1u;
  command.mode = SGC_CTRL_PRESSURE;
  control->on_command(command);
  control->on_sensor(sensor);
  control->on_tick();
  assert(control->ctrl_state().pressure_ref_kpa == 12.0f);
  executor.remove_node(observer);
  rclcpp::shutdown();
  return 0;
}
