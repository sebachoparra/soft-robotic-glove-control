// Copyright 2026 Sebastian Parra

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "glove_interfaces/msg/actuator_command.hpp"
#include "glove_interfaces/msg/control_command.hpp"
#include "glove_interfaces/msg/controller_status.hpp"
#include "glove_interfaces/msg/estimator_state.hpp"
#include "glove_interfaces/msg/sensor_frame.hpp"
#include "rclcpp/rclcpp.hpp"
extern "C" {
#include "soft_glove_core/soft_glove_core_sched.h"
}

class GloveControlNode : public rclcpp::Node
{
public:
  GloveControlNode()
  : Node("glove_control_node"),
    steady_clock_(RCL_STEADY_TIME),
    base_tick_ms_(declare_parameter<int>("base_tick_ms", 1)),
    estimator_max_age_ms_(declare_parameter<int>("estimator_max_age_ms", 150)),
    sensor_max_age_ms_(declare_parameter<int>("sensor_max_age_ms", 50)),
    flex_zero_raw_(static_cast<float>(declare_parameter<double>("flex_zero_raw", 0.0))),
    csv_path_(declare_parameter<std::string>("csv_path", ""))
  {
    sgc_sup_init(&sup_);
    sgc_ctrl_state_init(&ctrl_);
    sched_ = {};
    ukf_mirror_ = {};

    actuator_publisher_ = create_publisher<glove_interfaces::msg::ActuatorCommand>(
      "/glove_control/actuator_command", 10);
    status_publisher_ = create_publisher<glove_interfaces::msg::ControllerStatus>(
      "/glove_control/controller_status", 10);
    sensor_subscription_ = create_subscription<glove_interfaces::msg::SensorFrame>(
      "/pico_bridge/sensor_frame", 10,
      [this](glove_interfaces::msg::SensorFrame::ConstSharedPtr msg) {
        on_sensor(*msg);
      });
    estimator_subscription_ = create_subscription<glove_interfaces::msg::EstimatorState>(
      "/ukf/estimator_state", 10,
      [this](glove_interfaces::msg::EstimatorState::ConstSharedPtr msg) {
        on_estimator(*msg);
      });
    command_subscription_ = create_subscription<glove_interfaces::msg::ControlCommand>(
      "/glove_control/control_command", 10,
      [this](glove_interfaces::msg::ControlCommand::ConstSharedPtr msg) {
        on_command(*msg);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(base_tick_ms_), [this]() {on_tick();});
  }

  void on_sensor(const glove_interfaces::msg::SensorFrame & msg)
  {
    latest_sensor_ = msg;
    have_sensor_ = true;
    sensor_received_ms_ =
      static_cast<uint32_t>(steady_clock_.now().nanoseconds() / 1000000);
  }

  void on_estimator(const glove_interfaces::msg::EstimatorState & msg)
  {
    latest_estimator_ = msg;
    have_estimator_ = true;
    estimator_received_ms_ =
      static_cast<uint32_t>(steady_clock_.now().nanoseconds() / 1000000);
    copy_estimator_telemetry(msg);
  }

  void on_command(const glove_interfaces::msg::ControlCommand & msg)
  {
    // Emergency shutdown is terminal until this node is restarted.
    if (abort_active_ || overpressure_latched_) {return;}
    switch (msg.kind) {
      case 0u:
        pending_.set_q = true;
        pending_.q_value = msg.q_ref_pct;
        break;
      case 1u:
        pending_.set_mode = true;
        pending_.mode_value = static_cast<SgcControllerMode>(msg.mode);
        break;
      case 2u:
        pending_.set_logging = true;
        pending_.logging_value = msg.enable;
        break;
      case 3u:
        pending_.set_reset = true;
        break;
      case 4u:
        pending_.cmd_abort = true;
        break;
      case glove_interfaces::msg::ControlCommand::SET_P:
        if (sup_.controller_mode == SGC_CTRL_PRESSURE &&
          std::isfinite(msg.pressure_ref_kpa) &&
          msg.pressure_ref_kpa >= SGC_PRESSURE_REF_MIN_KPA &&
          msg.pressure_ref_kpa <= SGC_PRESSURE_REF_MAX_KPA)
        {
          ctrl_.pressure_ref_kpa = msg.pressure_ref_kpa;
        }
        break;
      default:
        break;
    }
  }

  void on_tick()
  {
    uint32_t now_ms =
      static_cast<uint32_t>(steady_clock_.now().nanoseconds() / 1000000);
    SgcTickInput in = pending_;
    pending_ = {};
    in.now_ms = now_ms;

    sensor_stale_ = !have_sensor_ ||
      static_cast<uint32_t>(now_ms - sensor_received_ms_) >
      static_cast<uint32_t>(sensor_max_age_ms_);
    if (!sensor_stale_) {
      held_sensor_ = latest_sensor_;
      have_held_sensor_ = true;
    }
    if (have_held_sensor_) {
      in.pressure_kpa = held_sensor_.pressure_kpa;
      in.pressure_filtered = held_sensor_.pressure_filtered_kpa;
      in.flex_filtered_raw = held_sensor_.flex_filtered_raw;
      in.latest_position_pct = sgc_clampf(
        sgc_flex_raw_to_position_unclipped(
          in.flex_filtered_raw, flex_zero_raw_), 0.0f, 100.0f);
    }

    estimator_stale_ = !have_estimator_ ||
      static_cast<uint32_t>(now_ms - estimator_received_ms_) >
      static_cast<uint32_t>(estimator_max_age_ms_);
    if (have_estimator_) {
      ukf_mirror_.telem.valid =
        !estimator_stale_ && latest_estimator_.valid;
    }

    if (abort_active_ || overpressure_latched_) {
      last_actions_ = {};
      publish_status(last_actions_);
      return;
    }

    const bool had_pulse = sup_.pulse_active;
    SgcActions out;
    int rc = sgc_sched_tick(
      &sched_, &sup_, &ukf_mirror_, &ctrl_, &in, &out);
    if (rc == 0 && in.set_mode && in.mode_value == SGC_CTRL_NONE) {
      // ALL_OFF also cancels the Pico pulse; HOLD alone leaves outputs energized.
      sup_.pulse_active = false;
      sup_.last_pulse_ms = 0u;
      sup_.pneumatic_state = SGC_STATE_OFF;
      out.cancel_pulse = out.cancel_pulse || had_pulse;
      out.set_valves = false;
      out.arm_pulse = false;
      out.outputs_off = true;
    }
    last_input_ = in;
    last_actions_ = out;
    ++tick_calls_;

    const bool entering_active_mode =
      rc == 0 && in.set_mode && sup_.control_enabled &&
      (in.mode_value == SGC_CTRL_PI ||
      in.mode_value == SGC_CTRL_ADRC ||
      in.mode_value == SGC_CTRL_PI_UKF ||
      in.mode_value == SGC_CTRL_ADRC_UKF ||
      in.mode_value == SGC_CTRL_PRESSURE);

    if (entering_active_mode) {
      // Golden active-mode entry:
      // pump ON first, then force the pneumatic path to HOLD.
      publish_actuator(3u, 0u);
      publish_actuator(0u, 0u);
      sup_.pneumatic_state = SGC_STATE_HOLD;

      if (out.set_valves && out.valve_cmd == SGC_STATE_HOLD) {
        out.set_valves = false;
      }
    }
    publish_actions(out);
    if (rc == 1) {
      overpressure_latched_ = true;
    }
    if (rc == 2) {
      abort_active_ = true;
    }
    publish_status(out);
  }

  const SgcTickInput & last_input() const {return last_input_;}
  const SgcActions & last_actions() const {return last_actions_;}
  uint64_t tick_calls() const {return tick_calls_;}
  const SgcSchedState & sched_state() const {return sched_;}
  const SgcSupState & sup_state() const {return sup_;}
  const SgcCtrlState & ctrl_state() const {return ctrl_;}
  const SgcUkfState & ukf_state() const {return ukf_mirror_;}
#ifdef GLOVE_CONTROL_TESTING
  void test_publish_actions(const SgcActions & out) {publish_actions(out);}
#endif

private:
  void copy_estimator_telemetry(const glove_interfaces::msg::EstimatorState & msg)
  {
    ukf_mirror_.telem.s_hat = msg.s_hat;
    ukf_mirror_.telem.v_hat = msg.v_hat;
    ukf_mirror_.telem.eta_hat = msg.eta_hat;
    ukf_mirror_.telem.b_f_hat = msg.b_f_hat;
    ukf_mirror_.telem.theta_deg = msg.theta_deg;
    ukf_mirror_.telem.innovation_flex = msg.innovation_flex;
    ukf_mirror_.telem.innovation_theta = msg.innovation_theta;
    ukf_mirror_.telem.nis = msg.nis;
    ukf_mirror_.telem.sigma_s = msg.sigma_s;
    ukf_mirror_.telem.sigma_v = msg.sigma_v;
    ukf_mirror_.telem.sigma_eta = msg.sigma_eta;
    ukf_mirror_.telem.sigma_b_f = msg.sigma_b_f;
    ukf_mirror_.telem.rho_eta_b_f = msg.rho_eta_b_f;
    ukf_mirror_.telem.valid = msg.valid;
    ukf_mirror_.telem.reference_valid = msg.reference_valid;
    ukf_mirror_.telem.theta_valid = msg.theta_valid;
    ukf_mirror_.reference_valid = msg.reference_valid;
    ukf_mirror_.x[0] = msg.s_hat;
    ukf_mirror_.x[1] = msg.v_hat;
    ukf_mirror_.x[2] = msg.eta_hat;
    ukf_mirror_.x[3] = msg.b_f_hat;
  }

  static uint8_t valve_command(SgcPneumaticState state)
  {
    switch (state) {
      case SGC_STATE_FILL:
        return 1u;
      case SGC_STATE_VENT:
        return 2u;
      case SGC_STATE_HOLD:
      default:
        return 0u;
    }
  }

  void publish_actuator(uint8_t command, uint32_t pulse_ms)
  {
    glove_interfaces::msg::ActuatorCommand msg;
    msg.command = command;
    msg.pulse_ms = pulse_ms;
    msg.seq = ++command_seq_;
    actuator_publisher_->publish(msg);
  }

  void publish_actions(const SgcActions & out)
  {
    if (out.set_valves) {
      publish_actuator(valve_command(out.valve_cmd), 0u);
    }
    if (out.arm_pulse) {
      publish_actuator(valve_command(out.valve_cmd), out.arm_pulse_ms);
    }
    if (out.pump_off) {
      publish_actuator(4u, 0u);
    }
    if (out.outputs_off) {
      publish_actuator(5u, 0u);
    }
  }

  void publish_status(const SgcActions & out)
  {
    glove_interfaces::msg::ControllerStatus status;
    status.controller_mode = static_cast<uint8_t>(sup_.controller_mode);
    status.control_enabled = sup_.control_enabled;
    status.position_ref_pct = sup_.position_ref_pct;
    status.pressure_ref_kpa = ctrl_.pressure_ref_kpa;
    status.active_kp = ctrl_.active_kp_position;
    status.active_ki = ctrl_.active_ki_position;
    status.active_b0 = ctrl_.active_b0;
    status.eso_z1_pct = ctrl_.eso_z1_pct;
    status.eso_z2_pct_s = ctrl_.eso_z2_pct_s;
    status.ff_gamma = ctrl_.ff_gamma_state;
    status.pneumatic_state = static_cast<int8_t>(sup_.pneumatic_state);
    status.last_pulse_ms = sup_.last_pulse_ms;
    status.rate_limited =
      ctrl_.last_position_pi.rate_limited || ctrl_.last_ladrc.rate_limited;
    status.saturated =
      ctrl_.last_position_pi.saturated || ctrl_.last_ladrc.saturated;
    status.ran_sample = out.ran_sample;
    status.ran_ukf = out.ran_ukf;
    status.ran_eso = out.ran_eso;
    status.ran_outer = out.ran_outer;
    status.ran_pressure = out.ran_pressure;
    status.ran_log = out.ran_log;
    if (have_estimator_) {
      status.estimator_source_seq = latest_estimator_.source_seq;
      status.estimator_pipeline_latency_us = static_cast<uint32_t>(
        (get_clock()->now().nanoseconds() -
        rclcpp::Time(latest_estimator_.estimator_stamp).nanoseconds()) / 1000);
      status.estimator_seq_repeat =
        have_status_source_seq_ &&
        latest_estimator_.source_seq == last_status_source_seq_;
      last_status_source_seq_ = latest_estimator_.source_seq;
      have_status_source_seq_ = true;
    }
    if (have_sensor_ && have_estimator_) {
      status.estimator_acquisition_age_us =
        latest_sensor_.pico_timestamp_us -
        latest_estimator_.source_timestamp_us;
    }
    status.abort_active = abort_active_;
    status.overpressure_latched = overpressure_latched_;
    status.link_up = have_sensor_ && !sensor_stale_;
    status.estimator_stale = estimator_stale_;
    status.sensor_stale = sensor_stale_;
    status_publisher_->publish(status);
  }

  rclcpp::Clock steady_clock_;
  int base_tick_ms_;
  int estimator_max_age_ms_;
  int sensor_max_age_ms_;
  float flex_zero_raw_;
  std::string csv_path_;
  SgcSchedState sched_{};
  SgcSupState sup_{};
  SgcCtrlState ctrl_{};
  SgcUkfState ukf_mirror_{};
  SgcTickInput pending_{};
  SgcTickInput last_input_{};
  SgcActions last_actions_{};
  glove_interfaces::msg::SensorFrame latest_sensor_;
  glove_interfaces::msg::SensorFrame held_sensor_;
  glove_interfaces::msg::EstimatorState latest_estimator_;
  bool have_sensor_{false};
  bool have_held_sensor_{false};
  bool have_estimator_{false};
  uint32_t sensor_received_ms_{0u};
  uint32_t estimator_received_ms_{0u};
  bool sensor_stale_{true};
  bool estimator_stale_{true};
  bool abort_active_{false};
  bool overpressure_latched_{false};
  bool have_status_source_seq_{false};
  uint32_t last_status_source_seq_{0u};
  uint32_t command_seq_{0u};
  uint64_t tick_calls_{0u};
  rclcpp::Subscription<glove_interfaces::msg::SensorFrame>::SharedPtr sensor_subscription_;
  rclcpp::Subscription<glove_interfaces::msg::EstimatorState>::SharedPtr estimator_subscription_;
  rclcpp::Subscription<glove_interfaces::msg::ControlCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<glove_interfaces::msg::ActuatorCommand>::SharedPtr actuator_publisher_;
  rclcpp::Publisher<glove_interfaces::msg::ControllerStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GloveControlNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
