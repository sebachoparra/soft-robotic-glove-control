// Copyright 2026 Sebastian Parra

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "glove_interfaces/msg/estimator_input_frame.hpp"
#include "glove_interfaces/msg/estimator_state.hpp"
#include "glove_interfaces/srv/calibrate_ukf_reference.hpp"
#include "rclcpp/rclcpp.hpp"
#include "soft_glove_core/soft_glove_core_ukf_compose.h"

class UkfNode : public rclcpp::Node
{
public:
  UkfNode()
  : Node("ukf_node"),
    ref_samples_(declare_parameter<int>("ref_samples", 40))
  {
    sgc_ukf_compose_init(&compose_);
    publisher_ = create_publisher<glove_interfaces::msg::EstimatorState>(
      "/ukf/estimator_state", 10);
    subscription_ = create_subscription<glove_interfaces::msg::EstimatorInputFrame>(
      "/pico_bridge/estimator_input", 10,
      [this](glove_interfaces::msg::EstimatorInputFrame::ConstSharedPtr msg) {
        on_frame(*msg);
      });
    service_ = create_service<glove_interfaces::srv::CalibrateUkfReference>(
      "~/calibrate_ukf_reference",
      [this](
        const std::shared_ptr<glove_interfaces::srv::CalibrateUkfReference::Request> request,
        std::shared_ptr<glove_interfaces::srv::CalibrateUkfReference::Response> response) {
        (void)request;
        on_calibrate(*response);
      });
  }

  const SgcUkfCompose & compose_state() const {return compose_;}
  bool seq_discontinuity() const {return seq_discontinuity_;}
  uint64_t seq_gap_count() const {return seq_gap_count_;}

  void on_frame(const glove_interfaces::msg::EstimatorInputFrame & msg)
  {
    SgcQuat q1 = {
      msg.bno1_quat[0], msg.bno1_quat[1], msg.bno1_quat[2], msg.bno1_quat[3]};
    SgcQuat q2 = {
      msg.bno2_quat[0], msg.bno2_quat[1], msg.bno2_quat[2], msg.bno2_quat[3]};

    seq_discontinuity_ = have_seq_ && msg.seq != last_seq_ + 1u;
    if (seq_discontinuity_) {
      ++seq_gap_count_;
      RCLCPP_WARN(get_logger(), "Estimator input sequence gap: expected=%u received=%u",
        last_seq_ + 1u, msg.seq);
    }
    have_seq_ = true;
    last_seq_ = msg.seq;

    if (ref_samples_ > 0) {
      recent_.push_back(SgcRefSample{msg.bno1_ok, q1, msg.bno2_ok, q2});
      while (recent_.size() > static_cast<size_t>(ref_samples_)) {
        recent_.pop_front();
      }
    }

    // Keep every reference sample above; only filter updates use the 50 ms grid.
    // Unsigned subtraction preserves Pico uint32_t timestamp wraparound.
    if (!have_update_tick_) {
      update_tick_us_ = msg.pico_timestamp_us;
      have_update_tick_ = true;
    } else {
      const uint32_t elapsed_us = msg.pico_timestamp_us - update_tick_us_;
      if (elapsed_us < kUpdatePeriodUs) {
        return;
      }
      // Keep the grid phase (20 ms input gives alternating 60/40 ms arrivals).
      // Skip missed ticks after a gap rather than replaying the same sample.
      update_tick_us_ += (elapsed_us / kUpdatePeriodUs) * kUpdatePeriodUs;
    }
    sgc_ukf_compose_step(
      &compose_, msg.flex_filtered_raw,
      msg.bno1_ok, q1, msg.bno2_ok, q2, update_tick_us_);

    glove_interfaces::msg::EstimatorState out;
    out.header = msg.header;
    out.s_hat = compose_.ukf.telem.s_hat;
    out.v_hat = compose_.ukf.telem.v_hat;
    out.eta_hat = compose_.ukf.telem.eta_hat;
    out.b_f_hat = compose_.ukf.telem.b_f_hat;
    out.theta_deg = compose_.ukf.telem.theta_deg;
    out.innovation_flex = compose_.ukf.telem.innovation_flex;
    out.innovation_theta = compose_.ukf.telem.innovation_theta;
    out.nis = compose_.ukf.telem.nis;
    out.sigma_s = compose_.ukf.telem.sigma_s;
    out.sigma_v = compose_.ukf.telem.sigma_v;
    out.sigma_eta = compose_.ukf.telem.sigma_eta;
    out.sigma_b_f = compose_.ukf.telem.sigma_b_f;
    out.rho_eta_b_f = compose_.ukf.telem.rho_eta_b_f;
    out.dt_used = compose_.dt_used;
    out.valid = compose_.ukf.telem.valid;
    out.reference_valid = compose_.ukf.telem.reference_valid;
    out.theta_valid = compose_.ukf.telem.theta_valid;
    out.bno1_ok = compose_.bno1_ok;
    out.bno2_ok = compose_.bno2_ok;
    out.source_seq = msg.seq;
    out.source_timestamp_us = msg.pico_timestamp_us;
    out.estimator_stamp = now();
    publisher_->publish(out);
  }

  void on_calibrate(glove_interfaces::srv::CalibrateUkfReference::Response & response)
  {
    response.success = false;
    response.good_samples = 0u;
    if (ref_samples_ <= 0 || recent_.size() < static_cast<size_t>(ref_samples_)) {
      response.message = "Insufficient estimator input frames";
      return;
    }
    std::vector<SgcRefSample> samples(recent_.begin(), recent_.end());
    for (const auto & sample : samples) {
      if (sample.ok1 && sample.ok2) {
        ++response.good_samples;
      }
    }
    response.success = sgc_ukf_accumulate_reference(
      &compose_, samples.data(), static_cast<uint32_t>(samples.size()));
    if (response.success) {
      // The golden reset makes the first subsequent update use nominal dt.
      compose_.last_update_us = 0u;
      have_update_tick_ = false;
    }
    response.q_rel0 = {
      compose_.ukf.qrel0.w, compose_.ukf.qrel0.x,
      compose_.ukf.qrel0.y, compose_.ukf.qrel0.z};
    response.message = response.success ? "Reference calibrated" : "Insufficient valid samples";
  }

private:
  static constexpr uint32_t kUpdatePeriodUs = 50000u;
  bool have_update_tick_{false};
  uint32_t update_tick_us_{0u};
  SgcUkfCompose compose_{};
  int ref_samples_;
  std::deque<SgcRefSample> recent_;
  bool have_seq_{false};
  uint32_t last_seq_{0u};
  bool seq_discontinuity_{false};
  uint64_t seq_gap_count_{0u};
  rclcpp::Subscription<glove_interfaces::msg::EstimatorInputFrame>::SharedPtr subscription_;
  rclcpp::Publisher<glove_interfaces::msg::EstimatorState>::SharedPtr publisher_;
  rclcpp::Service<glove_interfaces::srv::CalibrateUkfReference>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UkfNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
