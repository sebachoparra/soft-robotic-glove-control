// Copyright 2026 Sebastian Parra
#define main ukf_node_program_main
#include "../src/ukf_node.cpp"  // NOLINT(build/include)
#undef main

#include <rcl/time.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

static bool same_float(float a, float b)
{
  return std::memcmp(&a, &b, sizeof(float)) == 0;
}
static int compare_message(
  const glove_interfaces::msg::EstimatorState & out,
  const SgcUkfCompose & expected,
  const glove_interfaces::msg::EstimatorInputFrame & in)
{
  int bad = 0;
#define CHECK_FLOAT(field, value) if (!same_float(out.field, value)) {++bad;}
  CHECK_FLOAT(s_hat, expected.ukf.telem.s_hat)
  CHECK_FLOAT(v_hat, expected.ukf.telem.v_hat)
  CHECK_FLOAT(eta_hat, expected.ukf.telem.eta_hat)
  CHECK_FLOAT(b_f_hat, expected.ukf.telem.b_f_hat)
  CHECK_FLOAT(theta_deg, expected.ukf.telem.theta_deg)
  CHECK_FLOAT(innovation_flex, expected.ukf.telem.innovation_flex)
  CHECK_FLOAT(innovation_theta, expected.ukf.telem.innovation_theta)
  CHECK_FLOAT(nis, expected.ukf.telem.nis)
  CHECK_FLOAT(sigma_s, expected.ukf.telem.sigma_s)
  CHECK_FLOAT(sigma_v, expected.ukf.telem.sigma_v)
  CHECK_FLOAT(sigma_eta, expected.ukf.telem.sigma_eta)
  CHECK_FLOAT(sigma_b_f, expected.ukf.telem.sigma_b_f)
  CHECK_FLOAT(rho_eta_b_f, expected.ukf.telem.rho_eta_b_f)
  CHECK_FLOAT(dt_used, expected.dt_used)
#undef CHECK_FLOAT
  bad += out.valid != expected.ukf.telem.valid;
  bad += out.reference_valid != expected.ukf.telem.reference_valid;
  bad += out.theta_valid != expected.ukf.telem.theta_valid;
  bad += out.bno1_ok != expected.bno1_ok;
  bad += out.bno2_ok != expected.bno2_ok;
  bad += out.source_seq != in.seq;
  bad += out.source_timestamp_us != in.pico_timestamp_us;
  bad += out.header.frame_id != in.header.frame_id;
  return bad;
}
// Drive 50 Hz Pico time deterministically; DDS waits do not drive the gate.
static int test_50hz(uint32_t start_us)
{
  auto node = std::make_shared<UkfNode>();
  auto observer = std::make_shared<rclcpp::Node>("ukf_timing_test");
  std::vector<glove_interfaces::msg::EstimatorState> received;
  auto subscription = observer->create_subscription<glove_interfaces::msg::EstimatorState>(
    "/ukf/estimator_state", 100,
    [&received](glove_interfaces::msg::EstimatorState::ConstSharedPtr msg) {
      received.push_back(*msg);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  SgcUkfCompose expected;
  sgc_ukf_compose_init(&expected);
  std::deque<SgcRefSample> recent;
  int bad = 0;
  size_t outputs = 0;
  uint32_t seq = 0u;
  for (uint32_t phase = 0u; phase < 2u; ++phase) {
    const uint32_t count = phase == 0u ? 40u : 51u;
    const size_t before = outputs;
    for (uint32_t i = 0u; i < count; ++i) {
      glove_interfaces::msg::EstimatorInputFrame in;
      in.header.frame_id = "glove";
      // A sequence gap on a skipped frame must still be detected.
      seq += i == 1u ? 2u : 1u;
      in.seq = seq;
      in.pico_timestamp_us = start_us + i * 20000u;
      in.flex_filtered_raw = 20.0f + static_cast<float>(i % 7u);
      in.bno1_ok = !(phase == 1u && i == 10u);
      in.bno2_ok = in.bno1_ok;
      in.bno1_quat = {1.0f, 0.0f, 0.0f, 0.0f};
      in.bno2_quat = {1.0f, 0.005f * static_cast<float>(i), 0.0f, 0.0f};
      SgcQuat q1 = {1.0f, 0.0f, 0.0f, 0.0f};
      SgcQuat q2 = {in.bno2_quat[0], in.bno2_quat[1], 0.0f, 0.0f};
      recent.push_back(SgcRefSample{in.bno1_ok, q1, in.bno2_ok, q2});
      if (recent.size() > 40u) {
        recent.pop_front();
      }
      // Independently enumerate arrivals at ceil(k * 50 / 20): 0,3,5,8,10,...
      const bool update = i == 0u || i % 5u == 0u || i % 5u == 3u;
      if (update) {
        const uint32_t tick = start_us + static_cast<uint32_t>(outputs - before) * 50000u;
        sgc_ukf_compose_step(&expected, in.flex_filtered_raw,
          in.bno1_ok, q1, in.bno2_ok, q2, tick);
        ++outputs;
      }
      // ROS clock jumps must not affect the Pico-based cadence.
      bad += rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()) != RCL_RET_OK;
      bad += rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
        i % 2u == 0u ? 1000ll : 500000000000ll) != RCL_RET_OK;
      node->on_frame(in);
      bad += node->seq_discontinuity() != (i == 1u);
      bad += node->seq_gap_count() != phase + (i >= 1u ? 1u : 0u);
      bad += std::memcmp(&node->compose_state(), &expected, sizeof(expected)) != 0;
      for (int n = 0; n < (update ? 100 : 3); ++n) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (update && received.size() >= outputs) {
          break;
        }
      }
      bad += received.size() != outputs;
      if (update && received.size() == outputs) {
        bad += compare_message(received.back(), expected, in);
        bad += !same_float(received.back().dt_used, 0.05f);
        bad += received.back().reference_valid != (phase == 1u);
        bad += received.back().valid != (phase == 1u && in.bno1_ok);
        bad += received.back().theta_valid != (phase == 1u && in.bno1_ok);
      }
      if (phase == 0u && i == 38u) {
        glove_interfaces::srv::CalibrateUkfReference::Response early;
        node->on_calibrate(early);
        bad += early.success;
      }
    }
    // 21 updates including t=0 over the closed interval [0,1 s].
    bad += outputs - before != (phase == 0u ? 16u : 21u);
    glove_interfaces::srv::CalibrateUkfReference::Response response;
    node->on_calibrate(response);
    std::vector<SgcRefSample> samples(recent.begin(), recent.end());
    bad += !sgc_ukf_accumulate_reference(&expected, samples.data(), 40u);
    expected.last_update_us = 0u;
    bad += !response.success;
    bad += response.good_samples != 40u;
    // Varying quaternions prove calibration uses every one of the latest 40 frames.
    bad += std::memcmp(&node->compose_state(), &expected, sizeof(expected)) != 0;
    start_us += count * 20000u;
  }
  (void)subscription;
  return bad;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UkfNode>();
  auto observer = std::make_shared<rclcpp::Node>("ukf_node_wrapper_test");
  std::vector<glove_interfaces::msg::EstimatorState> received;
  auto subscription = observer->create_subscription<glove_interfaces::msg::EstimatorState>(
    "/ukf/estimator_state", 10,
    [&received](glove_interfaces::msg::EstimatorState::ConstSharedPtr msg) {
      received.push_back(*msg);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  SgcUkfCompose expected;
  sgc_ukf_compose_init(&expected);
  int bad = 0;
  std::vector<SgcRefSample> reference_samples;
  for (uint32_t i = 0; i < 3u; ++i) {
    glove_interfaces::msg::EstimatorInputFrame in;
    in.header.frame_id = "glove";
    in.seq = i == 2u ? 4u : i + 1u;
    in.pico_timestamp_us = 100000u + i * 50000u;
    in.flex_filtered_raw = 20.0f + static_cast<float>(i);
    in.bno1_ok = i == 1u;
    in.bno2_ok = i == 1u;
    in.bno1_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    in.bno2_quat = {0.9238795f, 0.3826834f, 0.0f, 0.0f};
    if (i == 1u) {
      bad += rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()) != RCL_RET_OK;
      bad += rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
        500000000000ll) != RCL_RET_OK;
    }
    if (i == 2u) {
      bad += rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
        1000ll) != RCL_RET_OK;
    }
    SgcQuat q1 = {in.bno1_quat[0], in.bno1_quat[1], in.bno1_quat[2], in.bno1_quat[3]};
    SgcQuat q2 = {in.bno2_quat[0], in.bno2_quat[1], in.bno2_quat[2], in.bno2_quat[3]};
    reference_samples.push_back(SgcRefSample{in.bno1_ok, q1, in.bno2_ok, q2});
    sgc_ukf_compose_step(&expected, in.flex_filtered_raw, in.bno1_ok, q1,
      in.bno2_ok, q2, in.pico_timestamp_us);
    node->on_frame(in);
    bad += std::memcmp(&node->compose_state(), &expected, sizeof(expected)) != 0;
    bad += node->seq_discontinuity() != (i == 2u);
    bad += node->seq_gap_count() != (i == 2u ? 1u : 0u);
    for (int n = 0; n < 100 && received.size() <= i; ++n) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (received.size() <= i) {
      ++bad;
    } else {
      bad += compare_message(received[i], expected, in);
      if (i == 1u) {
        bad += received[i].estimator_stamp.sec != 500;
      }
      if (i == 2u) {
        bad += received[i].estimator_stamp.sec != 0;
      }
      bad += !same_float(received[i].dt_used, 0.05f);
    }
  }
  glove_interfaces::srv::CalibrateUkfReference::Response early;
  node->on_calibrate(early);
  bad += early.success;
  for (uint32_t i = 3u; i < 40u; ++i) {
    glove_interfaces::msg::EstimatorInputFrame in;
    in.seq = i + 2u;
    in.pico_timestamp_us = 100000u + i * 50000u;
    in.flex_filtered_raw = 20.0f + static_cast<float>(i);
    in.bno1_ok = true;
    in.bno2_ok = true;
    in.bno1_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    in.bno2_quat = {0.9238795f, 0.3826834f, 0.0f, 0.0f};
    SgcQuat q1 = {in.bno1_quat[0], in.bno1_quat[1], in.bno1_quat[2], in.bno1_quat[3]};
    SgcQuat q2 = {in.bno2_quat[0], in.bno2_quat[1], in.bno2_quat[2], in.bno2_quat[3]};
    reference_samples.push_back(SgcRefSample{true, q1, true, q2});
    sgc_ukf_compose_step(&expected, in.flex_filtered_raw, true, q1, true, q2,
      in.pico_timestamp_us);
    node->on_frame(in);
    bad += std::memcmp(&node->compose_state(), &expected, sizeof(expected)) != 0;
  }
  bad += node->seq_gap_count() != 1u;
  glove_interfaces::srv::CalibrateUkfReference::Response calibrated;
  node->on_calibrate(calibrated);
  bool core_success = sgc_ukf_accumulate_reference(
    &expected, reference_samples.data(), static_cast<uint32_t>(reference_samples.size()));
  expected.last_update_us = 0u;
  bad += calibrated.success != core_success;
  bad += calibrated.good_samples != 38u;
  bad += std::memcmp(&node->compose_state(), &expected, sizeof(expected)) != 0;
  bad += !same_float(calibrated.q_rel0[0], expected.ukf.qrel0.w);
  bad += !same_float(calibrated.q_rel0[1], expected.ukf.qrel0.x);
  bad += !same_float(calibrated.q_rel0[2], expected.ukf.qrel0.y);
  bad += !same_float(calibrated.q_rel0[3], expected.ukf.qrel0.z);
  (void)subscription;
  executor.remove_node(observer);
  executor.remove_node(node);
  node.reset();
  bad += test_50hz(100000u);
  bad += test_50hz(0xfff00000u);
  bad += test_50hz(0u - 1050000u);  // A logical tick lands exactly on zero.
  rclcpp::shutdown();
  return bad ? 1 : 0;
}
