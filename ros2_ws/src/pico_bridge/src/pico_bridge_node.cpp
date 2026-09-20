// Copyright 2026 Sebastian Parra

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "glove_interfaces/msg/actuator_command.hpp"
#include "glove_interfaces/msg/estimator_input_frame.hpp"
#include "glove_interfaces/msg/pico_status.hpp"
#include "glove_interfaces/msg/sensor_frame.hpp"
#include "glove_interfaces/srv/calibrate_sensors.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr size_t kMaxLine = 512u;

uint16_t crc16(const std::string & bytes)
{
  uint16_t crc = 0xffffu;
  for (unsigned char byte : bytes) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

bool parse_uint(const std::string & field, uint32_t * value)
{
  if (field.empty()) {return false;}
  for (char ch : field) {
    if (ch < '0' || ch > '9') {
      return false;
    }
  }
  errno = 0;
  char * end = nullptr;
  uint64_t parsed = std::strtoull(field.c_str(), &end, 10);
  if (errno || *end || parsed > UINT32_MAX) {return false;}
  *value = static_cast<uint32_t>(parsed);
  return true;
}

bool parse_int8(const std::string & field, int8_t * value)
{
  if (field.empty()) {return false;}
  size_t start = field[0] == '-' ? 1u : 0u;
  if (start == field.size()) {return false;}
  for (size_t i = start; i < field.size(); ++i) {
    if (field[i] < '0' || field[i] > '9') {return false;}
  }
  errno = 0;
  char * end = nullptr;
  int64_t parsed = std::strtoll(field.c_str(), &end, 10);
  if (errno || *end || parsed < -128 || parsed > 127) {return false;}
  *value = static_cast<int8_t>(parsed);
  return true;
}

bool parse_bool(const std::string & field, bool * value)
{
  if (field == "0") {*value = false; return true;}
  if (field == "1") {*value = true; return true;}
  return false;
}

bool parse_float(const std::string & field, float * value)
{
  size_t start = (field.size() && (field[0] == '+' || field[0] == '-')) ? 1u : 0u;
  if (field.compare(start, 2, "0x") != 0 && field.compare(start, 2, "0X") != 0) {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  float parsed = std::strtof(field.c_str(), &end);
  if (errno || *end || end == field.c_str()) {return false;}
  *value = parsed;
  return true;
}

std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> fields;
  size_t begin = 0u;
  while (begin <= line.size()) {
    size_t comma = line.find(',', begin);
    if (comma == std::string::npos) {
      fields.push_back(line.substr(begin));
      break;
    }
    fields.push_back(line.substr(begin, comma - begin));
    begin = comma + 1u;
  }
  return fields;
}

std::string with_crc(const std::string & prefix)
{
  return prefix + std::to_string(crc16(prefix)) + "\n";
}
}  // namespace

class PicoBridgeNode : public rclcpp::Node
{
public:
  explicit PicoBridgeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::function<std::chrono::steady_clock::time_point()> clock = std::chrono::steady_clock::now)
  : Node("pico_bridge_node", options),
    port_(declare_parameter<std::string>("port", "/dev/ttyACM0")),
    baud_(declare_parameter<int>("baud", 921600)),
    transport_(declare_parameter<std::string>("transport", "serial")),
    reconnect_backoff_ms_(declare_parameter<int>("reconnect_backoff_ms", 250)),
    rx_timeout_ms_(declare_parameter<int>("rx_timeout_ms", 100)),
    clock_(clock)
  {
    if (rx_timeout_ms_ <= 0) {throw std::invalid_argument("rx_timeout_ms must be positive");}
    sensor_pub_ = create_publisher<glove_interfaces::msg::SensorFrame>(
      "/pico_bridge/sensor_frame", 10);
    estimator_pub_ = create_publisher<glove_interfaces::msg::EstimatorInputFrame>(
      "/pico_bridge/estimator_input", 10);
    status_pub_ = create_publisher<glove_interfaces::msg::PicoStatus>(
      "/pico_bridge/pico_status", 10);
    actuator_sub_ = create_subscription<glove_interfaces::msg::ActuatorCommand>(
      "/glove_control/actuator_command", 10,
      [this](glove_interfaces::msg::ActuatorCommand::ConstSharedPtr msg) {
        on_actuator(*msg);
      });
    calibrate_srv_ = create_service<glove_interfaces::srv::CalibrateSensors>(
      "/pico_bridge/calibrate_sensors",
      [this](std::shared_ptr<rmw_request_id_t> header,
      std::shared_ptr<glove_interfaces::srv::CalibrateSensors::Request>) {
        if (calibration_request_ || !refresh_link()) {
          glove_interfaces::srv::CalibrateSensors::Response response;
          response.success = false;
          response.message = "Calibration busy or Pico disconnected";
          calibrate_srv_->send_response(*header, response);
          return;
        }
        calibration_request_ = header;
        calibration_deadline_ = clock_() + std::chrono::seconds(2);
        if (send_line(with_crc("C," + std::to_string(++calibration_seq_) + ","))) {
          ++tx_frames_;
        }
      });
    if (transport_ == "loopback") {
      loopback_connected_ = true;
    } else if (transport_ == "serial") {
      try_connect();
    } else {
      throw std::invalid_argument("transport must be serial or loopback");
    }
    io_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {poll_transport();});
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_status();});
  }

  ~PicoBridgeNode() override
  {
    // Do not synthesize ALL_OFF: it acknowledges/rearms the Pico safety latch.
    // Stopping TX leaves the local Pico timeout responsible for sustained VENT.
    if (fd_ >= 0) {::close(fd_);}
  }

  void inject_loopback_line(const std::string & line)
  {
    if (transport_ == "loopback" && loopback_connected_) {receive_line(line);}
  }

  void simulate_drop()
  {
    if (transport_ == "loopback") {loopback_connected_ = false; set_link(false);}
  }

  void simulate_reconnect()
  {
    if (transport_ == "loopback" && !loopback_connected_) {
      ++reconnect_count_;
      loopback_connected_ = true;
    }
  }

  void on_actuator(const glove_interfaces::msg::ActuatorCommand & msg)
  {
    if (!refresh_link()) {return;}
    const bool acknowledge = msg.command == 5u && msg.pulse_ms == 0u;
    if (rearm_required_ && !acknowledge) {return;}
    if (!send_line(encode_actuator(msg))) {return;}
    // The Pico still validates sequence/deadline/pressure before accepting rearm.
    if (acknowledge) {rearm_required_ = false;}
    ++tx_frames_;
    publish_status();
  }

  static std::string encode_actuator(const glove_interfaces::msg::ActuatorCommand & msg)
  {
    return with_crc("A," + std::to_string(msg.seq) + "," +
      std::to_string(msg.command) + "," + std::to_string(msg.pulse_ms) + "," +
      std::to_string(msg.deadline_us) + ",");
  }

  const std::vector<std::string> & captured_lines() const {return captured_lines_;}
  uint32_t crc_errors() const {return crc_errors_;}
  uint32_t framing_errors() const {return framing_errors_;}
  uint32_t truncated_errors() const {return truncated_errors_;}
  uint32_t short_fields() const {return short_fields_;}
  uint32_t long_fields() const {return long_fields_;}
  uint32_t numeric_errors() const {return numeric_errors_;}
  uint32_t seq_repeats() const {return seq_repeats_;}
  uint32_t seq_gaps() const {return seq_gaps_;}
  uint32_t rx_frames() const {return rx_frames_;}
  uint32_t tx_frames() const {return tx_frames_;}
  uint32_t reconnect_count() const {return reconnect_count_;}
  bool link_up() const {return link_up_;}
#ifdef PICO_BRIDGE_TESTING
  void test_poll_transport() {poll_transport();}
  void test_disconnect_serial() {disconnect_serial();}
#endif

private:
  void finish_calibration(bool success, float pressure, float flex, const std::string & message)
  {
    if (!calibration_request_) {return;}
    glove_interfaces::srv::CalibrateSensors::Response response;
    response.success = success;
    response.pressure_zero_vout = pressure;
    response.flex_zero_raw = flex;
    response.message = message;
    calibrate_srv_->send_response(*calibration_request_, response);
    calibration_request_.reset();
  }

  void set_link(bool up)
  {
    if (!up) {rearm_required_ = true;}
    if (link_up_ == up) {return;}
    link_up_ = up;
    if (!up) {finish_calibration(false, 0.0f, 0.0f, "Pico disconnected during calibration");}
    publish_status();
  }

  bool refresh_link()
  {
    // Check at TX as well as the timer: queued ROS commands cannot bypass stale RX.
    if (link_up_ && clock_() - last_rx_ >= std::chrono::milliseconds(rx_timeout_ms_)) {
      set_link(false);
    }
    return link_up_;
  }

  void valid_rx()
  {
    // Detect a gap even if RX runs before the timer after an executor delay.
    refresh_link();
    last_rx_ = clock_();
    set_link(true);
  }

  void publish_status()
  {
    glove_interfaces::msg::PicoStatus status;
    status.header.stamp = now();
    status.link_up = link_up_;
    status.rx_frames = rx_frames_;
    status.tx_frames = tx_frames_;
    status.crc_errors = crc_errors_;
    status.framing_errors = framing_errors_;
    status.seq_gaps = seq_gaps_;
    status.last_rx_pico_us = last_rx_pico_us_;
    status.reconnect_count = reconnect_count_;
    status_pub_->publish(status);
  }

  void bad_framing() {++framing_errors_; publish_status();}
  void bad_numeric() {++numeric_errors_; bad_framing();}

  void receive_line(const std::string & wire)
  {
    refresh_link();
    if (wire.empty() || wire.back() != '\n') {
      ++truncated_errors_;
      bad_framing();
      return;
    }
    if (wire.size() > kMaxLine || wire.find('\n') != wire.size() - 1u ||
      wire.find('\r') != std::string::npos)
    {
      bad_framing();
      return;
    }
    std::string body = wire.substr(0u, wire.size() - 1u);
    const size_t last_comma = body.rfind(',');
    if (last_comma == std::string::npos) {bad_framing(); return;}
    uint32_t received_crc = 0u;
    if (!parse_uint(body.substr(last_comma + 1u), &received_crc) || received_crc > 65535u) {
      bad_framing(); return;
    }
    if (crc16(body.substr(0u, last_comma + 1u)) != received_crc) {
      ++crc_errors_;
      publish_status();
      return;
    }
    const auto fields = split_csv(body);
    if (!fields.empty() && fields[0] == "Z") {
      uint32_t seq;
      bool success;
      float pressure, flex;
      if (fields.size() != 6u || !parse_uint(fields[1], &seq) ||
        !parse_bool(fields[2], &success) || !parse_float(fields[3], &pressure) ||
        !parse_float(fields[4], &flex) || !std::isfinite(pressure) || !std::isfinite(flex))
      {
        bad_numeric(); return;
      }
      if (calibration_request_ && seq == calibration_seq_) {
        finish_calibration(success, pressure, flex, success ? "Pico zero calibration complete" :
          "Pico rejected calibration: require pump off, continuous VENT and valid pressure");
      }
      valid_rx();
      ++rx_frames_;
      return;
    }
    if (fields.empty() || (fields[0] != "S" && fields[0] != "E")) {
      bad_framing(); return;
    }
    const size_t expected = fields[0] == "S" ? 17u : 15u;
    if (fields.size() < expected) {++short_fields_; bad_framing(); return;}
    if (fields.size() > expected) {++long_fields_; bad_framing(); return;}
    if (fields[0] == "S") {decode_sensor(fields);} else {decode_estimator(fields);}
  }

  void accepted(uint32_t seq, uint32_t pico_us, bool sensor)
  {
    bool & have = sensor ? have_sensor_seq_ : have_estimator_seq_;
    uint32_t & previous = sensor ? sensor_seq_ : estimator_seq_;
    if (have) {
      if (seq == previous) {++seq_repeats_;} else if (seq != previous + 1u) {++seq_gaps_;}
    }
    previous = seq;
    have = true;
    last_rx_pico_us_ = pico_us;
    valid_rx();
    ++rx_frames_;
    publish_status();
  }

  void decode_sensor(const std::vector<std::string> & f)
  {
    glove_interfaces::msg::SensorFrame msg;
    uint32_t watchdog = 0u;
    if (!parse_uint(f[1], &msg.seq) || !parse_uint(f[2], &msg.pico_timestamp_us) ||
      !parse_float(f[3], &msg.pressure_raw) || !parse_float(f[4], &msg.pressure_vout) ||
      !parse_float(f[5], &msg.pressure_kpa) ||
      !parse_float(f[6], &msg.pressure_filtered_kpa) ||
      !parse_float(f[7], &msg.flex_raw) || !parse_float(f[8], &msg.flex_filtered_raw) ||
      !parse_int8(f[9], &msg.pneumatic_state) ||
      !parse_bool(f[10], &msg.pulse_active) ||
      !parse_uint(f[11], &msg.pulse_ms_remaining) ||
      !parse_bool(f[12], &msg.pump_on) || !parse_bool(f[13], &msg.safety_latched) ||
      !parse_uint(f[14], &msg.pulse_alarm_fail_count) ||
      !parse_uint(f[15], &watchdog) || watchdog > 255u)
    {
      bad_numeric(); return;
    }
    msg.watchdog_state = static_cast<uint8_t>(watchdog);
    msg.header.stamp = now();
    accepted(msg.seq, msg.pico_timestamp_us, true);
    sensor_pub_->publish(msg);
  }

  void decode_estimator(const std::vector<std::string> & f)
  {
    glove_interfaces::msg::EstimatorInputFrame msg;
    if (!parse_uint(f[1], &msg.seq) || !parse_uint(f[2], &msg.pico_timestamp_us) ||
      !parse_float(f[3], &msg.flex_filtered_raw) || !parse_bool(f[4], &msg.bno1_ok) ||
      !parse_float(f[5], &msg.bno1_quat[0]) || !parse_float(f[6], &msg.bno1_quat[1]) ||
      !parse_float(f[7], &msg.bno1_quat[2]) || !parse_float(f[8], &msg.bno1_quat[3]) ||
      !parse_bool(f[9], &msg.bno2_ok) || !parse_float(f[10], &msg.bno2_quat[0]) ||
      !parse_float(f[11], &msg.bno2_quat[1]) || !parse_float(f[12], &msg.bno2_quat[2]) ||
      !parse_float(f[13], &msg.bno2_quat[3]))
    {
      bad_numeric(); return;
    }
    msg.header.stamp = now();
    accepted(msg.seq, msg.pico_timestamp_us, false);
    estimator_pub_->publish(msg);
  }

  bool send_line(const std::string & line)
  {
    if (!refresh_link()) {return false;}
    if (transport_ == "loopback") {captured_lines_.push_back(line); return true;}
    if (fd_ < 0 || ::write(fd_, line.data(), line.size()) != static_cast<ssize_t>(line.size())) {
      disconnect_serial();
      return false;
    }
    return true;
  }

  void try_connect()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {return;}
    termios settings{};
    if (::tcgetattr(fd_, &settings) != 0) {disconnect_serial(); return;}
    ::cfmakeraw(&settings);
    if (baud_ != 921600 || ::cfsetispeed(&settings, B921600) != 0 ||
      ::cfsetospeed(&settings, B921600) != 0 || ::tcsetattr(fd_, TCSANOW, &settings) != 0)
    {
      disconnect_serial(); return;
    }
    if (ever_connected_) {++reconnect_count_;}
    ever_connected_ = true;
    rx_buffer_.clear();
    set_link(false);  // An open port is not evidence of valid Pico RX.
  }

  void disconnect_serial()
  {
    if (fd_ >= 0) {::close(fd_); fd_ = -1;}
    set_link(false);
    next_connect_ = clock_() +
      std::chrono::milliseconds(reconnect_backoff_ms_);
  }

  void poll_transport()
  {
    auto current = clock_();
    if (calibration_request_ && current >= calibration_deadline_) {
      finish_calibration(false, 0.0f, 0.0f, "Pico calibration response timed out");
    }
    refresh_link();
    if (transport_ == "loopback") {return;}
    if (fd_ < 0) {
      if (current >= next_connect_) {
        try_connect(); next_connect_ = current +
          std::chrono::milliseconds(reconnect_backoff_ms_);
      }
      return;
    }
    char bytes[256];
    for (;; ) {
      ssize_t n = ::read(fd_, bytes, sizeof(bytes));
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          rx_buffer_ += bytes[i];
          if (bytes[i] == '\n') {
            receive_line(rx_buffer_); rx_buffer_.clear();
          } else if (rx_buffer_.size() > kMaxLine) {rx_buffer_.clear(); bad_framing();}
        }
      } else if (n == 0) {
        disconnect_serial(); return;
      } else if (errno == EAGAIN || errno == EWOULDBLOCK) {break;} else {
        disconnect_serial(); return;
      }
    }
    // Keep an open port readable while stale so valid RX can recover the link.
    refresh_link();
  }

  std::shared_ptr<rmw_request_id_t> calibration_request_;
  std::chrono::steady_clock::time_point calibration_deadline_{};
  uint32_t calibration_seq_{0u};
  std::string port_;
  int baud_;
  std::string transport_;
  int reconnect_backoff_ms_;
  int rx_timeout_ms_;
  std::function<std::chrono::steady_clock::time_point()> clock_;
  int fd_{-1};
  bool loopback_connected_{false};
  bool rearm_required_{true};
  bool link_up_{false};
  bool ever_connected_{false};
  std::chrono::steady_clock::time_point last_rx_{};
  std::chrono::steady_clock::time_point next_connect_{};
  std::string rx_buffer_;
  std::vector<std::string> captured_lines_;
  uint32_t rx_frames_{0u}, tx_frames_{0u}, crc_errors_{0u}, framing_errors_{0u};
  uint32_t truncated_errors_{0u}, short_fields_{0u}, long_fields_{0u}, numeric_errors_{0u};
  uint32_t seq_repeats_{0u}, seq_gaps_{0u}, last_rx_pico_us_{0u}, reconnect_count_{0u};
  uint32_t sensor_seq_{0u}, estimator_seq_{0u};
  bool have_sensor_seq_{false}, have_estimator_seq_{false};
  rclcpp::Publisher<glove_interfaces::msg::SensorFrame>::SharedPtr sensor_pub_;
  rclcpp::Publisher<glove_interfaces::msg::EstimatorInputFrame>::SharedPtr estimator_pub_;
  rclcpp::Publisher<glove_interfaces::msg::PicoStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<glove_interfaces::msg::ActuatorCommand>::SharedPtr actuator_sub_;
  rclcpp::Service<glove_interfaces::srv::CalibrateSensors>::SharedPtr calibrate_srv_;
  rclcpp::TimerBase::SharedPtr io_timer_, status_timer_;
};

#ifndef PICO_BRIDGE_TESTING
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PicoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
