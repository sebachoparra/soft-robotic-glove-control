// Copyright 2026 Sebastian Parra

#include <cassert>
#include <chrono>
#include <memory>
#include <string>

#define PICO_BRIDGE_TESTING
#include "../src/pico_bridge_node.cpp"  // NOLINT(build/include)
extern "C" {
#include "pico_actuator.h"  // NOLINT(build/include_subdir)
#include "pico_board_config.h"  // NOLINT(build/include_subdir)
#include "pico_frame_parser.h"  // NOLINT(build/include_subdir)
}

static bool levels[32];
extern "C" void gpio_init(uint32_t) {}
extern "C" void gpio_set_dir(uint32_t, bool) {}
extern "C" void gpio_put(uint32_t pin, bool value) {assert(pin < 32); levels[pin] = value;}

class Harness
{
public:
  Harness()
  {
    master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0 && ::grantpt(master) == 0 && ::unlockpt(master) == 0);
    rclcpp::NodeOptions options;
    options.append_parameter_override("port", std::string(::ptsname(master)));
    options.append_parameter_override("reconnect_backoff_ms", 0);
    bridge = std::make_shared<PicoBridgeNode>(options, [this]() {
          return std::chrono::steady_clock::time_point{} + std::chrono::microseconds(us);
      });
    pico_actuator_init(&pico);
    pico_actuator_step(&pico, us, 20, true);
    assert(!bridge->link_up());
  }

  ~Harness() {bridge.reset(); ::close(master);}

  void advance(uint32_t delta)
  {
    us += delta;
    pico_actuator_step(&pico, us, 20, true);
  }

  void rx(const std::string & wire)
  {
    assert(::write(master, wire.data(), wire.size()) == static_cast<ssize_t>(wire.size()));
    bridge->test_poll_transport();
  }

  void sensor()
  {
    PicoSensorFrame f = {};
    f.seq = ++rx_seq;
    f.pico_us = us;
    f.pressure_kpa = 20;
    pico_actuator_telemetry(&pico, us, &f);
    char wire[PICO_PROTOCOL_MAX_LINE];
    assert(pico_protocol_encode_sensor(&f, wire, sizeof(wire)));
    rx(wire);
  }

  void invalid_imus()
  {
    PicoEstimatorFrame e = {};
    e.seq = ++rx_seq;
    e.pico_us = us;
    e.bno1_quat[0] = e.bno2_quat[0] = 1;
    char wire[PICO_PROTOCOL_MAX_LINE];
    assert(pico_protocol_encode_estimator(&e, wire, sizeof(wire)));
    rx(wire);
  }

  void command(uint8_t cmd, uint32_t pulse = 0)
  {
    glove_interfaces::msg::ActuatorCommand msg;
    msg.seq = ++tx_seq;
    msg.command = cmd;
    msg.pulse_ms = pulse;
    bridge->on_actuator(msg);
    drain();
  }

  void drain()
  {
    char bytes[512];
    auto n = ::read(master, bytes, sizeof(bytes));
    if (n < 0) {assert(errno == EAGAIN || errno == EWOULDBLOCK); return;}
    assert(n > 0);
    std::string wire(bytes, static_cast<size_t>(n));
    size_t start = 0;
    while (start < wire.size()) {
      size_t end = wire.find('\n', start);
      assert(end != std::string::npos);
      PicoParsedFrame frame;
      assert(pico_frame_parse_line(wire.data() + start, end - start + 1, &frame));
      assert(frame.kind == PICO_FRAME_ACTUATOR);
      assert(pico_actuator_accept(&pico, &frame.value.actuator, us));
      ++received_commands;
      start = end + 1;
    }
  }

  void active()
  {
    sensor();
    command(PICO_COMMAND_ALL_OFF);
    command(PICO_COMMAND_PUMP_ON);
    command(PICO_COMMAND_FILL, 1000);
    assert(pico.pump_on && pico.pulse_active && levels[PICO_PUMP_GPIO]);
  }

  void vented()
  {
    assert(pico.communication_lost && pico.safety_latched);
    assert(!pico.pump_on && !pico.pulse_active && pico.command == PICO_COMMAND_VENT);
    assert(!levels[PICO_PUMP_GPIO] && levels[PICO_V1_GPIO] && levels[PICO_V2_GPIO]);
  }

  int master;
  uint32_t us{0}, rx_seq{0}, tx_seq{0}, received_commands{0};
  PicoActuatorState pico{};
  std::shared_ptr<PicoBridgeNode> bridge;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    Harness h;
    // No RX at startup: even explicit ALL_OFF must not keep the Pico timer alive.
    h.command(PICO_COMMAND_ALL_OFF);
    assert(h.received_commands == 0);
    h.active();
    h.advance(99000);
    h.command(PICO_COMMAND_FILL, 1000);
    assert(h.bridge->link_up());
    auto sent = h.received_commands;
    h.advance(1000);
    // B1: command callback itself detects expiry, without polling first.
    for (uint8_t cmd = 0; cmd <= PICO_COMMAND_ALL_OFF; ++cmd) {
      h.command(cmd);
    }
    assert(!h.bridge->link_up() && h.received_commands == sent);
    assert(h.bridge->tx_frames() == sent);
    // B2: continuing ROS commands cannot refresh actual firmware last_command_us.
    for (unsigned i = 0; i < 19; ++i) {
      h.advance(10000);
      h.command(PICO_COMMAND_VENT);
      h.command(PICO_COMMAND_PUMP_ON);
    }
    h.advance(9000);  // Exactly 200 ms from last accepted A.
    assert(h.pico.pump_on && !h.pico.communication_lost);
    h.advance(1);
    h.vented();
    assert(h.received_commands == sent);
    // B3: valid RX restores link only, and never replays an old command.
    h.sensor();
    assert(h.bridge->link_up());
    h.drain();
    for (uint8_t cmd = 0; cmd < PICO_COMMAND_ALL_OFF; ++cmd) {
      h.command(cmd);
    }
    assert(h.received_commands == sent);
    h.vented();
    h.command(PICO_COMMAND_ALL_OFF);
    assert(!h.pico.safety_latched && !h.pico.communication_lost);
    assert(!h.pico.pump_on && !h.pico.pulse_active && h.pico.command == PICO_COMMAND_ALL_OFF);
    assert(!levels[PICO_PUMP_GPIO] && !levels[PICO_V1_GPIO] && !levels[PICO_V2_GPIO]);
    h.command(PICO_COMMAND_PUMP_ON);
    assert(h.pico.pump_on);
    // B4: sensor-only traffic and E with both IMUs invalid each keep RX fresh.
    for (unsigned i = 0; i < 30; ++i) {
      h.advance(10000);
      h.sensor();
      h.invalid_imus();
      h.command(PICO_COMMAND_HOLD);
      assert(h.bridge->link_up());
    }
    for (unsigned i = 0; i < 30; ++i) {
      h.advance(10000);
      h.sensor();
      assert(h.bridge->link_up());
    }
    for (unsigned i = 0; i < 30; ++i) {
      h.advance(20000);
      h.invalid_imus();
      assert(h.bridge->link_up());
    }
    // B5: bad CRC and CRC-valid rejected frames do not renew freshness.
    sent = h.received_commands;
    for (unsigned i = 0; i < 10; ++i) {
      h.advance(10000);
      h.rx("E,0,0,0\n");
      h.rx(with_crc("E,0,0,"));
      h.rx(with_crc("UNKNOWN,0,"));
    }
    assert(!h.bridge->link_up() && h.bridge->crc_errors() > 0);
    for (uint8_t cmd = 0; cmd <= PICO_COMMAND_ALL_OFF; ++cmd) {
      h.command(cmd);
    }
    assert(h.received_commands == sent);
    // A fresh frame arriving before the stale timer must still latch rearm.
    h.sensor();
    h.command(PICO_COMMAND_ALL_OFF);
    h.advance(100000);
    h.sensor();
    sent = h.received_commands;
    h.command(PICO_COMMAND_PUMP_ON);
    assert(h.bridge->link_up() && h.received_commands == sent);
    // Reopening a writable port with zero backoff must not renew RX or rearm.
    h.bridge->test_disconnect_serial();
    h.bridge->test_poll_transport();
    assert(!h.bridge->link_up());
    h.command(PICO_COMMAND_ALL_OFF);
    h.command(PICO_COMMAND_PUMP_ON);
    assert(h.received_commands == sent);
    h.sensor();
    h.command(PICO_COMMAND_PUMP_ON);
    assert(h.bridge->link_up() && h.received_commands == sent);
    // Destruction must not synthesize an ALL_OFF acknowledgement.
    h.bridge.reset();
    char bytes[512];
    assert(::read(h.master, bytes, sizeof(bytes)) <= 0);
  }
  {
    // An RX callback can arrive before the stale timer after an executor pause.
    auto time = std::chrono::steady_clock::time_point{};
    rclcpp::NodeOptions options;
    options.append_parameter_override("transport", "loopback");
    auto bridge = std::make_shared<PicoBridgeNode>(options, [&]() {return time;});
    const auto frame = with_crc("Z,1,1,0x0p+0,0x0p+0,");
    bridge->inject_loopback_line(frame);
    glove_interfaces::msg::ActuatorCommand command;
    command.command = PICO_COMMAND_ALL_OFF;
    bridge->on_actuator(command);
    time += std::chrono::milliseconds(100);
    bridge->inject_loopback_line(frame);
    auto sent = bridge->tx_frames();
    command.command = PICO_COMMAND_PUMP_ON;
    bridge->on_actuator(command);
    assert(bridge->link_up() && bridge->tx_frames() == sent);
  }
  rclcpp::shutdown();
  return 0;
}
