#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico_actuator.h"
#include "pico_board_config.h"
#include "pico_frame_parser.h"
#include "pico_sensors.h"
#include "pico_watchdog.h"

_Static_assert(PICO_ACTUATOR_OUTPUT_ENABLE == 0, "Tests require disabled physical outputs");
static bool levels[32], initialized[32], directed[32], preloaded[32];
static unsigned adc_channel, adc_calls[3];
static bool bno_mock, bno_read_ok;
static unsigned startup_delays;
static uint8_t bno_register;
static uint16_t adc_values[3] = {400, 0, 2000};
i2c_inst_t *i2c0;
i2c_inst_t *i2c1;
void gpio_init(uint32_t p) { initialized[p] = true; }
void gpio_put(uint32_t p, bool v) {
  assert(initialized[p]);
  if (!directed[p]) { assert(!v); preloaded[p] = true; }
  levels[p] = v;
}
void gpio_set_dir(uint32_t p, bool out) {
  assert(initialized[p] && preloaded[p] && out && !levels[p]); directed[p] = true;
}
void gpio_set_function(uint32_t p, int f) { (void)p; (void)f; }
void gpio_pull_up(uint32_t p) { (void)p; }
void sleep_us(uint64_t us) { (void)us; }
void sleep_ms(uint32_t ms) { if (ms == 1500) ++startup_delays; }
void adc_init(void) {}
void adc_gpio_init(uint32_t p) { (void)p; }
void adc_select_input(uint32_t ch) { adc_channel = ch; }
uint16_t adc_read(void) { ++adc_calls[adc_channel]; return adc_values[adc_channel]; }
uint32_t i2c_init(i2c_inst_t *i, uint32_t b) { (void)i; return b; }
int i2c_write_blocking(i2c_inst_t *i, uint8_t a, const uint8_t *b, size_t n, bool s) {
  (void)i; (void)a; (void)s;
  if (!bno_mock) return -1;
  bno_register = b[0];
  return (int)n;
}
int i2c_read_blocking(i2c_inst_t *i, uint8_t a, uint8_t *b, size_t n, bool s) {
  (void)i; (void)s;
  if (!bno_mock) return -1;
  memset(b, 0, n);
  if (bno_register == 0 && n == 1) { b[0] = 0xa0; return 1; }
  if (!bno_read_ok) return -1;
  assert(bno_register == 0x20 && n == 8);
  b[a == PICO_BNO1_ADDRESS ? 1 : 3] = 0x40; // unit w / unit x
  return (int)n;
}
static void pins(bool pump, bool v1, bool v2) {
  assert(levels[PICO_PUMP_GPIO] == pump);
  assert(levels[PICO_V1_GPIO] == v1);
  assert(levels[PICO_V2_GPIO] == v2);
}
static uint32_t seq;
static void send(PicoActuatorState *s, uint8_t cmd, uint32_t pulse, uint32_t now) {
  PicoActuatorCommand c = {++seq, cmd, pulse, 0};
  char wire[PICO_PROTOCOL_MAX_LINE]; PicoParsedFrame parsed;
  assert(pico_protocol_encode_actuator(&c, wire, sizeof(wire)));
  assert(pico_frame_parse_line(wire, strlen(wire), &parsed));
  assert(pico_actuator_accept(s, &parsed.value.actuator, now));
}
static void assert_failsafe(PicoActuatorState *a) {
  pins(0, 1, 1);
  assert(a->safety_latched && !a->pump_on && !a->pulse_active);
  assert(a->command == PICO_COMMAND_VENT);
  PicoSensorFrame f = {0};
  pico_actuator_telemetry(a, 0, &f);
  assert(f.pneumatic_state == PICO_PNEUMATIC_VENT && f.safety_latched);
  assert(!f.pump_on && !f.pulse_active && !f.pulse_ms_remaining);
}
static void test_faults(void) {
  // T1-T5, each from pump ON + an active FILL pulse, including wraparound.
  for (unsigned fault = 0; fault < 5; ++fault) {
    PicoActuatorState a;
    pico_actuator_init(&a);
    uint32_t start = fault == 4 ? UINT32_MAX - 100000u : 1000u;
    a.last_command_us = start;
    pico_actuator_step(&a, start, 20, true);
    send(&a, PICO_COMMAND_ALL_OFF, 0, start);
    send(&a, PICO_COMMAND_PUMP_ON, 0, start);
    send(&a, PICO_COMMAND_FILL, 1000, start);
    pins(1, 0, 0); assert(a.pulse_active);
    uint32_t now = start + 1;
    if (fault == 0 || fault == 4) {
      pico_actuator_step(&a, start + PICO_COMMAND_TIMEOUT_US, 20, true);
      assert(a.pump_on && a.pulse_active && !a.communication_lost);
      now = start + PICO_COMMAND_TIMEOUT_US + 1;
      pico_actuator_step(&a, now, 20, true);
      assert(a.communication_lost);
    } else if (fault == 1) {
      pico_actuator_watchdog_trip(&a); assert(a.watchdog_latched);
    } else if (fault == 2) {
      pico_actuator_step(&a, now, PICO_HARD_PRESSURE_KPA, true);
      assert(a.overpressure);
    } else {
      pico_actuator_step(&a, now, 20, false);
    }
    assert_failsafe(&a);
    const uint8_t forbidden[] = {PICO_COMMAND_FILL, PICO_COMMAND_PUMP_ON, PICO_COMMAND_HOLD};
    for (unsigned i = 0; i < sizeof(forbidden); ++i) {
      PicoActuatorCommand c = {++seq, forbidden[i], 0, 0};
      uint32_t last = a.last_command_us;
      assert(!pico_actuator_accept(&a, &c, now));
      assert(a.last_command_us == last); assert_failsafe(&a);
    }
    send(&a, PICO_COMMAND_PUMP_OFF, 0, now); assert_failsafe(&a);
    if (fault == 2 || fault == 3) {
      send(&a, PICO_COMMAND_ALL_OFF, 0, now); assert_failsafe(&a);
    }
    pico_actuator_step(&a, now + 1, 20, true); assert_failsafe(&a);
    send(&a, PICO_COMMAND_ALL_OFF, 0, now + 2);
    if (fault == 2) { assert_failsafe(&a); continue; }
    pins(0, 0, 0);
    assert(!a.safety_latched && !a.communication_lost && !a.watchdog_latched);
    assert(!a.pump_on && !a.pulse_active && a.command == PICO_COMMAND_ALL_OFF);
    pico_actuator_step(&a, now + 3, 20, true); pins(0, 0, 0);
    send(&a, PICO_COMMAND_PUMP_ON, 0, now + 4); pins(1, 0, 0);
  }
}
int main(void) {
  PicoActuatorState a;
  pico_actuator_init(&a); pins(0, 0, 0);
  assert(a.safety_latched && !a.pump_on);
  PicoActuatorCommand boot_on = {1, PICO_COMMAND_PUMP_ON, 0, 0};
  assert(!pico_actuator_accept(&a, &boot_on, 0));
  pico_actuator_step(&a, 0, 0, true);
  send(&a, PICO_COMMAND_ALL_OFF, 0, 0); // Explicit acknowledge after valid pressure.
  send(&a, PICO_COMMAND_PUMP_ON, 0, 1); pins(1, 0, 0); assert(a.pump_on);
  send(&a, PICO_COMMAND_HOLD, 0, 2); pins(1, 0, 1);
  send(&a, PICO_COMMAND_VENT, 0, 3); pins(1, 1, 1);
  send(&a, PICO_COMMAND_PUMP_OFF, 0, 4); pins(0, 1, 1); assert(!a.pump_on);
  send(&a, PICO_COMMAND_FILL, 0, 5); pins(0, 0, 0); assert(!a.pump_on);
  send(&a, PICO_COMMAND_PUMP_ON, 0, 6); pins(1, 0, 0);
  for (unsigned cmd = PICO_COMMAND_FILL; cmd <= PICO_COMMAND_VENT; ++cmd) {
    send(&a, cmd, 10, 1000);
    pins(1, cmd == PICO_COMMAND_VENT, cmd == PICO_COMMAND_VENT);
    pico_actuator_step(&a, 10999, 20, true); assert(a.pulse_active);
    PicoSensorFrame frame = {0}; pico_actuator_telemetry(&a, 10999, &frame);
    assert(frame.pump_on && frame.pulse_active && frame.pulse_ms_remaining == 1);
    assert(frame.pneumatic_state == (cmd == PICO_COMMAND_FILL ? 1 : -1));
    pico_actuator_step(&a, 11000, 20, true); pins(1, 0, 1);
    assert(!a.pulse_active && a.pump_on && a.command == PICO_COMMAND_HOLD);
    pico_actuator_telemetry(&a, 11000, &frame);
    assert(frame.pneumatic_state == 0 && !frame.pulse_active && frame.pulse_ms_remaining == 0);
  }
  send(&a, PICO_COMMAND_ALL_OFF, 0, 12000); pins(0, 0, 0);
  send(&a, PICO_COMMAND_PUMP_ON, 0, 12001);
  PicoActuatorCommand bad = {seq, PICO_COMMAND_FILL, 5, 0};
  assert(!pico_actuator_accept(&a, &bad, 12002)); // duplicate
  bad.seq = seq + 1; bad.deadline_us = 100;
  assert(!pico_actuator_accept(&a, &bad, 12002));
  bad.deadline_us = 0; bad.pulse_ms = PICO_MAX_PULSE_MS + 1;
  assert(!pico_actuator_accept(&a, &bad, 12002));
  assert(!a.pulse_active && a.pump_on); // Invalid duration did not partially apply.
  pico_actuator_step(&a, 212001, 20, true); assert(a.pump_on); // exactly 200 ms
  pico_actuator_step(&a, 212002, 20, true); pins(0, 1, 1);
  assert(a.communication_lost && a.safety_latched);
  PicoSensorFrame frame = {0}; pico_actuator_telemetry(&a, 212002, &frame);
  assert(!frame.pump_on && frame.safety_latched && frame.watchdog_state == 1);
  assert(frame.pneumatic_state == PICO_PNEUMATIC_VENT);
  bad.pulse_ms = 5; assert(!pico_actuator_accept(&a, &bad, 212003));
  send(&a, PICO_COMMAND_ALL_OFF, 0, 212004);
  send(&a, PICO_COMMAND_PUMP_ON, 0, 212005);
  PicoWatchdogState watchdog; pico_watchdog_init(&watchdog, 212005);
  assert(pico_watchdog_expired(&watchdog, 412006, PICO_COMMAND_TIMEOUT_US));
  pico_actuator_watchdog_trip(&a); pins(0, 1, 1);
  pico_actuator_telemetry(&a, 412006, &frame);
  assert(frame.safety_latched && frame.watchdog_state == 2);
  send(&a, PICO_COMMAND_ALL_OFF, 0, 412007);
  send(&a, PICO_COMMAND_PUMP_ON, 0, 412008);
  pico_actuator_step(&a, 412009, NAN, true); pins(0, 1, 1); assert(a.safety_latched);
  pico_actuator_step(&a, 412010, 20, false); assert(a.safety_latched);
  pico_actuator_step(&a, 412011, 20, true); assert(a.safety_latched);
  send(&a, PICO_COMMAND_ALL_OFF, 0, 412012);
  send(&a, PICO_COMMAND_PUMP_ON, 0, 412013);
  pico_actuator_step(&a, 412014, 200, true); pins(0, 1, 1);
  assert(a.overpressure && a.safety_latched);
  pico_actuator_step(&a, 412015, 20, true);
  send(&a, PICO_COMMAND_ALL_OFF, 0, 412016);
  pico_actuator_arm(&a, true, 20); assert(a.overpressure && a.safety_latched);
  bad.seq = ++seq; bad.command = PICO_COMMAND_PUMP_ON; bad.pulse_ms = 0;
  assert(!pico_actuator_accept(&a, &bad, 412017));

  PicoSensorState sensors; pico_sensors_init(&sensors);
  assert(pico_sensors_read_adc(&sensors, 0, &frame));
  float vout = (400.0f * 3.3f / 4095.0f) * 1.545f;
  assert(frame.pressure_vout == vout && frame.pressure_kpa == vout * 50.0f);
  assert(pico_pressure_from_vout(0.1f, 0.2f) == -5.0f);
  assert(!pico_sensors_zero_begin(&sensors, 1, 0, false));
  assert(pico_sensors_zero_begin(&sensors, 1, 0, true));
  assert(!pico_sensors_zero_begin(&sensors, 2, 0, true));
  unsigned p_before = adc_calls[0], f_before = adc_calls[2];
  PicoCalibrationResult result;
  for (unsigned i = 0; i < 500; ++i) {
    assert(pico_sensors_zero_step(&sensors, i * 200, true, &result) == (i == 499));
  }
  assert(adc_calls[0] - p_before == 500 && adc_calls[2] - f_before == 500);
  assert(result.success && result.seq == 1 && result.pressure_zero_vout == vout);
  assert(sensors.calibration.calibrated && result.flex_zero_raw == 2000);
  assert(pico_sensors_read_adc(&sensors, 100000, &frame));
  assert(frame.pressure_kpa == 0 && frame.pressure_filtered_kpa == 0);
  adc_values[0] = 500;
  assert(pico_sensors_read_adc(&sensors, 110000, &frame));
  assert(frame.pressure_kpa == (((500.0f * 3.3f / 4095.0f) * 1.545f) - vout) * 50.0f);
  float invalid = NAN, flex = 2000;
  assert(!pico_sensors_calibrate(&sensors, &invalid, 1, &flex, 1));
  assert(sensors.calibration.pressure_zero_vout == vout);
  assert(pico_sensors_zero_begin(&sensors, 3, 0, true));
  assert(pico_sensors_zero_step(&sensors, 0, false, &result));
  assert(!result.success && sensors.calibration.pressure_zero_vout == vout);
  char wire[PICO_PROTOCOL_MAX_LINE]; PicoParsedFrame parsed;
  result = (PicoCalibrationResult){12, true, vout, 2000};
  assert(pico_protocol_encode_calibration(&result, wire, sizeof(wire)));
  assert(pico_frame_parse_line(wire, strlen(wire), &parsed));
  assert(parsed.kind == PICO_FRAME_CALIBRATION_RESULT && parsed.value.calibration.success);
  assert(parsed.value.calibration.pressure_zero_vout == vout);
  const char *request = "C,42,";
  snprintf(wire, sizeof(wire), "%s%u\n", request,
    pico_crc16_ccitt_false((const uint8_t *)request, strlen(request)));
  assert(pico_frame_parse_line(wire, strlen(wire), &parsed));
  assert(parsed.kind == PICO_FRAME_CALIBRATE && parsed.value.calibration.seq == 42);
  test_faults();
  puts("GPIO, pump/valve independence, pulse expiry, safety, telemetry, calibration: PASS");
  return 0;
}
