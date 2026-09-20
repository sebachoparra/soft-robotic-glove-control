#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include <stdio.h>

#include "pico_actuator.h"
#include "pico_board_config.h"
#include "pico_frame_parser.h"
#include "pico_sensors.h"
#include "pico_watchdog.h"

// The LF-only protocol is transported over USB CDC.
#if !LIB_PICO_STDIO_USB
#error "The LF-only protocol requires USB stdio"
#endif

int main(void)
{
  stdio_init_all();
  // Preserve protocol LF bytes: USB stdio defaults to translating LF to CRLF.
  stdio_set_translate_crlf(&stdio_usb, false);
  PicoSensorState sensors;
  PicoActuatorState actuators;
  PicoWatchdogState watchdog;
  pico_actuator_init(&actuators);

  // Match validated firmware: allow BNO055 devices to finish power-up
  // before CHIP_ID detection and NDOF configuration.
  sleep_ms(1500u);

  pico_sensors_init(&sensors);
  pico_watchdog_init(&watchdog, time_us_32());
  char line[PICO_PROTOCOL_MAX_LINE];
  size_t line_length = 0u;
  uint32_t next_adc = time_us_32();
  uint32_t next_bno = next_adc;
  PicoSensorFrame sensor_frame = {0};
  PicoEstimatorFrame estimator_frame;
  while (true) {
    uint32_t now = time_us_32();
    // Cooperative loop stall detector; host loss is checked by actuator_step.
    if (pico_watchdog_expired(&watchdog, now, PICO_COMMAND_TIMEOUT_US)) {
      pico_actuator_watchdog_trip(&actuators);
    }
    pico_watchdog_feed(&watchdog, now);
    bool sampled = false;
    if ((int32_t)(now - next_adc) >= 0) {
      next_adc += 10000u;
      sampled = pico_sensors_read_adc(&sensors, now, &sensor_frame);
    }
    pico_actuator_step(&actuators, now, sensor_frame.pressure_kpa, sensors.pressure_valid);
    int ch;
    while ((ch = getchar_timeout_us(0)) >= 0) {
      if (line_length >= PICO_PROTOCOL_MAX_LINE) {line_length = 0u; continue;}
      line[line_length++] = (char)ch;
      if (ch == '\n') {
        PicoParsedFrame parsed;
        if (pico_frame_parse_line(line, line_length, &parsed)) {
          if (parsed.kind == PICO_FRAME_ACTUATOR) {
            const PicoActuatorCommand *cmd = &parsed.value.actuator;
            // Zero calibration is required before any pump-powered operation.
            if ((cmd->command != PICO_COMMAND_PUMP_ON || sensors.calibration.calibrated) &&
              (!sensors.zero_active || cmd->command == PICO_COMMAND_VENT ||
              cmd->command == PICO_COMMAND_PUMP_OFF || cmd->command == PICO_COMMAND_ALL_OFF)) {
              (void)pico_actuator_accept(&actuators, cmd, time_us_32());
            }
          } else if (parsed.kind == PICO_FRAME_CALIBRATE) {
            bool vented = !actuators.pump_on && actuators.command == PICO_COMMAND_VENT &&
              !actuators.pulse_active && !actuators.safety_latched;
            if (!pico_sensors_zero_begin(&sensors, parsed.value.calibration.seq,
              time_us_32(), vented)) {
              PicoCalibrationResult rejected = {.seq = parsed.value.calibration.seq};
              char encoded[PICO_PROTOCOL_MAX_LINE];
              if (pico_protocol_encode_calibration(&rejected, encoded, sizeof(encoded))) {
                printf("%s", encoded);
              }
            }
          }
        }
        line_length = 0u;
      }
    }
    now = time_us_32();
    pico_actuator_step(&actuators, now, sensor_frame.pressure_kpa, sensors.pressure_valid);
    PicoCalibrationResult calibration;
    bool vented = !actuators.pump_on && actuators.command == PICO_COMMAND_VENT &&
      !actuators.pulse_active && !actuators.safety_latched;
    if (pico_sensors_zero_step(&sensors, now, vented, &calibration)) {
      // Normal calibration ends ALL_OFF; aborted calibration must preserve VENT.
      if (!actuators.safety_latched) pico_actuator_safe(&actuators);
      if (calibration.success) {
        sampled = pico_sensors_read_adc(&sensors, now, &sensor_frame);
      }
      char encoded[PICO_PROTOCOL_MAX_LINE];
      if (pico_protocol_encode_calibration(&calibration, encoded, sizeof(encoded))) {
        printf("%s", encoded);
      }
    }
    if (sampled) {
      pico_actuator_telemetry(&actuators, now, &sensor_frame);
      char encoded[PICO_PROTOCOL_MAX_LINE];
      if (pico_protocol_encode_sensor(&sensor_frame, encoded, sizeof(encoded))) printf("%s", encoded);
    }
    if ((int32_t)(now - next_bno) >= 0) {
      next_bno += 20000u;
      // Always populated, including invalid flags and last-good quaternions.
      (void)pico_sensors_read_bno(&sensors, now, &estimator_frame);
      char encoded[PICO_PROTOCOL_MAX_LINE];
      if (pico_protocol_encode_estimator(&estimator_frame, encoded, sizeof(encoded))) printf("%s", encoded);
    }
    tight_loop_contents();
  }
  return 0;
}
