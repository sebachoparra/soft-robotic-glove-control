#include <assert.h>
#include <math.h>
#include <string.h>

#include "pico_frame_parser.h"

int main(void)
{
  const char *vector = "123456789";
  assert(pico_crc16_ccitt_false((const uint8_t *)vector, strlen(vector)) == 0x29b1u);

  PicoActuatorCommand command = {71u, PICO_COMMAND_VENT, 19u, 999u};
  char line[PICO_PROTOCOL_MAX_LINE];
  assert(pico_protocol_encode_actuator(&command, line, sizeof(line)));
  PicoParsedFrame parsed;
  assert(pico_frame_parse_line(line, strlen(line), &parsed));
  assert(parsed.kind == PICO_FRAME_ACTUATOR);
  assert(!pico_frame_parse_line(line, strlen(line) - 1u, &parsed));
  line[strlen(line) - 2u] = line[strlen(line) - 2u] == '0' ? '1' : '0';
  assert(!pico_frame_parse_line(line, strlen(line), &parsed));
  assert(!pico_frame_parse_line("X,1,2,3,4,0\\n", 12u, &parsed));

  PicoSensorFrame sensor = {0};
  sensor.seq = 8u; sensor.pico_us = 123456u; sensor.pressure_kpa = 12.0f;
  sensor.flex_filtered_raw = 500.0f; sensor.pneumatic_state = PICO_PNEUMATIC_VENT;
  sensor.pressure_raw = 1.0f; sensor.pressure_vout = 0.5f;
  sensor.pressure_filtered_kpa = 11.5f; sensor.flex_raw = 500.0f;
  sensor.flex_filtered_raw = 480.0f; sensor.pulse_active = true;
  sensor.pulse_ms_remaining = 31u; sensor.pump_on = true;
  sensor.alarm_fail_count = 2u; sensor.watchdog_state = 3u;
  assert(pico_protocol_encode_sensor(&sensor, line, sizeof(line)));
  assert(strcmp(line, "S,8,123456,+0x1.000000p+0,+0x1.000000p-1,+0x1.800000p+3,+0x1.700000p+3,+0x1.f40000p+8,+0x1.e00000p+8,-1,1,31,1,0,2,3,22997\n") == 0);
  assert(line[0] == 'S');
  assert(pico_frame_parse_line(line, strlen(line), &parsed));
  assert(parsed.kind == PICO_FRAME_SENSOR && parsed.value.sensor.seq == 8u);

  PicoEstimatorFrame estimator = {0};
  estimator.seq = 13u; estimator.pico_us = 123457u; estimator.bno1_quat[0] = 1.0f;
  estimator.flex_filtered_raw = 480.0f; estimator.bno1_ok = true;
  estimator.bno1_quat[1] = -0.5f; estimator.bno1_quat[3] = 0.25f;
  assert(pico_protocol_encode_estimator(&estimator, line, sizeof(line)));
  assert(strcmp(line, "E,13,123457,+0x1.e00000p+8,1,+0x1.000000p+0,-0x1.000000p-1,+0x0p+0,+0x1.000000p-2,0,+0x0p+0,+0x0p+0,+0x0p+0,+0x0p+0,28726\n") == 0);
  assert(line[0] == 'E');
  assert(pico_frame_parse_line(line, strlen(line), &parsed));
  assert(parsed.kind == PICO_FRAME_ESTIMATOR && parsed.value.estimator.bno1_quat[0] == 1.0f);
  assert(pico_protocol_sequence_accept(UINT32_MAX, true, 0u));

  estimator.bno1_quat[2] = -0.0f;
  estimator.bno2_ok = true;
  estimator.bno2_quat[0] = NAN;
  estimator.bno2_quat[1] = INFINITY;
  estimator.bno2_quat[2] = -INFINITY;
  assert(pico_protocol_encode_estimator(&estimator, line, sizeof(line)));
  assert(strstr(line, "-0x0p+0") != NULL);
  assert(strstr(line, "nan") != NULL);
  assert(strstr(line, "inf") != NULL);
  assert(strstr(line, "-inf") != NULL);
  char tiny[8];
  assert(!pico_protocol_encode_actuator(&command, tiny, sizeof(tiny)));
  return 0;
}
