#include <assert.h>
#include <math.h>
#include <string.h>

#include "pico_actuator.h"
#include "pico_frame_parser.h"
#include "pico_protocol.h"
#include "pico_watchdog.h"
#include "pico_sensors.h"

int main(void)
{
  assert(pico_pressure_from_vout(0.2f, 0.2f) == 0.0f);
  assert(fabsf(pico_pressure_from_vout(4.8f, 0.2f) - 230.0f) < 0.0001f);
  assert(pico_ewma_step(0.0f, 10.0f) == 10.0f);
  assert(pico_ewma_step(10.0f, 20.0f) == 12.0f);

  float zero_quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float near_unit_quaternion[4] = {0.999f, 0.01f, -0.02f, 0.03f};
  assert(!pico_quaternion_normalize(zero_quaternion));
  assert(pico_quaternion_normalize(near_unit_quaternion));
  assert(fabsf(near_unit_quaternion[0]) > 0.99f);

  float bno1_last_good[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float bno2_last_good[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  assert(!pico_sensors_commit_quaternion(bno1_last_good, zero_quaternion));
  assert(bno1_last_good[0] == 1.0f && bno1_last_good[1] == 0.0f);
  assert(pico_sensors_commit_quaternion(bno2_last_good, near_unit_quaternion));
  assert(bno2_last_good[0] > 0.99f && bno2_last_good[1] != 0.0f);
  assert(bno1_last_good[0] == 1.0f && bno2_last_good[0] > 0.99f);

  PicoActuatorState a;
  pico_actuator_init(&a);
  assert(a.command == PICO_COMMAND_ALL_OFF && a.safety_latched);
  pico_actuator_arm(&a, true, 20.0f);
  PicoActuatorCommand fill = {1u, PICO_COMMAND_FILL, 10u, 0u};
  assert(pico_actuator_accept(&a, &fill, 1000u));
  assert(a.pulse_active && a.command == PICO_COMMAND_FILL);
  pico_actuator_step(&a, 11001u, 20.0f, true);
  assert(!a.pulse_active && a.command == PICO_COMMAND_HOLD);
  PicoActuatorCommand duplicate = {1u, PICO_COMMAND_FILL, 1u, 0u};
  assert(!pico_actuator_accept(&a, &duplicate, 12000u));
  pico_actuator_arm(&a, true, 20.0f);
  PicoActuatorCommand stale = {2u, PICO_COMMAND_FILL, 1u, 9000u};
  assert(!pico_actuator_accept(&a, &stale, 10000u));
  PicoActuatorCommand over = {3u, PICO_COMMAND_FILL, 1u, 0u};
  assert(pico_actuator_accept(&a, &over, 13000u));
  pico_actuator_step(&a, 14000u, 200.0f, true);
  assert(a.overpressure && a.safety_latched && a.command == PICO_COMMAND_VENT);
  assert(!pico_actuator_accept(&a, &fill, 15000u));

  PicoWatchdogState w;
  pico_watchdog_init(&w, 100u);
  assert(!pico_watchdog_expired(&w, 200u, 200u));
  assert(pico_watchdog_expired(&w, 301u, 200u));

  PicoActuatorCommand command = {71u, PICO_COMMAND_VENT, 19u, 999u};
  char line[PICO_PROTOCOL_MAX_LINE];
  assert(pico_protocol_encode_actuator(&command, line, sizeof(line)));
  PicoParsedFrame parsed;
  assert(pico_frame_parse_line(line, strlen(line), &parsed));
  assert(parsed.kind == PICO_FRAME_ACTUATOR && parsed.value.actuator.command == PICO_COMMAND_VENT);
  line[strlen(line) - 2u] = line[strlen(line) - 2u] == '0' ? '1' : '0';
  assert(!pico_frame_parse_line(line, strlen(line), &parsed));
  return 0;
}
