#ifndef PICO_ACTUATOR_H_
#define PICO_ACTUATOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "pico_protocol.h"

typedef struct {
  uint8_t command;  /* Last valve disposition, independent of the pump. */
  bool pump_on;
  bool pressure_valid;
  bool watchdog_latched;
  bool pulse_active;
  uint32_t pulse_start_us;
  uint32_t pulse_duration_ms;
  uint32_t last_command_us;
  uint32_t last_sequence;
  bool have_sequence;
  bool safety_latched;
  bool overpressure;
  bool communication_lost;
} PicoActuatorState;

void pico_actuator_init(PicoActuatorState *state);
bool pico_actuator_accept(PicoActuatorState *state, const PicoActuatorCommand *command,
  uint32_t now_us);
void pico_actuator_step(PicoActuatorState *state, uint32_t now_us, float pressure_kpa,
  bool pressure_valid);
/* Normal ALL_OFF: pump off, both valves closed. Only call after rearm or with no fault latched. */
void pico_actuator_safe(PicoActuatorState *state);
/* Emergency: pump off, both valves open, pulses cancelled, fault latched. */
void pico_actuator_failsafe_vent(PicoActuatorState *state);
void pico_actuator_watchdog_trip(PicoActuatorState *state);
void pico_actuator_telemetry(const PicoActuatorState *state, uint32_t now_us,
  PicoSensorFrame *frame);
void pico_actuator_arm(PicoActuatorState *state, bool pressure_valid, float pressure_kpa);

#endif
