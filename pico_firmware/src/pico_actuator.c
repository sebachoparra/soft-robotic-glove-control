#include "pico_actuator.h"
#include "pico_frame_parser.h"
#include "pico_board_config.h"
#include <math.h>

/* The test backend links only host GPIO spies. */
#if PICO_ACTUATOR_OUTPUT_ENABLE || defined(PICO_ACTUATOR_TEST_GPIO)
#include "pico/stdlib.h"
#endif

static void apply_outputs(uint8_t command)
{
#if PICO_ACTUATOR_OUTPUT_ENABLE || defined(PICO_ACTUATOR_TEST_GPIO)
  switch (command) {
    case PICO_COMMAND_PUMP_ON: gpio_put(PICO_PUMP_GPIO, 1); break;
    case PICO_COMMAND_PUMP_OFF: gpio_put(PICO_PUMP_GPIO, 0); break;
    case PICO_COMMAND_FILL:
      gpio_put(PICO_V1_GPIO, 0); gpio_put(PICO_V2_GPIO, 0); break;
    case PICO_COMMAND_HOLD:
      gpio_put(PICO_V1_GPIO, 0); gpio_put(PICO_V2_GPIO, 1); break;
    case PICO_COMMAND_VENT:
      gpio_put(PICO_V1_GPIO, 1); gpio_put(PICO_V2_GPIO, 1); break;
    case PICO_COMMAND_ALL_OFF:
      gpio_put(PICO_PUMP_GPIO, 0);
      gpio_put(PICO_V1_GPIO, 0); gpio_put(PICO_V2_GPIO, 0); break;
    default: break;
  }
#else
  (void)command;
#endif
}

void pico_actuator_init(PicoActuatorState *state)
{
  *state = (PicoActuatorState){.command = PICO_COMMAND_ALL_OFF, .safety_latched = true};
#if PICO_ACTUATOR_OUTPUT_ENABLE || defined(PICO_ACTUATOR_TEST_GPIO)
  const uint pins[] = {PICO_PUMP_GPIO, PICO_V1_GPIO, PICO_V2_GPIO};
  for (unsigned i = 0u; i < 3u; ++i) {
    gpio_init(pins[i]);
    gpio_put(pins[i], 0);  /* Preload safe level before enabling output direction. */
    gpio_set_dir(pins[i], GPIO_OUT);
  }
#endif
  apply_outputs(PICO_COMMAND_ALL_OFF);
}

void pico_actuator_arm(PicoActuatorState *state, bool pressure_valid, float pressure_kpa)
{
  state->pressure_valid = pressure_valid && isfinite(pressure_kpa);
  if (state->pressure_valid && pressure_kpa < PICO_HARD_PRESSURE_KPA && !state->overpressure) {
    state->safety_latched = false;
    state->communication_lost = false;
    state->watchdog_latched = false;
  }
}

void pico_actuator_safe(PicoActuatorState *state)
{
  state->command = PICO_COMMAND_ALL_OFF;
  state->pump_on = false;
  state->pulse_active = false;
  apply_outputs(PICO_COMMAND_ALL_OFF);
}

void pico_actuator_failsafe_vent(PicoActuatorState *state)
{
  state->safety_latched = true;
  state->pump_on = false;
  state->pulse_active = false;
  state->command = PICO_COMMAND_VENT;
  apply_outputs(PICO_COMMAND_PUMP_OFF);
  apply_outputs(PICO_COMMAND_VENT);
}

void pico_actuator_watchdog_trip(PicoActuatorState *state)
{
  state->watchdog_latched = true;
  pico_actuator_failsafe_vent(state);
}

bool pico_actuator_accept(PicoActuatorState *state, const PicoActuatorCommand *command,
  uint32_t now_us)
{
  const uint8_t cmd = command->command;
  const bool off = cmd == PICO_COMMAND_ALL_OFF || cmd == PICO_COMMAND_PUMP_OFF;
  const bool timed_valve = cmd == PICO_COMMAND_FILL || cmd == PICO_COMMAND_VENT;
  if (cmd > PICO_COMMAND_ALL_OFF || command->pulse_ms > PICO_MAX_PULSE_MS ||
    (!timed_valve && command->pulse_ms != 0u) ||
    (state->have_sequence && command->seq == state->last_sequence) ||
    pico_protocol_deadline_expired(now_us, command->deadline_us)) return false;
  if (!off && (state->overpressure || state->safety_latched ||
    state->communication_lost || !state->pressure_valid)) return false;
  state->last_sequence = command->seq;
  state->have_sequence = true;
  state->last_command_us = now_us;
  if (cmd == PICO_COMMAND_ALL_OFF) {
    // Explicit acknowledgement can recover a link/invalid-pressure trip, never overpressure.
    if (state->pressure_valid && !state->overpressure) {
      state->safety_latched = false;
      state->communication_lost = false;
      state->watchdog_latched = false;
      pico_actuator_safe(state);
    } else {
      pico_actuator_failsafe_vent(state);
    }
  } else if (cmd == PICO_COMMAND_PUMP_ON || cmd == PICO_COMMAND_PUMP_OFF) {
    state->pump_on = cmd == PICO_COMMAND_PUMP_ON;
    apply_outputs(cmd);
  } else {
    state->command = cmd;
    state->pulse_active = command->pulse_ms != 0u;
    state->pulse_start_us = now_us;
    state->pulse_duration_ms = command->pulse_ms;
    apply_outputs(cmd);
  }
  return true;
}

void pico_actuator_step(PicoActuatorState *state, uint32_t now_us, float pressure_kpa,
  bool pressure_valid)
{
  state->pressure_valid = pressure_valid && isfinite(pressure_kpa);
  state->overpressure = state->overpressure ||
    (state->pressure_valid && pressure_kpa >= PICO_HARD_PRESSURE_KPA);
  if ((uint32_t)(now_us - state->last_command_us) > PICO_COMMAND_TIMEOUT_US) {
    state->communication_lost = true;
  }
  if (!state->pressure_valid || state->overpressure || state->communication_lost ||
    state->watchdog_latched || state->safety_latched) {
    pico_actuator_failsafe_vent(state);
    return;
  }
  if (state->pulse_active && (uint32_t)(now_us - state->pulse_start_us) >=
    state->pulse_duration_ms * 1000u) {
    state->pulse_active = false;
    state->command = PICO_COMMAND_HOLD;
    apply_outputs(PICO_COMMAND_HOLD);
  }
}

void pico_actuator_telemetry(const PicoActuatorState *state, uint32_t now_us,
  PicoSensorFrame *frame)
{
  frame->pneumatic_state = state->command == PICO_COMMAND_FILL ? PICO_PNEUMATIC_FILL :
    state->command == PICO_COMMAND_VENT ? PICO_PNEUMATIC_VENT :
    state->command == PICO_COMMAND_HOLD ? PICO_PNEUMATIC_HOLD : PICO_PNEUMATIC_OFF;
  frame->pump_on = state->pump_on;
  frame->pulse_active = state->pulse_active;
  const uint32_t elapsed = now_us - state->pulse_start_us;
  const uint32_t duration = state->pulse_duration_ms * 1000u;
  frame->pulse_ms_remaining = state->pulse_active && elapsed < duration ?
    (duration - elapsed + 999u) / 1000u : 0u;
  frame->safety_latched = state->safety_latched;
  // Bit 0: command timeout; bit 1: main-loop watchdog trip.
  frame->watchdog_state = (state->communication_lost ? 1u : 0u) |
    (state->watchdog_latched ? 2u : 0u);
}
