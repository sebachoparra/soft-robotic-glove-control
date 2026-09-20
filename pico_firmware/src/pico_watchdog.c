#include "pico_watchdog.h"

void pico_watchdog_init(PicoWatchdogState *state, uint32_t now_us)
{
  state->last_feed_us = now_us;
  state->initialized = true;
}

void pico_watchdog_feed(PicoWatchdogState *state, uint32_t now_us)
{
  state->last_feed_us = now_us;
  state->initialized = true;
}

bool pico_watchdog_expired(const PicoWatchdogState *state, uint32_t now_us, uint32_t timeout_us)
{
  return !state->initialized || (int32_t)(now_us - state->last_feed_us) > (int32_t)timeout_us;
}
