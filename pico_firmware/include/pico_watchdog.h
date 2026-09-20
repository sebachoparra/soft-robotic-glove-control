#ifndef PICO_WATCHDOG_H_
#define PICO_WATCHDOG_H_

#include <stdbool.h>
#include <stdint.h>

/* Cooperative loop delay detector, checked when the loop resumes.
 * Not a hardware watchdog or the host-command timeout (last_command_us). */
typedef struct { uint32_t last_feed_us; bool initialized; } PicoWatchdogState;
void pico_watchdog_init(PicoWatchdogState *state, uint32_t now_us);
void pico_watchdog_feed(PicoWatchdogState *state, uint32_t now_us);
bool pico_watchdog_expired(const PicoWatchdogState *state, uint32_t now_us, uint32_t timeout_us);

#endif
