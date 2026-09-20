# Troubleshooting

Common failure modes for the ROS2/Pico glove stack, how they present, and how
to recover. Based on `pico_firmware/FAILSAFE_VALIDATION.md`,
`ros2_ws/src/pico_bridge/RX_FRESHNESS.md`, `docs/PRESSURE_BRINGUP.md`, and the
safety-latch/overpressure logic in `pico_firmware/src/pico_actuator.c`.

## 1. Host-to-Pico command loss ("communication_lost")

**Cause:** No `A` (actuator) command accepted by the Pico for more than 200 ms
(`PICO_COMMAND_TIMEOUT_US`, `pico_firmware/include/pico_board_config.h:25`),
checked in `pico_actuator_step()` (`pico_firmware/src/pico_actuator.c:125-127`).

**Presentation:** `communication_lost = true`; the Pico forces sustained VENT
(`pico_actuator_failsafe_vent()`, pump OFF, both valves open, `safety_latched
= true`) every step until acknowledged (`pico_firmware/src/pico_actuator.c:128-132`,
`pico_firmware/FAILSAFE_VALIDATION.md` table). `SensorFrame.watchdog_state`
bit 0 is set (`pico_firmware/src/pico_actuator.c:154-156`).

**Recovery:** Requires an explicit host `ALL_OFF` (`command=5`) accepted while
pressure is valid and not overpressure; this clears `communication_lost`,
`safety_latched`, and `watchdog_latched` (never `overpressure`)
(`pico_firmware/src/pico_actuator.c:96-105`). A new command is then needed to
re-energize the pump — acknowledgement alone never turns the pump on.

## 2. Pico-to-host telemetry loss (one-way, RX staleness at the bridge)

**Cause:** No valid `S`/`E`/`Z` frame accepted by `pico_bridge_node` within
`rx_timeout_ms` (default 100 ms) of host time
(`ros2_ws/src/pico_bridge/src/pico_bridge_node.cpp:257-264`,
`ros2_ws/src/pico_bridge/RX_FRESHNESS.md`).

**Presentation:** Bridge sets `link_up = false`, publishes it on
`/pico_bridge/pico_status`, latches `rearm_required = true`, and refuses to
transmit *any* further `A` commands, including ALL_OFF/PUMP_OFF/periodic
VENT, regardless of continuing ROS command traffic
(`ros2_ws/src/pico_bridge/RX_FRESHNESS.md`, "RX health and TX gate"). Because
the bridge stops sending commands, the Pico's own 200 ms command timeout then
trips independently (failure mode 1 above), producing sustained local VENT.
End-to-end nominal bound is roughly 100 ms + 200 ms plus transport/execution
latency, not 200 ms alone.

**Recovery:** Once valid RX resumes, link health recovers but actuation stays
blocked until the host sends an explicit `ALL_OFF` with zero pulse duration
over the now-fresh link; only a fully successful serial write of that
acknowledgement clears the bridge-side gate
(`ros2_ws/src/pico_bridge/RX_FRESHNESS.md`, "Recovery"). The bridge does not
auto-generate an ALL_OFF on destruction/shutdown — it just stops TX and lets
the Pico's local timeout handle safety.

## 3. Overpressure trip

**Cause:** `pressure_kpa >= PICO_HARD_PRESSURE_KPA` (200 kPa) while pressure
is valid, detected in `pico_actuator_step()`
(`pico_firmware/src/pico_actuator.c:123-124`).

**Presentation:** `overpressure = true` latched permanently ("sticky") —
pump OFF and sustained VENT, `safety_latched = true`.

**Recovery:** None from the wire protocol. Overpressure cannot be cleared by
any accepted command, including ALL_OFF (`pico_actuator_accept()`,
`pico_firmware/src/pico_actuator.c:91-105`, gate:
`!off && (state->overpressure || ...)`, and the ALL_OFF branch itself only
clears the *other* latches when `!state->overpressure`). This is terminal for
that Pico instance/session — per `pico_firmware/FAILSAFE_VALIDATION.md`,
"sticky overpressure ... even after pressure falls." Recovery requires a
power cycle / firmware reset. Never acknowledge an overpressure trip to try
to resume a run (`docs/PRESSURE_BRINGUP.md`, step 1).

## 4. Invalid/nonfinite pressure reading

**Cause:** `pressure_valid = false` or non-finite pressure value passed into
`pico_actuator_step()` (`pico_firmware/src/pico_actuator.c:122`).

**Presentation:** Same failsafe branch as communication loss — sustained
VENT, `safety_latched = true`. Restored validity alone does not clear the
latch (`pico_firmware/FAILSAFE_VALIDATION.md` table: "restored validity alone
does not rearm").

**Recovery:** Explicit ALL_OFF once pressure is valid again and not
overpressure (same path as failure mode 1).

## 5. Main-loop / cooperative watchdog trip

**Cause:** The Pico main loop stalls long enough that
`pico_watchdog_expired()` returns true — elapsed time since the last feed
exceeds `PICO_COMMAND_TIMEOUT_US` (200 ms), checked once per loop iteration
before the loop re-feeds the watchdog (`pico_firmware/src/main.c:42-45`,
`pico_firmware/include/pico_watchdog.h:7-8`). This is a *cooperative* check —
it only detects a stall once execution resumes, and cannot react during a
genuine permanent CPU/I2C hang (`pico_firmware/FAILSAFE_VALIDATION.md`,
"Watchdog and one-way telemetry loss").

**Presentation:** `pico_actuator_watchdog_trip()` sets `watchdog_latched =
true` and forces sustained VENT (`pico_firmware/src/pico_actuator.c:75-79`).
`SensorFrame.watchdog_state` bit 1 is set.

**Recovery:** Same ALL_OFF acknowledgement path as failure mode 1 (clears
`watchdog_latched` along with the other non-overpressure latches).

## 6. Calibration cannot start / rejected

**Cause:** `pico_bridge_node`'s `/pico_bridge/calibrate_sensors` service
requires no concurrent calibration in progress and the link to be up
(`ros2_ws/src/pico_bridge/src/pico_bridge_node.cpp:149-161`); the Pico itself
rejects `zero_begin` unless outputs are vented (pump off, command=VENT, no
pulse, no safety latch) (`pico_firmware/src/main.c:67-77`,
`docs/PRESSURE_BRINGUP.md` "Vented pressure-zero primitive"). Calibration
also times out after 2 seconds if no matching `Z` response arrives
(`pico_bridge_node.cpp:157`, `:443-444`).

**Presentation:** Service returns `success=false` with a message such as
"Calibration busy or Pico disconnected" or "Pico calibration response timed
out"; if the link drops mid-calibration, it fails with "Pico disconnected
during calibration" (`pico_bridge_node.cpp:150-152`, `:253`, `:444`).

**Recovery:** Ensure valid pressure telemetry, send a fresh ALL_OFF to
acknowledge any boot/link latch, command VENT and hold it (refreshing every
50 ms with a distinct sequence number to avoid tripping the 200 ms timeout)
for the validated 10 seconds before retrying calibration
(`docs/PRESSURE_BRINGUP.md`, "Preparation" steps 1-2). PUMP_ON is rejected
by the Pico until zero calibration has succeeded at least once since boot
(`pico_firmware/src/main.c:61-63`); calibration is RAM-only and does not
survive a reboot (`docs/PRESSURE_BRINGUP.md`).

## Quick reference: which latches ALL_OFF can clear

| Latch | Cleared by accepted ALL_OFF (pressure valid, not overpressure)? |
|---|---|
| `communication_lost` | Yes |
| `safety_latched` | Yes |
| `watchdog_latched` | Yes |
| `overpressure` | No — sticky, terminal until reset |

Source: `pico_firmware/src/pico_actuator.c:96-105` (accept-time ALL_OFF
branch) and `:47-55` (`pico_actuator_arm`).
