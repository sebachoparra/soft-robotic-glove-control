# Pico Firmware (`pico_firmware/`)

This document describes the current, ROS2-integrated RP2040 firmware under
`pico_firmware/`. It is based on reading `pico_firmware/include/*.h`,
`pico_firmware/src/*.c` (excluding the stray backup file
`pico_firmware/src/pico_sensors.c.ndof_backup`, which was intentionally not
read or used as a source of claims here), and the existing
`pico_firmware/FAILSAFE_VALIDATION.md` and
`ros2_ws/src/pico_bridge/RX_FRESHNESS.md` design notes.

## Relationship to `reference/rp2040_validated/`

`reference/rp2040_validated/` is the frozen, standalone golden-reference RP2040
firmware (files such as `main.c`, `plant_schedule.h`, `ukf_model_lut.h`,
`ukf_shadow_rp2040.h`, `GAIN_SCHEDULING.md` at the top level — listed, not
read or modified for this document) tagged `rp2040-golden-v1` per the
project's migration policy. That firmware runs the *entire* control stack (UKF, gain
scheduling, PI/LADRC, pressure control) standalone on the Pico. It must never
be modified.

`pico_firmware/` (this document's subject) is architecturally different: per
the project's target architecture, the Pico here is deliberately reduced to
sensor acquisition (ADC pressure/flex, BNO055 IMUs over I2C), low-level GPIO
actuation, precise pulse timing, a local communication/watchdog safety layer,
and a text/CSV line protocol over USB CDC to a host running the real
controller (`glove_controller`) and estimator (`glove_estimator`) in ROS 2.
`pico_firmware/src/main.c` contains no PI/LADRC/UKF logic — it only reads
sensors, applies actuator commands received over the wire, and runs local
failsafe checks. Do not conflate the two: constants that happen to share
names (e.g. `PICO_HARD_PRESSURE_KPA` / `SGC_HARD_PRESSURE_KPA` = 200 kPa) live
in different codebases with different responsibilities.

## File structure and roles

- `pico_firmware/src/main.c` — firmware entry point / cooperative main loop.
  Initializes stdio over USB CDC with CRLF translation disabled to preserve
  LF-only protocol framing (`main.c:19-21`), waits `sleep_ms(1500u)` before
  sensor init to let BNO055 devices power up (`main.c:29`), then loops:
  feeds/checks the cooperative watchdog, samples ADC every 10 ms
  (`next_adc += 10000u`, `main.c:47-50`), steps the actuator state machine,
  parses incoming USB bytes line-by-line, applies accepted actuator/calibrate
  commands, steps calibration ("zero") sampling, emits sensor telemetry (`S`)
  whenever a new ADC sample was taken, and emits estimator telemetry (`E`)
  every 20 ms (`next_bno += 20000u`, `main.c:104-105`) unconditionally
  (even on IMU read failure, per `main.c:106-109` and
  `FAILSAFE_VALIDATION.md`).
- `pico_firmware/src/pico_protocol.c` — wire-frame encoders
  (`pico_protocol_encode_sensor`, `pico_protocol_encode_estimator`,
  `pico_protocol_encode_actuator`, `pico_protocol_encode_calibration`) and a
  custom exact hexadecimal float formatter (`float_hex()`,
  `pico_protocol.c:7-25`) so floats round-trip bit-exactly as text.
- `pico_firmware/src/pico_frame_parser.c` — inbound line parsing
  (`pico_frame_parse_line`), per-field validators (uint/int8/bool/float),
  CRC verification (`valid_crc()`), plus `pico_protocol_deadline_expired()`
  and `pico_protocol_sequence_accept()` helpers.
- `pico_firmware/src/pico_watchdog.c` — cooperative loop-stall detector
  (`pico_watchdog_init/feed/expired`); not a hardware watchdog.
- `pico_firmware/src/pico_actuator.c` — actuator/GPIO state machine:
  `pico_actuator_init`, `pico_actuator_arm`, `pico_actuator_accept`,
  `pico_actuator_step`, `pico_actuator_safe`, `pico_actuator_failsafe_vent`,
  `pico_actuator_watchdog_trip`, `pico_actuator_telemetry`.
- `pico_firmware/src/pico_sensors.c` — ADC pressure/flex acquisition, BNO055
  reads, EWMA filtering, and the vented pressure/flex zero-calibration
  sequence (194 lines; not exhaustively read line-by-line for this doc beyond
  its header contract in `pico_sensors.h`).
- `pico_firmware/src/pico_sensor_math.c` — small pure-math helpers:
  `pico_quaternion_normalize()`, `pico_sensors_commit_quaternion()`,
  `pico_pressure_from_vout()` (`(vout - zero_vout) * 50.0f`), and
  `pico_ewma_step()` (`previous == 0.0f ? sample : 0.8f*previous + 0.2f*sample`)
  (`pico_sensor_math.c:27-34`).
- `pico_firmware/src/crc16_ccitt.c` — `pico_crc16_ccitt_false()`, CRC16-CCITT
  (False) implementation, initial value `0xFFFF`, polynomial `0x1021`,
  MSB-first, no final XOR (`crc16_ccitt.c:3-13`).
- `pico_firmware/include/pico_board_config.h` — verified board wiring and
  tunables: `PICO_PUMP_GPIO=16`, `PICO_V1_GPIO=17`, `PICO_V2_GPIO=14`,
  `PICO_PRESSURE_GPIO=26` (ADC0), `PICO_FLEX_GPIO=28` (ADC2), BNO I2C on
  `i2c0`, SDA=GPIO4, SCL=GPIO5 at 400 kHz, BNO addresses `0x28`/`0x29`,
  `PICO_MAX_PULSE_MS=5000`, `PICO_COMMAND_TIMEOUT_US=200000` (200 ms),
  `PICO_HARD_PRESSURE_KPA=200.0f`, `PICO_ADC_AVERAGE_SAMPLES=8`,
  `PICO_PRESSURE_DIVIDER_GAIN=1.545f`, `PICO_ZERO_SAMPLES=500`,
  `PICO_ADC_VREF_V=3.3f`, `PICO_ADC_MAX_COUNTS=4095.0f`
  (`pico_board_config.h:7-31`).
- `pico_firmware/include/pico_protocol.h`, `pico_frame_parser.h`,
  `pico_watchdog.h`, `pico_actuator.h`, `pico_sensors.h` — headers for the
  above modules.
- `pico_firmware/CMakeLists.txt`, `pico_sdk_import.cmake` — Pico SDK build
  glue.
- `pico_firmware/tests/` — host-side CTest suite (`test_pico_protocol.c`,
  `test_final_firmware.c`, `test_pressure_hardware.c`, `test_main_loop.c`)
  plus fixtures/stubs; not modified or extended as part of this
  documentation task.
- `pico_firmware/src/pico_sensors.c.ndof_backup` — stray backup file,
  explicitly excluded from this document's claims per task instructions.

## Serial wire protocol

Transport: USB CDC (`stdio_usb`), LF-only text lines, `stdio_set_translate_crlf(&stdio_usb, false)`
(`main.c:21`) so `\n` is preserved rather than becoming `\r\n`. Max line
length `PICO_PROTOCOL_MAX_LINE = 512` bytes (`pico_protocol.h:8`).

General frame shape: comma-separated ASCII fields, terminated with the
decimal CRC16 value and a trailing `\n`, e.g. `<kind>,<field>,...,<crc>\n`.
Floats are encoded as C99 hexadecimal floating-point literals
(`%a`-style, hand-rolled in `float_hex()`, `pico_protocol.c:7-25`) so the
exact bit pattern (including `nan`/`inf`) survives the round trip; integers
are decimal.

CRC scheme: CRC16-CCITT (False variant) computed by
`pico_crc16_ccitt_false()` (`crc16_ccitt.c`) over every byte of the frame up
to and including the comma before the CRC field. The CRC is appended as a
decimal `uint16` (`0`–`65535`) followed by `\n` (`finish()`,
`pico_protocol.c:53-58`). Inbound: `pico_frame_parse_line()` requires
`line[length-1] == '\n'`, locates the last comma, parses the trailing decimal
CRC, and recomputes the CRC over everything before that comma
(`pico_frame_parser.c:48-58`, `:62-67`); a mismatch causes the frame to be
rejected before any field-level decoding.

Frame kinds (`PicoFrameKind`, `pico_protocol.h:63-70`), all fields decimal
integers/hex-floats unless noted:

- **`S` (sensor, PICO_FRAME_SENSOR)** — Pico → host, emitted on every new ADC
  sample (main.c). 17 comma-separated fields total including the CRC field
  and terminal comma structure (`pico_frame_parser.c:72-83`,
  `count == 17u`): `S, seq, pico_us, pressure_raw, pressure_vout,
  pressure_kpa, pressure_filtered_kpa, flex_raw, flex_filtered_raw,
  pneumatic_state(int8), pulse_active(bool 0/1), pulse_ms_remaining,
  pump_on(bool), safety_latched(bool), alarm_fail_count, watchdog_state(0-255), crc`.
- **`E` (estimator input, PICO_FRAME_ESTIMATOR)** — Pico → host, emitted
  every 20 ms regardless of IMU read success. 15 fields
  (`pico_frame_parser.c:85-95`, `count == 15u`): `E, seq, pico_us,
  flex_filtered_raw, bno1_ok(bool), bno1_quat[4] (w,x,y,z), bno2_ok(bool),
  bno2_quat[4] (w,x,y,z), crc`.
- **`A` (actuator command, PICO_FRAME_ACTUATOR)** — host → Pico. 6 fields
  (`pico_frame_parser.c:106-112`, `count == 6u`): `A, seq, command,
  pulse_ms, deadline_us, crc`, where `command` must be `<= PICO_COMMAND_ALL_OFF`
  (i.e. one of `0`=HOLD, `1`=FILL, `2`=VENT, `3`=PUMP_ON, `4`=PUMP_OFF,
  `5`=ALL_OFF, per `pico_protocol.h:14-19`).
- **`C` (calibrate request, PICO_FRAME_CALIBRATE)** — host → Pico. 3 fields
  (`pico_frame_parser.c:96-99`): `C, seq, crc`.
- **`Z` (calibration result, PICO_FRAME_CALIBRATION_RESULT)** — Pico → host.
  6 fields (`pico_frame_parser.c:100-105`): `Z, seq, success(bool),
  pressure_zero_vout, flex_zero_raw, <trailing field per encoder>, crc`
  (encoder side: `pico_protocol_encode_calibration`, `pico_protocol.c:102-111`).

Sequence/deadline helpers used by the actuator accept path
(`pico_frame_parser.c:116-124`): `pico_protocol_deadline_expired(now_us,
deadline_us)` returns true (frame stale/rejected) when `deadline_us != 0` and
`now_us` is past it (wraparound-safe signed-difference comparison);
`pico_protocol_sequence_accept(previous, have_previous, next)` rejects a
repeated sequence number equal to the last accepted one.

## Watchdog, failsafe, and timeout behavior

Two independent timeout mechanisms exist, explicitly distinguished in
`pico_watchdog.h:7-8`:

1. **Cooperative main-loop watchdog** (`pico_watchdog.c`,
   `PicoWatchdogState{last_feed_us, initialized}`): every main-loop iteration
   calls `pico_watchdog_expired()` then `pico_watchdog_feed()` (`main.c:42-45`).
   `pico_watchdog_expired()` is true if uninitialized or if elapsed time since
   the last feed exceeds `PICO_COMMAND_TIMEOUT_US` (200000 us = 200 ms — the
   same constant is reused as the watchdog timeout in `main.c:42`). This
   detects the loop itself stalling (e.g. blocked I2C); it is *not* a
   hardware interrupt-driven watchdog and cannot act during a genuine
   CPU/I2C permanent hang (`FAILSAFE_VALIDATION.md`, "Watchdog and one-way
   telemetry loss"). On expiry, `pico_actuator_watchdog_trip()` is called,
   which sets `watchdog_latched = true` and forces
   `pico_actuator_failsafe_vent()`.

2. **Host-command timeout** (`pico_actuator_step()`,
   `pico_actuator.c:119-139`): tracks `last_command_us`, the timestamp of the
   last *accepted* `A` command. If `now_us - last_command_us >
   PICO_COMMAND_TIMEOUT_US` (200 ms), `communication_lost` is latched true.

`pico_actuator_step()` failsafe condition: if pressure is invalid/non-finite,
overpressure is latched, communication is lost, the watchdog is latched, or
`safety_latched` is already true, the actuator calls
`pico_actuator_failsafe_vent()` every step and returns early
(`pico_actuator.c:128-132`).

`pico_actuator_failsafe_vent()` (`pico_actuator.c:65-73`): sets
`safety_latched = true`, `pump_on = false`, `pulse_active = false`,
`command = PICO_COMMAND_VENT`, and drives GPIO to pump OFF then both valves
open (V1=1, V2=1 — sustained VENT), per `FAILSAFE_VALIDATION.md`. This
replaced an earlier ALL_OFF-on-fault behavior; the vent is now sustained
until acknowledged.

Overpressure: `pico_actuator_step()` latches `overpressure = true`
permanently (sticky) once `pressure_kpa >= PICO_HARD_PRESSURE_KPA` (200 kPa)
while pressure is valid (`pico_actuator.c:123-124`); overpressure cannot be
cleared by any accepted command including ALL_OFF (`pico_actuator_accept()`,
`pico_actuator.c:96-105`: ALL_OFF only clears `safety_latched` /
`communication_lost` / `watchdog_latched` when `pressure_valid && !overpressure`).

Recovery / rearm: `pico_actuator_accept()` (`pico_actuator.c:81-117`) rejects
any non-off command while `overpressure`, `safety_latched`,
`communication_lost`, or invalid pressure holds. An accepted `ALL_OFF` command
can clear `safety_latched`, `communication_lost`, and `watchdog_latched`
(never `overpressure`) provided pressure is currently valid and not
overpressure; otherwise even an accepted ALL_OFF still calls
`pico_actuator_failsafe_vent()` rather than truly turning outputs off
(`pico_actuator.c:96-105`). `pico_actuator_arm()` similarly requires valid,
sub-threshold pressure to clear latches (`pico_actuator.c:47-55`), but is
only invoked from firmware internals, not directly from the wire protocol.

Pulse timing: accepted FILL/VENT commands with nonzero `pulse_ms` (capped at
`PICO_MAX_PULSE_MS = 5000`) auto-return to HOLD once
`now_us - pulse_start_us >= pulse_duration_ms * 1000` (`pico_actuator_step()`,
`pico_actuator.c:133-138`).

Calibration gating (`main.c:61-77`): a `PICO_COMMAND_PUMP_ON` command is only
processed if `sensors.calibration.calibrated`; while a zero-calibration
sequence is active (`sensors.zero_active`), only VENT, PUMP_OFF, and ALL_OFF
actuator commands are processed, all others are dropped.

### Bridge-side RX-freshness failsafe (host side, not on the Pico)

`ros2_ws/src/pico_bridge/src/pico_bridge_node.cpp` adds a second, independent
staleness gate at the host/bridge level, documented in
`ros2_ws/src/pico_bridge/RX_FRESHNESS.md`: if no valid `S`/`E`/`Z` frame has
been accepted within `rx_timeout_ms` (default 100 ms) of host time, the
bridge sets `link_up = false`, latches `rearm_required = true`, and refuses
to transmit any further `A` commands (`refresh_link()` /
`pico_bridge_node.cpp:257-264`) until an explicit ALL_OFF with zero pulse
duration is sent over a fresh link. This is layered in front of, not a
replacement for, the Pico's own 200 ms command timeout: end-to-end nominal
worst case for full local VENT is roughly 100 ms (bridge RX staleness) + 200 ms
(Pico command timeout) plus transport/execution latency, per
`RX_FRESHNESS.md`, "Required chain and timing".

## Summary table of key timing/safety constants

| Constant | Value | Source |
|---|---|---|
| `PICO_COMMAND_TIMEOUT_US` | 200000 us (200 ms) | `pico_board_config.h:25` |
| `PICO_HARD_PRESSURE_KPA` | 200.0 kPa | `pico_board_config.h:26` |
| `PICO_MAX_PULSE_MS` | 5000 ms | `pico_board_config.h:24` |
| `PICO_ZERO_SAMPLES` | 500 | `pico_board_config.h:29` |
| ADC sample period (main loop) | 10 ms | `main.c:48` |
| BNO/estimator frame period | 20 ms | `main.c:105` |
| BNO startup delay | 1500 ms | `main.c:29` |
| Bridge `rx_timeout_ms` default | 100 ms | `pico_bridge_node.cpp:130`, `RX_FRESHNESS.md` |
| Bridge `reconnect_backoff_ms` default | 250 ms | `pico_bridge_node.cpp:129` |
