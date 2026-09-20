# RP2040: BNO telemetry and sustained failsafe vent

## Scope and changes

No PI/LADRC/UKF code, ROS controller behavior, sensor math, wire encoder/parser,
CRC, calibration math, or GPIO mapping was changed. The BNO startup delay remains
`sleep_ms(1500u)` before sensor initialization. No hardware was flashed.

Production changes:

- `src/main.c`: serialize E at every 20 ms BNO deadline regardless of read success.
  The existing sensor function still supplies actual validity flags, last-good
  normalized quaternions (identity before any good read), FLEX, sequence and time.
  Calibration completion calls normal ALL_OFF only when no fault is latched;
  aborting calibration during a fault cannot close the vent valves.
- `src/pico_actuator.c`, `include/pico_actuator.h`: explicit
  `pico_actuator_failsafe_vent` sets pump OFF, V1=1, V2=1, command VENT,
  pulse_active=false and safety_latched=true. Timeout, watchdog trip, invalid
  pressure and overpressure use this function. Subsequent steps maintain VENT
  even if pressure becomes valid again. FILL/HOLD/PUMP_ON remain rejected.
  PUMP_OFF cannot cancel the vent or clear the latch. ALL_OFF acknowledges a
  link/watchdog/invalid-pressure fault only with valid pressure and no latched
  overpressure; otherwise it preserves VENT even within the command handler.
  Successful acknowledgement enters normal ALL_OFF, never resumes a pulse or pump.
  Overpressure remains sticky until reset, as before (even after pressure falls).
- `include/pico_watchdog.h`: clarify that the unchanged watchdog implementation
  monitors elapsed main-loop time; host loss uses last_command_us separately.
- `include/pico_board_config.h`: permit a build-time output-enable override for
  host tests. Production default remains 1; GPIO 16/17/14, 200 ms timeout and
  200 kPa threshold are unchanged.

Test/build changes:

- `tests/CMakeLists.txt`: explicitly disable physical GPIO for host builds and
  register `test_main_loop`; GPIO tests link spies against the actual actuator code.
- `tests/test_final_firmware.c`: expect emergency VENT instead of ALL_OFF.
- `tests/test_pressure_hardware.c`: T1-T5, GPIO assertions, pulse cancellation,
  timeout boundary and counter wraparound, invalid-pressure acknowledgement,
  sticky overpressure, no auto-resume, and mock BNO register responses.
- `tests/test_main_loop.c`: execute production main and sensors with simulated
  time/USB/I2C; T6/T7, initial identity and last-good retention, sequence/time/FLEX,
  1500 ms delay, and calibration abort during invalid pressure.
- `tests/stubs/pico/stdio.h`, `tests/stubs/pico/stdio_usb.h`: host USB declarations.
- `tests/fixtures/estimator-valid.txt`, `tests/fixtures/estimator-invalid.txt`:
  exact E bytes shared by main-loop and bridge tests, including CRC and LF.
- `../ros2_ws/src/pico_bridge/test/test_pico_bridge_node.cpp` and its package
  `CMakeLists.txt`: decode those fixtures with the existing bridge implementation;
  verify suppressed commands after RX stale, including the real serial path via
  a PTY. The subsequent RX-freshness follow-up changes the bridge gate, documented below.

## Physical behavior

| Event | Local outcome |
| --- | --- |
| USB disconnected, Pico and valve power retained | After more than 200 ms since the last accepted command, the next actuator step switches pump OFF and V1=V2=1; fault stays latched. |
| No accepted commands for >200 ms | Same sustained VENT. Exactly 200 ms does not yet trip, preserving the existing comparison. Rejected commands do not refresh the timer. |
| Pressure >=200 kPa | Pump OFF and sustained VENT, overpressure and safety latched; PUMP_ON and ALL_OFF cannot rearm it. |
| Pressure invalid/nonfinite | Pump OFF and sustained VENT; restored validity alone does not rearm. |
| Both BNO invalid | E continues with both flags zero, retained quaternions, FLEX, sequence and timestamp; BNO failure alone does not introduce a new pneumatic trip. |
| Watchdog trip | Pump OFF and sustained VENT, watchdog and safety latched. |

Normal ALL_OFF remains pump=0/V1=0/V2=0. An accepted explicit ALL_OFF can rearm
communication/watchdog/invalid-pressure faults once pressure is valid and no
overpressure is latched. New commands are then needed to energize the pump.
Startup still initializes GPIO low before the 1500 ms delay; the first safety
step maintains the startup latch in VENT until the host acknowledges it.

These are software/GPIO results, not a measured hardware decompression result.
If unplugging USB also removes electrical power, firmware cannot energize VENT.
The existing ADC validity criteria are unchanged: a disconnected analog sensor
that produces plausible ADC values is not necessarily detected as invalid.

## Watchdog and one-way telemetry loss

The watchdog is cooperative: main checks elapsed time since its last feed and
then feeds it. It trips when execution resumes after a delay. It is not a
hardware interrupt/watchdog capable of switching valves during a permanent
CPU/I2C/main-loop hang. This separation was preserved, not silently redesigned.

The Pico command timeout cannot detect missing Pico-to-host telemetry while
accepted host commands keep arriving.

The RX-freshness follow-up now closes the one-way telemetry gap in the bridge.
`refresh_link()` requires accepted S/E/Z within 100 ms (startup parameter
`rx_timeout_ms`), checked at the timer and before actuator TX. Opening/reopening
a port does not renew freshness. Stale RX sets link_up=false and blocks all
actuator TX, regardless of continuing ROS commands or reconnect backoff.
Valid RX recovery requires explicit host ALL_OFF before resuming new actuation;
no automatic ALL_OFF is emitted by the bridge destructor.

The chain is: telemetry stops -> bridge stops A TX at 100 ms RX age -> Pico's
unchanged >200 ms command timeout -> pump OFF + sustained VENT + latch.
The end-to-end nominal bound is about 300 ms plus execution/transport latency,
not 200 ms from the last telemetry frame. E with invalid IMUs is valid RX traffic.

See `../ros2_ws/src/pico_bridge/RX_FRESHNESS.md` for the exact gate, recovery
semantics, deterministic B1-B5 PTY/Pico tests, artifacts and patch for this
follow-up. All firmware source and protocol contracts in this report remain
unchanged by that follow-up.

## Validation and artifacts

Debug host build: `/tmp/stage6-failsafe-tests`.
Four tests pass: test_pico_protocol, test_final_firmware,
test_pressure_hardware, test_main_loop. These include all T1-T7 requirements.
Bridge build: `/tmp/stage6-failsafe-ros`; transport regression, deterministic RX/Pico safety integration, and five lint checks (7/7 PASS).
RP2040 Release build: `/tmp/stage6-failsafe-build/pico_firmware.uf2`.
SDK: `<path-to-pico-sdk>`; compiler: arm-none-eabi-gcc.
No build/flash was performed against connected hardware.

Reproduction:

```sh
cmake -S pico_firmware/tests -B /tmp/stage6-failsafe-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/stage6-failsafe-tests -j4
ctest --test-dir /tmp/stage6-failsafe-tests --output-on-failure
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/stage6-failsafe-ros/log build --base-paths ros2_ws/src/glove_interfaces ros2_ws/src/pico_bridge --build-base /tmp/stage6-failsafe-ros/build --install-base /tmp/stage6-failsafe-ros/install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source /tmp/stage6-failsafe-ros/install/setup.bash
ctest --test-dir /tmp/stage6-failsafe-ros/build/pico_bridge --output-on-failure
cmake -S pico_firmware -B /tmp/stage6-failsafe-build -DPICO_SDK_PATH=<path-to-pico-sdk> -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/stage6-failsafe-build -j4
```

## Deviation and diff provenance

Intentional deviations are unconditional diagnostic E delivery, sustained VENT
instead of ALL_OFF for emergencies, maintaining VENT while latched (including
startup safety steps), and preventing ALL_OFF/calibration abort from prematurely
closing emergency vents. Normal calibration and normal ALL_OFF remain unchanged.
The validated reference directory was not edited. This is not a new complete
algorithm-equivalence certification against the validated firmware.

Git status/diff are unavailable; metadata was not repaired. The pre-edit copy is
`/tmp/stage6-failsafe-before`; the reviewable unified patch is
`/tmp/stage6-failsafe.patch`, relative to that snapshot (not an inferred Git HEAD).
