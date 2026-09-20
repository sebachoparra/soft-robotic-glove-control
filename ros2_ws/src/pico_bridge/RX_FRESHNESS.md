# Stage 6: fail-safe for one-way Pico telemetry loss

## RX health and TX gate

`src/pico_bridge_node.cpp::refresh_link()` compares monotonic host time with
`last_rx_`. At age >= `rx_timeout_ms`, it sets `link_up=false` and latches
`rearm_required=true`. Default `rx_timeout_ms` is **100 ms** (previously 200 ms),
about ten SensorFrame periods at 100 Hz or five EstimatorFrame periods at 50 Hz.
The parameter is read at startup and must be positive; changing the configured
threshold changes detection latency. The Pico timeout remains **200000 us**.

`valid_rx()` is the only writer of `last_rx_`. It runs only after the existing
CRC, framing, field-count and numeric decoder accepts S, E or Z. Either S or E
alone is sufficient. A syntactically valid E with both IMU flags false is still
valid transport traffic. Corrupt/rejected frames and arbitrary bytes do not
refresh RX. Sequence handling remains the existing protocol behavior.

`refresh_link()` runs in the 10 ms polling callback, immediately before handling
an actuator command, and again in `send_line()`. Thus a late timer or continuing
ROS command callbacks cannot keep actuator TX alive. RX processing also expires
the old timestamp before renewing it, so a valid frame arriving before the stale
timer cannot bypass the recovery latch.

An open serial port no longer implies a healthy link. Startup and reconnect
leave link_up=false until valid RX arrives. Stale RX leaves the port open for
reading, but blocks **all A commands**, including ALL_OFF, PUMP_OFF and periodic
VENT. Actual serial errors close the port and use the existing reconnect policy.
The safety property no longer depends on reconnect_backoff_ms exceeding 200 ms.

## Recovery

Valid RX restores link health, not permission to resume old actuation. The bridge
keeps `rearm_required` latched and drops actuator commands until the host submits
an explicit ALL_OFF with zero pulse duration over a fresh link. Only a complete
successful serial write of that acknowledgement clears the bridge gate. The Pico
remains authoritative: its existing sequence, deadline, valid-pressure and
sticky-overpressure rules decide whether the command actually rearms hardware.
If the Pico rejects the acknowledgement, its safety latch continues to block
energizing commands. No wire ACK or new protocol is introduced.

The bridge has no saved command/setpoint to replay. It does not generate recovery
commands. Its former destructor-generated ALL_OFF was removed because ALL_OFF
is a rearm acknowledgement and would close a failsafe vent. Shutdown now stops
TX and lets the Pico's local timeout execute the emergency action.

The gate also starts latched at startup, consistent with the existing Pico
startup requirement for ALL_OFF. New host actuator commands are required after
acknowledgement; acknowledging alone never turns the pump on.

## Required chain and timing

Pico telemetry stops -> after 100 ms without an accepted Pico frame, bridge
stops all actuator TX and reports link_up=false -> once more than 200 ms have
elapsed since the Pico last accepted an actuator command, its local actuator
step sets communication_lost=true, safety_latched=true, pump OFF, pulse inactive,
command VENT, GP16=0, GP17=1 and GP14=1.

The thresholds are sequential, not a 200 ms end-to-end guarantee. Nominal worst
case is approximately 100+200 ms from last valid telemetry, plus transport and
execution delays. Bytes already handed to the OS/USB before stale detection can
still be in flight. The existing cooperative Pico loop/watchdog and maintained
power are prerequisites; no hardware watchdog redesign is included.

## Deterministic validation

`test/test_rx_failsafe.cpp` injects a monotonic virtual clock (no wall-clock sleeps
for safety timing). A PTY exercises the real serial write/read path, connecting
the actual bridge to the actual Pico CRC/parser/actuator C implementation with
host GPIO spies. Only hardware/time are simulated; the fail-safe state machine
is not duplicated in the test.

- B1: valid RX and active pump/FILL, then continued ROS commands with zero Pico
  RX; the command callback blocks all commands exactly at 100 ms, without waiting
  for the timer. Actual serial received-command counts remain unchanged.
- B2: after continued blocked commands, pump remains active at exactly 200 ms
  since last accepted A and switches to latched VENT at 200 ms + 1 us; GPIO,
  communication_lost, safety_latched, pump, command and pulse are asserted.
- B3: RX recovery restores health but not actuation; all commands remain blocked
  until ALL_OFF. It leaves pump OFF/pulses cancelled. Only a subsequent new
  PUMP_ON energizes. Reopen with zero backoff cannot create fresh RX or rearm.
- B4: combined S and invalid-IMU E, S-only, and invalid-IMU E-only traffic each
  maintain link health over multiple freshness periods.
- B5: bad CRC, CRC-valid incomplete E and unknown frames cannot renew freshness.
- Additional cases: no RX at startup blocks ALL_OFF; a valid RX callback before
  the stale timer still latches recovery; destruction cannot emit an implicit
  acknowledgement. Existing bridge codec/calibration/publication tests remain.

Results: bridge CTest **7/7 PASS** (two executable tests plus cppcheck, cpplint,
lint_cmake, uncrustify and xmllint); firmware CTest **4/4 PASS**, preserving T1-T7.
No PI/LADRC/UKF, firmware source, GPIO, protocol/CRC/calibration math or 1500 ms
BNO startup delay was modified. No hardware was flashed.

Build artifacts:

- Bridge: `/tmp/stage6-failsafe-ros/build/pico_bridge/pico_bridge_node`.
- Firmware: `/tmp/stage6-failsafe-build/pico_firmware.uf2`.
- UF2 SHA256: `227eff0394ed394f8f8e95e5006e9c5ca1fa22f2017245d665c8631544806cd4`
  (identical to the previous firmware artifact).
- Test log: `/tmp/stage6-rx-tests.log`.
- Full diff for this follow-up: `/tmp/stage6-rx-freshness.patch`.
- Pre-edit snapshot: `/tmp/stage6-rx-before`.

Git metadata still references a missing worktree directory, so the patch is
against the pre-edit snapshot of this task, not an inferred Git HEAD.

Reproduce from the repository root:

```sh
source /tmp/stage6-failsafe-ros/install/setup.bash
cmake -S ros2_ws/src/pico_bridge -B /tmp/stage6-failsafe-ros/build/pico_bridge -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/stage6-failsafe-ros/build/pico_bridge -j4
ctest --test-dir /tmp/stage6-failsafe-ros/build/pico_bridge --output-on-failure
cmake --build /tmp/stage6-failsafe-tests -j4
ctest --test-dir /tmp/stage6-failsafe-tests --output-on-failure
cmake --build /tmp/stage6-failsafe-build -j4
```
