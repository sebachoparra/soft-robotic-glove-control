# Stage 6 pressure-only bring-up

The physical output gate remains `PICO_ACTUATOR_OUTPUT_ENABLE=0`. No firmware has
been flashed as part of this change. GPIO tests use host spies. With the gate at
zero, actuator telemetry describes the commanded state; the GPIO backend is disabled.

Only `CTRL_PRESSURE` mode entry has been connected to PUMP_ON in this change.
PI position, UKF and ADRC hardware validation are outside this task.

## Interface

`/glove_control/control_command` (`glove_interfaces/msg/ControlCommand`):

- `kind: 1, mode: 3` enters CTRL_PRESSURE and reseeds P_ref from current filtered
  pressure, as in the validated firmware. It sends PUMP_ON followed by HOLD.
- `kind: 5, pressure_ref_kpa: 20.0` is SET_P. Wait for the pressure mode status
  before sending it. Only finite values in [0, 170] are accepted while already in
  pressure mode; invalid requests leave P_ref unchanged. Mode re-entry reseeds P_ref.
- `kind: 1, mode: 0` ends control and sends ALL_OFF.

Wire actuator commands remain `A,seq,command,pulse_ms,deadline_us,crc\n`:
0 HOLD, 1 FILL, 2 VENT, **3 PUMP_ON**, 4 PUMP_OFF, 5 ALL_OFF.
Pump commands do not change valve positions. Timed FILL/VENT returns to HOLD.
ALL_OFF stops the pulse and drives GP16/GP17/GP14 low. Oversized pulse requests
are rejected before changing state. The 200 ms communication timeout is unchanged.

Local trips require explicit acknowledgement: once pressure is valid, a fresh
ALL_OFF can clear a communication/invalid-pressure/watchdog latch. Overpressure
is terminal for that Pico instance. Incoming normal commands cannot clear a latch.
PUMP_ON is rejected until zero calibration succeeds. Calibration is RAM-only and
must be repeated after reboot.

## Vented pressure-zero primitive

Previously the ROS calibration service was a stub, its Pico averaging helper was
not reachable, the divider gain was missing, and the pressure transfer formula
differed from the validated firmware. The conversion now is exactly:

```
Vout = (raw * 3.3 / 4095) * 1.545
P_kPa = (Vout - zero_vout) * 50
```

The existing `/pico_bridge/calibrate_sensors` service now requests **500 ADC samples
per channel** from the Pico. Collection is incremental so command handling, sensor
streaming and local safety continue. Pressure and flex zeros commit together only
on success; failure preserves previous calibration. Pressure and flex filters are
reseeded. The Pico ends collection with ALL_OFF. No UKF reference calibration or
full experiment-calibration sequence is added here.

Preparation, with the controller in NONE (or stopped):

1. Ensure valid pressure telemetry. Send fresh ALL_OFF (`command: 5`) to acknowledge
   the boot/link latch. Never acknowledge an overpressure trip to resume a run.
2. With pump OFF, command VENT (`command: 2, pulse_ms: 0`) and allow the validated
   **10 seconds** for venting. Refresh VENT every 50 ms with a **different sequence
   number each time**, including during the service request. This maintains the
   unchanged 200 ms command timeout; repeated identical sequence numbers do not.
3. Call `/pico_bridge/calibrate_sensors` with the empty request. The service returns
   success only after the matching CRC-checked Pico response. It rejects concurrent
   requests, reports disconnects and times out after 2 seconds. The Pico requires
   pump OFF, VENT, no pulse and no safety latch throughout collection. The operator
   must verify the line is actually vented; valve command state alone cannot prove it.
4. Stop VENT refresh, send fresh ALL_OFF, verify service success and near-zero
   pressure telemetry. A roughly 10 kPa idle offset must not be ignored.

The calibration wire extension is `C,request_seq,crc\n`, with response
`Z,request_seq,success,pressure_zero_vout,flex_zero_raw,crc\n`.
Floats use the existing hexadecimal encoding and CRC16-CCITT-FALSE convention.

## Requested dry-run sequence

Use the rebuilt message definitions, bridge, controller and Pico firmware together.
The currently flashed firmware cannot implement these new commands; flashing is
explicitly deferred for review. Host unit/integration tests exercise the new code
without connecting to hardware.

After successful vented calibration, use one already-discovered ROS publisher for
actuator commands and one for control commands. Send a fresh ALL_OFF acknowledgement
immediately before CTRL_PRESSURE (within 200 ms); separate `ros2 topic pub --once`
process startup delays can exceed that interval. Do not run a second actuator
publisher or a VENT refresher during pressure control.

1. `CTRL_PRESSURE`: `kind: 1, mode: 3`.
2. Verify controller mode 3, control enabled, P_ref initialized to current pressure,
   and Pico telemetry showing pump ON without a safety latch.
3. `SET_P 20`: `kind: 5, pressure_ref_kpa: 20.0`.
4. `SET_P 30`: `kind: 5, pressure_ref_kpa: 30.0`.
5. `SET_P 20`: `kind: 5, pressure_ref_kpa: 20.0`.
6. End with CTRL_NONE and verify pump OFF, pulse inactive and pneumatic state OFF.

With GPIO disabled this validates command/state flow, not a physical pressure
response. After explicit approval to enable and flash, physical testing still must
verify sensor zero, wiring levels, actual pressure response, pulse durations and
watchdog shutdown on the device. No automatic local-trip recovery is part of this
sequence.

## Reproducible validation

```
cmake -S pico_firmware/tests -B /tmp/stage6-pressure-validation/pico-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/stage6-pressure-validation/pico-tests
ctest --test-dir /tmp/stage6-pressure-validation/pico-tests --output-on-failure
```

ROS was built with Debug assertions enabled, from `ros2_ws/src`, into
`/tmp/stage6-pressure-validation/{build,install}`. Full `colcon test` used isolated
`ROS_DOMAIN_ID=176`. Stage 1–5 differential results and the disabled-output Pico
cross-build are recorded in `/tmp/stage6-pressure-validation`.
