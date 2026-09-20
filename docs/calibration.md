# Calibration: Pressure Zero, FLEX Zero, and UKF Reference

Sources: `tools/pressure/calibrate_pressure.py`, `tools/pressure/vent_to_zero.py`,
`tools/pressure/run_calibration.sh`, `glove_interfaces/srv/CalibrateSensors.srv`,
`glove_interfaces/srv/CalibrateUkfReference.srv`, `docs/PRESSURE_BRINGUP.md`,
`pico_firmware/src/pico_sensors.c`, `pico_firmware/include/pico_sensors.h`,
`pico_firmware/src/pico_protocol.c`, `pico_firmware/src/pico_frame_parser.c`,
`ros2_ws/src/glove_estimator/src/ukf_node.cpp`.

## 1. Pressure and FLEX zero calibration (`/pico_bridge/calibrate_sensors`)

Pressure zero and FLEX zero are calibrated **together**, in one Pico-side averaging
pass, exposed through the same ROS 2 service.

### 1.1 Service contract

`glove_interfaces/srv/CalibrateSensors.srv`:
```
---
bool success
float32 pressure_zero_vout
float32 flex_zero_raw
string message
```
(empty request). Response fields directly correspond to the two calibration constants
stored on the Pico.

### 1.2 Pico-side implementation

`pico_sensors_zero_begin()` / `pico_sensors_zero_step()`
(`pico_firmware/src/pico_sensors.c:150-192`):

- `pico_sensors_zero_begin()` refuses to start unless outputs are vented
  (`vented_outputs`) and pressure telemetry is currently valid
  (`pico_firmware/src/pico_sensors.c:152-158`).
- `pico_sensors_zero_step()` is called repeatedly; each call (throttled to a ~200 µs
  cadence via `zero_next_us`, `pico_firmware/src/pico_sensors.c:169`) reads one raw ADC
  sample from the pressure channel and one from the FLEX channel
  (`pico_firmware/src/pico_sensors.c:170-176`), accumulating sums until
  `PICO_ZERO_SAMPLES` samples have been collected (`pico_firmware/src/pico_sensors.c:181`).
  If venting or pressure validity is lost mid-collection, the pass aborts
  (`pico_firmware/src/pico_sensors.c:165-168`).
- On completion, the averaged raw pressure is converted to volts and scaled by the
  divider gain (`p = (p_raw*PICO_ADC_VREF_V/PICO_ADC_MAX_COUNTS)*PICO_PRESSURE_DIVIDER_GAIN`,
  `pico_firmware/src/pico_sensors.c:184`) — the same
  `Vout = (raw*3.3/4095)*1.545` transfer function documented in
  `docs/PRESSURE_BRINGUP.md:39-41` — and the averaged FLEX raw count is used directly
  (`pico_firmware/src/pico_sensors.c:185`). Both new zeros commit together only on
  success via `pico_sensors_calibrate()`
  (`pico_firmware/src/pico_sensors.c:186`); failure preserves the previous calibration
  (per `docs/PRESSURE_BRINGUP.md:47-48`).
- Wire protocol: request `C,request_seq,crc\n`, response
  `Z,request_seq,success,pressure_zero_vout,flex_zero_raw,crc\n`
  (`docs/PRESSURE_BRINGUP.md:67-69`; parsed on the Pico side by
  `pico_frame_parser.c:104` and serialized by `pico_protocol.c:108`).
- Per `docs/PRESSURE_BRINGUP.md:44-49`, the service requests **500 ADC samples per
  channel**; collection is incremental (command handling, sensor streaming, and local
  safety continue during collection); pressure and flex filters are reseeded on
  success; the Pico ends collection with `ALL_OFF`.

### 1.3 Required preparation sequence

Per `docs/PRESSURE_BRINGUP.md:51-65` (controller must be in `NONE`/stopped):

1. Ensure valid pressure telemetry; send a fresh `ALL_OFF` (`command: 5`) to
   acknowledge the boot/link latch. Never acknowledge an overpressure trip to resume a
   run.
2. With pump OFF, command `VENT` (`command: 2, pulse_ms: 0`) and hold for the
   validated **10 seconds**, refreshing VENT every 50 ms with a **different sequence
   number each time** (the Pico enforces a 200 ms command timeout, so identical
   repeated sequence numbers do not keep the link alive).
3. Call `/pico_bridge/calibrate_sensors` with an empty request. It returns success
   only after a matching CRC-checked Pico response; it rejects concurrent requests,
   reports disconnects, and times out after 2 seconds. The Pico requires pump OFF,
   VENT, no pulse, and no safety latch throughout collection. The operator must
   independently verify the line is actually vented — valve command state alone does
   not prove it.
4. Stop VENT refresh, send a fresh `ALL_OFF`, and verify service success and
   near-zero pressure telemetry (a ~10 kPa idle offset should not be ignored, per
   `docs/PRESSURE_BRINGUP.md:65`).

### 1.4 FLEX calibration

No separate FLEX-only calibration workflow/tool was found in `tools/`, `pico_firmware/`,
or `reference/`. FLEX zeroing is folded into the same `calibrate_sensors` /
`pico_sensors_zero_step()` pass described in §1.2 above (`flex_zero_raw` is averaged and
committed alongside `pressure_zero_vout` in the same call). **Not specified in source as
a standalone workflow.** Note also that in the UKF4 reference model, FLEX is explicitly
*not* re-zeroed per run at the estimator level — the UKF state `bF` tracks slow FLEX
offset drift instead (`reference/rp2040_validated/ukf_shadow_rp2040.h:47-48`; see
`docs/sensing_and_ukf.md` §1.1). The `flex_zero_raw` calibrated here is a separate,
lower-level ADC-to-position zero used by `sgc_flex_raw_to_position_unclipped()`
(`ros2_ws/src/soft_glove_core/src/soft_glove_core.c:284-289`,
`FLEX_SPAN_COUNTS = 494.794f`), not the UKF bias state.

## 2. UKF reference calibration (`/pico_bridge/calibrate_ukf_reference`... served by `ukf_node`)

Despite the task's service-name framing, this service is actually hosted by the
`ukf_node` (ROS 2 estimator), not `pico_bridge`, per
`ros2_ws/src/glove_estimator/src/ukf_node.cpp:30-37`: it is advertised as
`~/calibrate_ukf_reference` on the `ukf_node`.

### 2.1 Service contract

`glove_interfaces/srv/CalibrateUkfReference.srv`:
```
---
bool success
uint32 good_samples
float32[4] q_rel0
string message
```
(empty request).

### 2.2 Behavior

`UkfNode::on_calibrate()` (`ros2_ws/src/glove_estimator/src/ukf_node.cpp:112-137`):

- Requires that at least `ref_samples_` (ROS parameter `ref_samples`, default 40 —
  matching the reference firmware's `UKF_SHADOW_REF_SAMPLES = 40u`,
  `reference/rp2040_validated/ukf_shadow_rp2040.h:71`) BNO-quaternion samples have
  already been buffered from incoming `EstimatorInputFrame` messages
  (`ukf_node.cpp:60-65,116-119`); otherwise it fails with "Insufficient estimator input
  frames".
- Counts `good_samples` as those where both BNO1 and BNO2 report OK
  (`ukf_node.cpp:121-125`).
- Calls `sgc_ukf_accumulate_reference()` with the buffered samples
  (`ukf_node.cpp:126-127`) to compute the relative reference quaternion `q_rel0`,
  returned in the response.
- On success, resets the estimator's update-tick bookkeeping so the next UKF update
  uses a nominal `dt` rather than one spanning the calibration gap
  (`compose_.last_update_us = 0u; have_update_tick_ = false;`, `ukf_node.cpp:129-131`,
  commented "The golden reset makes the first subsequent update use nominal dt").

## 3. Running the calibration tools

### 3.1 `tools/pressure/vent_to_zero.py`

Standalone ROS 2 node. Waits (5 s deadline) for a `SensorFrame` on
`/pico_bridge/sensor_frame`, sends a fresh `ALL_OFF` (`command 5`), then drives `VENT`
(`command 2`) every 50 ms for **30 seconds** while printing live pressure/state
telemetry, then sends a final `ALL_OFF`
(`tools/pressure/vent_to_zero.py:48-108`). Intended as a pre-check / manual vent, not
itself calling any calibration service.

### 3.2 `tools/pressure/calibrate_pressure.py`

Standalone ROS 2 node (`Calibrator`). Waits for a `SensorFrame` (5 s deadline), sends
`ALL_OFF`, then vents (`command 2`) every 50 ms for **10 seconds** as call preparation,
then calls `/pico_bridge/calibrate_sensors` (empty request) while continuing to refresh
`VENT` every 50 ms until the future resolves (3 s timeout), prints
`success`/`pressure_zero_vout`/`flex_zero_raw`/`message`, and finishes with a final
`ALL_OFF` (`tools/pressure/calibrate_pressure.py:9-131`). This script performs the full
§1.3 preparation-and-call sequence in one process.

### 3.3 `tools/pressure/run_calibration.sh`

Orchestrates both tools plus a final check (`tools/pressure/run_calibration.sh:1-30`):

```bash
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"

python3 tools/pressure/vent_to_zero.py          # 1. VENT 30 s
python3 tools/pressure/calibrate_pressure.py    # 2. PRESSURE ZERO CALIBRATION
ros2 topic echo /pico_bridge/sensor_frame --once  # 3. FINAL SENSOR CHECK
```

Run it from the workspace with the Pico bridge, controller, and Pico firmware already
running and connected (it only publishes commands and calls the calibration service —
it does not launch any nodes itself). It assumes an already-sourced/built
`install/setup.bash` under the workspace root two levels above the script
(`$WORKSPACE_ROOT = tools/pressure/../..`, i.e. the repository root's `install/`).

## 4. Not specified in inspected source

- No dedicated `calibrate_flex.py` or similarly named standalone FLEX-only calibration
  tool was found; FLEX zeroing is bundled into `calibrate_sensors` as described in §1.4.
- The exact `PICO_ZERO_SAMPLES` constant value was not read in this pass (only its use
  site); `docs/PRESSURE_BRINGUP.md` states 500 samples per channel are requested by the
  ROS service.
