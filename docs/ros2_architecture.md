# ROS 2 Architecture (Jazzy)

This document describes the current ROS 2 workspace at `ros2_ws/src/`, covering
five packages: `glove_interfaces`, `soft_glove_core`, `pico_bridge`,
`glove_estimator`, and `glove_controller`. All topic names, message/service
types, and field lists below were taken directly from source: node `.cpp`
files (`create_publisher` / `create_subscription` / `create_service` calls),
`.msg`/`.srv` files, `package.xml`, and `CMakeLists.txt` for each package.

## Package: glove_interfaces

`ros2_ws/src/glove_interfaces/` — pure interface package (`rosidl_generate_interfaces`,
see `ros2_ws/src/glove_interfaces/CMakeLists.txt`). It has no nodes; it only
defines the messages and services used by the other four packages. It depends
on `std_msgs` and `builtin_interfaces` (`ros2_ws/src/glove_interfaces/package.xml`).

### Messages (`ros2_ws/src/glove_interfaces/msg/`)

- **ActuatorCommand.msg**: `std_msgs/Header header`, `uint8 command`,
  `uint32 pulse_ms`, `uint32 seq`, `uint32 deadline_us`.
- **ControlCommand.msg**: `std_msgs/Header header`, `uint8 SET_P=5` (constant),
  `uint8 kind`, `float32 q_ref_pct`, `uint8 mode`, `bool enable`,
  `float32 pressure_ref_kpa`.
- **ControllerStatus.msg**: `std_msgs/Header header`, `uint8 controller_mode`,
  `bool control_enabled`, `float32 position_ref_pct`, `float32 pressure_ref_kpa`,
  `float32 active_kp`, `float32 active_ki`, `float32 active_b0`,
  `float32 eso_z1_pct`, `float32 eso_z2_pct_s`, `float32 ff_gamma`,
  `int8 pneumatic_state`, `uint32 last_pulse_ms`, `bool rate_limited`,
  `bool saturated`, `bool ran_sample`, `bool ran_ukf`, `bool ran_eso`,
  `bool ran_outer`, `bool ran_pressure`, `bool ran_log`,
  `uint32 estimator_source_seq`, `uint32 estimator_acquisition_age_us`,
  `uint32 estimator_pipeline_latency_us`, `bool estimator_seq_repeat`,
  `bool abort_active`, `bool overpressure_latched`, `bool link_up`,
  `bool estimator_stale`, `bool sensor_stale`.
- **EstimatorInputFrame.msg**: `std_msgs/Header header`, `uint32 pico_timestamp_us`,
  `uint32 seq`, `float32 flex_filtered_raw`, `bool bno1_ok`,
  `float32[4] bno1_quat`, `bool bno2_ok`, `float32[4] bno2_quat`.
- **EstimatorState.msg**: `std_msgs/Header header`, `float32 s_hat`,
  `float32 v_hat`, `float32 eta_hat`, `float32 b_f_hat`, `float32 theta_deg`,
  `float32 innovation_flex`, `float32 innovation_theta`, `float32 nis`,
  `float32 sigma_s`, `float32 sigma_v`, `float32 sigma_eta`, `float32 sigma_b_f`,
  `float32 rho_eta_b_f`, `float32 dt_used`, `bool valid`, `bool reference_valid`,
  `bool theta_valid`, `bool bno1_ok`, `bool bno2_ok`, `uint32 source_seq`,
  `uint32 source_timestamp_us`, `builtin_interfaces/Time estimator_stamp`.
- **PicoStatus.msg**: `std_msgs/Header header`, `bool link_up`, `uint32 rx_frames`,
  `uint32 tx_frames`, `uint32 crc_errors`, `uint32 framing_errors`,
  `uint32 seq_gaps`, `uint32 last_rx_pico_us`, `uint32 reconnect_count`.
- **SensorFrame.msg**: `std_msgs/Header header`, `uint32 pico_timestamp_us`,
  `uint32 seq`, `float32 pressure_raw`, `float32 pressure_vout`,
  `float32 pressure_kpa`, `float32 pressure_filtered_kpa`, `float32 flex_raw`,
  `float32 flex_filtered_raw`, `int8 pneumatic_state`, `bool pulse_active`,
  `uint32 pulse_ms_remaining`, `bool pump_on`, `bool safety_latched`,
  `uint32 pulse_alarm_fail_count`, `uint8 watchdog_state`.

### Services (`ros2_ws/src/glove_interfaces/srv/`)

- **CalibrateSensors.srv**: request has no fields; response:
  `bool success`, `float32 pressure_zero_vout`, `float32 flex_zero_raw`,
  `string message`.
- **CalibrateUkfReference.srv**: request has no fields; response:
  `bool success`, `uint32 good_samples`, `float32[4] q_rel0`, `string message`.
- **SelectMode.srv**: request `uint8 controller_type`, `uint8 feedback_source`;
  response `bool success`, `string message`. Not specified in source as
  currently served or called: no `create_service<glove_interfaces::srv::SelectMode>`
  or client call was found in any `.cpp` under `ros2_ws/src` (grep of
  `SelectMode` across the workspace only matches the `.srv` file itself), so
  this service appears defined but not yet wired into any node.

## Package: soft_glove_core

`ros2_ws/src/soft_glove_core/` — a ROS-free static C library (`project(soft_glove_core LANGUAGES C)`,
`ros2_ws/src/soft_glove_core/CMakeLists.txt`) built from
`src/soft_glove_core.c`, `src/soft_glove_core_ukf.c`, `src/soft_glove_core_ctrl.c`,
`src/soft_glove_core_sched.c`, `src/soft_glove_core_ukf_compose.c`, exporting
headers under `include/soft_glove_core/`. It has no ROS nodes and no
topics/services of its own; it is the transcription of the validated
control/estimation logic (UKF, gain scheduling, PI/LADRC, pneumatic
supervisor) that `glove_controller` and `glove_estimator` link against
(`target_link_libraries(... soft_glove_core::soft_glove_core)` in both
consumer CMakeLists). Key public API used by the nodes, from
`ros2_ws/src/soft_glove_core/include/soft_glove_core/soft_glove_core_sched.h`:
`sgc_sup_init`, `sgc_ctrl_state_init` (used in `glove_control_node.cpp`),
and the top-level tick entry point `sgc_sched_tick(SgcSchedState*, SgcSupState*,
SgcUkfState*, SgcCtrlState*, const SgcTickInput*, SgcActions*)`. This header
also defines the pneumatic state enum (`SGC_STATE_VENT=-1`, `SGC_STATE_HOLD=0`,
`SGC_STATE_FILL=1`, `SGC_STATE_OFF=2`, `SGC_STATE_UNKNOWN=99`) and controller
mode enum (`SGC_CTRL_NONE=0`, `SGC_CTRL_PI=1`, `SGC_CTRL_ADRC=2`,
`SGC_CTRL_PRESSURE=3`, `SGC_CTRL_PI_UKF=4`, `SGC_CTRL_ADRC_UKF=5`), plus timing
constants `SGC_SAMPLE_PERIOD_MS=10`, `SGC_UKF_SHADOW_LOOP_MS=50`,
`SGC_LOG_PERIOD_MS=50`, `SGC_FILL_MIN_MS=5`, `SGC_FILL_MAX_MS=80`,
`SGC_VENT_MIN_MS=10`, `SGC_VENT_MAX_MS=100`, `SGC_REVERSAL_LOCKOUT_MS=200`,
`SGC_HARD_PRESSURE_KPA=200.0f`. Other headers present but not read in full
for this document: `soft_glove_core.h`, `soft_glove_core_ctrl.h`,
`soft_glove_core_ukf.h`, `soft_glove_core_ukf_compose.h`.

## Package: pico_bridge

`ros2_ws/src/pico_bridge/` — the serial/USB bridge between the ROS 2 graph and
the Pico firmware. Single node `pico_bridge_node`
(`ros2_ws/src/pico_bridge/src/pico_bridge_node.cpp`, class `PicoBridgeNode`).

Parameters (declared in the constructor, `pico_bridge_node.cpp:126-131`):
`port` (default `/dev/ttyACM0`), `baud` (default `921600`), `transport`
(`serial` or `loopback`), `reconnect_backoff_ms` (default `250`),
`rx_timeout_ms` (default `100`, must be positive or the node throws
`std::invalid_argument`, `pico_bridge_node.cpp:133`).

Publishers:
- `/pico_bridge/sensor_frame` — `glove_interfaces/msg/SensorFrame` (queue depth 10),
  published from `decode_sensor()` on accepted `S,...` wire frames
  (`pico_bridge_node.cpp:134-135`, `:382`).
- `/pico_bridge/estimator_input` — `glove_interfaces/msg/EstimatorInputFrame`
  (queue depth 10), published from `decode_estimator()` on accepted `E,...`
  wire frames (`:136-137`, `:400`).
- `/pico_bridge/pico_status` — `glove_interfaces/msg/PicoStatus` (queue depth 10),
  published from `publish_status()` roughly once per second via
  `status_timer_` and also on link/frame events (`:138-139`, `:170`, `:274-287`).

Subscriptions:
- `/glove_control/actuator_command` — `glove_interfaces/msg/ActuatorCommand`
  (queue depth 10), handled by `on_actuator()`, which encodes and writes an
  `A,...` wire frame if the link is up (`:140-144`, `:198-215`).

Services:
- `/pico_bridge/calibrate_sensors` — `glove_interfaces/srv/CalibrateSensors`,
  served asynchronously; the callback stores the `rmw_request_id_t`, sets a
  2-second deadline, and sends a `C,seq,crc\n` frame to the Pico; the response
  is sent later from `finish_calibration()` when a matching `Z,...` reply
  arrives or the deadline/link drops (`:145-161`, `:236-246`, `:443-444`).

Wire protocol handling (own implementation, distinct from the Pico firmware's
C implementation but wire-compatible): CRC16-CCITT computed by a local
`crc16()` lambda-free function (`:31-42`), line framing/parsing in
`receive_line()` (`:292-344`), with the same `S` (sensor, 17 fields),
`E` (estimator, 15 fields), `Z` (calibration result, 6 fields), and outbound
`A` (actuator command) frame kinds as the firmware protocol.

## Package: glove_estimator

`ros2_ws/src/glove_estimator/` — runs the UKF-based state estimator. Single
node `ukf_node` (`ros2_ws/src/glove_estimator/src/ukf_node.cpp`, class `UkfNode`).

Parameter: `ref_samples` (default `40`, `ukf_node.cpp:20`) — number of recent
input frames retained for reference calibration.

Publisher:
- `/ukf/estimator_state` — `glove_interfaces/msg/EstimatorState` (queue depth 10),
  published once per processed update from `on_frame()` (`:23-24`, `:85-109`).

Subscription:
- `/pico_bridge/estimator_input` — `glove_interfaces/msg/EstimatorInputFrame`
  (queue depth 10), handled by `on_frame()` (`:25-29`). Frames are consumed at
  the wire rate but the UKF update itself is gated to a fixed 50 ms grid
  (`kUpdatePeriodUs = 50000u`, `:140`); sequence discontinuities relative to
  `msg.seq` are logged and counted (`seq_discontinuity_`, `seq_gap_count_`,
  `:51-58`).

Service:
- `~/calibrate_ukf_reference` (private/relative name, resolves under the node
  namespace, i.e. `/ukf_node/calibrate_ukf_reference`) —
  `glove_interfaces/srv/CalibrateUkfReference`, served by `on_calibrate()`,
  which requires at least `ref_samples` buffered input frames and calls
  `sgc_ukf_accumulate_reference()` from `soft_glove_core` (`:30-37`, `:112-137`).

Uses `soft_glove_core/soft_glove_core_ukf_compose.h` (`sgc_ukf_compose_init`,
`sgc_ukf_compose_step`, `sgc_ukf_accumulate_reference`) for the actual UKF math;
this node is a thin ROS wrapper.

## Package: glove_controller

`ros2_ws/src/glove_controller/` — the high-level supervisory/control node.
Single node `glove_control_node`
(`ros2_ws/src/glove_controller/src/glove_control_node.cpp`, class `GloveControlNode`).
Depends on `pico_bridge` and `glove_estimator` only as `exec_depend` (launch-time
ordering), not a build dependency (`ros2_ws/src/glove_controller/package.xml`).

Parameters (constructor, `:25-29`): `base_tick_ms` (default `1`),
`estimator_max_age_ms` (default `150`), `sensor_max_age_ms` (default `50`),
`flex_zero_raw` (default `0.0`), `csv_path` (default empty).

Publishers:
- `/glove_control/actuator_command` — `glove_interfaces/msg/ActuatorCommand`
  (queue depth 10), published by `publish_actuator()` (`:36-37`, `:248-255`).
- `/glove_control/controller_status` — `glove_interfaces/msg/ControllerStatus`
  (queue depth 10), published by `publish_status()` once per tick (`:38-39`,
  `:273-320`).

Subscriptions:
- `/pico_bridge/sensor_frame` — `glove_interfaces/msg/SensorFrame` (queue
  depth 10), handled by `on_sensor()` (`:40-44`, `:59-65`).
- `/ukf/estimator_state` — `glove_interfaces/msg/EstimatorState` (queue
  depth 10), handled by `on_estimator()`, which also mirrors UKF telemetry
  fields into the local `SgcUkfState` used by `soft_glove_core`
  (`:45-49`, `:67-74`, `:210-233`).
- `/glove_control/control_command` — `glove_interfaces/msg/ControlCommand`
  (queue depth 10), handled by `on_command()`, which maps `msg.kind` to
  pending supervisor actions: `0`=set q reference, `1`=set mode, `2`=set
  logging, `3`=reset, `4`=abort, and `ControlCommand::SET_P` (constant `5`)
  = set pressure reference, gated to `SGC_CTRL_PRESSURE` mode and the range
  `[SGC_PRESSURE_REF_MIN_KPA, SGC_PRESSURE_REF_MAX_KPA]` (`:76-111`). Once
  `abort_active_` or `overpressure_latched_` is set, further commands are
  ignored (`:78-79`) — emergency shutdown is terminal until node restart.

Timer: `base_tick_ms_`-period wall timer drives `on_tick()`, which stages
sensor/estimator staleness checks, calls `sgc_sched_tick()` from
`soft_glove_core`, applies a special "golden active-mode entry" sequence
(pump ON, then force HOLD) when entering an active control mode (`:169-187`),
publishes resulting actuator commands, and publishes `ControllerStatus`
every tick (`:113-196`).

Actuator command encoding from `SgcActions`/valve state uses the same command
codes as the Pico wire protocol: `0`=HOLD, `1`=FILL, `2`=VENT, `3`=PUMP_ON,
`4`=PUMP_OFF, `5`=ALL_OFF (`valve_command()` at `:235-246`, plus literals at
`:180-181`, `:266`, `:269`).

## Cross-package data flow summary

```
pico_bridge_node  --/pico_bridge/sensor_frame-->        glove_control_node
pico_bridge_node  --/pico_bridge/estimator_input-->      ukf_node
ukf_node          --/ukf/estimator_state-->              glove_control_node
glove_control_node --/glove_control/actuator_command-->  pico_bridge_node
(external)        --/glove_control/control_command-->    glove_control_node
glove_control_node --/glove_control/controller_status--> (telemetry consumers)
pico_bridge_node  --/pico_bridge/pico_status-->          (telemetry consumers)
services: /pico_bridge/calibrate_sensors, /ukf_node/calibrate_ukf_reference (relative ~/calibrate_ukf_reference)
```
