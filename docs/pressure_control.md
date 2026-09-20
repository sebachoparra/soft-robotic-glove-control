# Internal Pressure PI Loop and Pneumatic State Machine

Sources: `reference/rp2040_validated/main.c` (golden, read-only reference),
`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c` and
`soft_glove_core_sched.c`/`soft_glove_core.c` (ROS 2 port of the same logic),
`ros2_ws/src/glove_controller/src/glove_control_node.cpp`, and
`docs/PRESSURE_BRINGUP.md`.

## 1. Pressure PI gains and structure (reference)

Constants (`reference/rp2040_validated/main.c:197-198,206`):

```
KP_PRESSURE               0.060f
KI_PRESSURE                0.025f
PRESSURE_DEADBAND_KPA       1.0f
```

The ROS 2 port defines the identical values as `SGC_KP_PRESSURE = 0.060f`,
`SGC_KI_PRESSURE = 0.025f`, `SGC_PRESSURE_DEADBAND_KPA = 1.0f`, plus an explicit loop
period `SGC_PRESSURE_LOOP_MS = 100` / `SGC_PRESSURE_TS_S = 0.100f` and integrator clamp
`SGC_PRESSURE_I_MIN = -1.0f` / `SGC_PRESSURE_I_MAX = 1.0f`
(`ros2_ws/src/soft_glove_core/include/soft_glove_core/soft_glove_core_ctrl.h:23-29`).

The ROS 2 port function `sgc_ctrl_pressure_pi_update()`
(`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:394-502`) implements:

- `error = reference - pressure` (line 402-403).
- `p_term = SGC_KP_PRESSURE * error` (line 405-407).
- **Deadband**: if `|error| <= SGC_PRESSURE_DEADBAND_KPA` (1.0 kPa), the integrator is
  held (not updated), `output_unsat = 0`, `output = 0` — the loop goes fully idle
  inside the deadband (lines 409-420).
- Outside the deadband, a candidate integral is formed:
  `candidate_integral = pressure_integral + SGC_KI_PRESSURE * error * SGC_PRESSURE_TS_S`,
  clamped to `[SGC_PRESSURE_I_MIN, SGC_PRESSURE_I_MAX] = [-1, 1]` (lines 422-433).
- Anti-windup: the integrator is only committed
  (`st->pressure_integral = candidate_integral`) if the resulting **unsaturated**
  command magnitude (`p_term` plus the positive/negative part of the candidate
  integral, depending on error sign) does not exceed 1.0 — i.e. conditional
  integration, not simple back-calculation (lines 435-463).
- Final `output` is `p_term + max(integral,0)` (error>0) or the mirrored
  expression for error<0, clamped to `[0,1]` and sign-applied
  (lines 465-499). This looks like a fractional duty/command in `[-1, 1]`, driving
  FILL (positive) or VENT (negative) pulse computation elsewhere (not shown in the
  excerpt read).

The reference `main.c` defines the equivalent `pressure_pi_update()` function at
`reference/rp2040_validated/main.c:1402` (called from `main.c:1612`); its gains/deadband
constants at lines 197-198/206 above are shared with the position-loop constants block.
The ROS 2 `soft_glove_core_ctrl.c` version reproduces the same gain values, deadband,
and clamp structure described above; this document does not re-diff the two functions
byte-for-byte, only confirms the constants and control structure line up per the source
read.

## 2. Pneumatic FILL / HOLD / VENT logic

Reference constants (`reference/rp2040_validated/main.c:107-114,213-217`):

```
V1_FILL_LEVEL  VALVE_OFF        V2_FILL_LEVEL  VALVE_OFF
V1_HOLD_LEVEL  VALVE_OFF        V2_HOLD_LEVEL  VALVE_ON
V1_VENT_LEVEL  VALVE_ON         V2_VENT_LEVEL  VALVE_ON
FILL_MIN_MS 5   FILL_MAX_MS 80
VENT_MIN_MS 10  VENT_MAX_MS 100
```

State enum `PneumaticState { STATE_VENT = -1, STATE_HOLD = 0, STATE_FILL = 1 }`
(`reference/rp2040_validated/main.c:240-242`), with `set_state()` driving GPIO levels
for V1/V2 per state (`reference/rp2040_validated/main.c:504-517`). Pump pin is GP16
(`PUMP_PIN`), V1=GP17, V2=GP14, pressure ADC=GP26 — matching the pin table restated in
`docs/PRESSURE_BRINGUP.md` and the Spanish gain-scheduling note in
`reference/rp2040_validated/GAIN_SCHEDULING.md:13` ("PUMP=GP16, V1=GP17, V2=GP14,
presión=GP26, flex=GP28").

On the ROS 2 side, `sup_.pneumatic_state` (`SgcPneumaticState`, opaque to this
document beyond its use) is mapped to wire actuator commands by
`GloveControlNode::valve_command()`
(`ros2_ws/src/glove_controller/src/glove_control_node.cpp:235-246`):

```
SGC_STATE_FILL -> 1u
SGC_STATE_VENT -> 2u
SGC_STATE_HOLD -> 0u (default)
```

These integers match the wire command table documented in `docs/PRESSURE_BRINGUP.md:22`:
`0 HOLD, 1 FILL, 2 VENT, 3 PUMP_ON, 4 PUMP_OFF, 5 ALL_OFF`.

Entering any active control mode (PI, ADRC, PI_UKF, ADRC_UKF, PRESSURE) forces
`pump ON` then `HOLD` before the scheduler's own valve command is applied — "Golden
active-mode entry: pump ON first, then force the pneumatic path to HOLD"
(`ros2_ws/src/glove_controller/src/glove_control_node.cpp:169-187`).

## 3. Pressure-reference rate limiting and saturation

The reference limits (mirrored in the ROS 2 port) at
`ros2_ws/src/soft_glove_core/include/soft_glove_core/soft_glove_core.h:24-28`:

```
SGC_PRESSURE_REF_MIN_KPA   0.0f
SGC_PRESSURE_REF_MAX_KPA 170.0f
SGC_PREF_RATE_UP_KPA_S    10.0f
SGC_PREF_RATE_DOWN_KPA_S  12.0f
```

These match `PRESSURE_REF_MIN_KPA = 0.0f` / `PRESSURE_REF_MAX_KPA = 170.0f`
(`reference/rp2040_validated/main.c:147-148`). The shared limiter function
`sgc_apply_common_pref_limits()` (declared
`ros2_ws/src/soft_glove_core/include/soft_glove_core/soft_glove_core.h:111`, implemented
`ros2_ws/src/soft_glove_core/src/soft_glove_core.c:342+`) is invoked from both the
position-PI loop (`sgc_ctrl_position_pi_update()`,
`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:160-166`) and LADRC
(`sgc_ctrl_ladrc_update()`, `soft_glove_core_ctrl.c:348-354`) to clamp/rate-limit the
pressure reference (`pressure_ref_kpa`) before it is committed, using
`SGC_PREF_RATE_UP_KPA_S`/`SGC_PREF_RATE_DOWN_KPA_S` × `SGC_OUTER_TS_S` per outer tick
(`soft_glove_core_ctrl.c:171-192,356-377`) and reporting `rate_limited`/`saturated`
flags that are surfaced on `ControllerStatus.rate_limited`/`.saturated`
(`glove_control_node.cpp:288-291`).

## 4. Pressure conversion formula

Per `docs/PRESSURE_BRINGUP.md:39-42` (matches the golden firmware's
`pressure_raw_to_vout()`/`pressure_raw_to_kpa()` at
`reference/rp2040_validated/main.c:562-576`, with `ADC_REF_VOLTAGE=3.3f`,
`ADC_MAX_VALUE=4095.0f`, `DIVIDER_GAIN=1.545f`, `KPA_PER_VOLT=50.0f`,
`reference/rp2040_validated/main.c:136-139`):

```
Vout   = (raw * 3.3 / 4095) * 1.545
P_kPa  = (Vout - zero_vout) * 50
```

## 5. ROS 2 exposure: `SET_P`, `CTRL_PRESSURE`, and the wire protocol

From `docs/PRESSURE_BRINGUP.md:12-25` and
`ros2_ws/src/glove_controller/src/glove_control_node.cpp:76-111`:

- `/glove_control/control_command` (`glove_interfaces/msg/ControlCommand`):
  - `kind: 1, mode: 3` enters `CTRL_PRESSURE` — reseeds `pressure_ref_kpa` from the
    current filtered pressure "as in the validated firmware", sends `PUMP_ON` then
    `HOLD`.
  - `kind: 5` (`ControlCommand::SET_P`) sets `ctrl_.pressure_ref_kpa` directly, but
    **only** if `sup_.controller_mode == SGC_CTRL_PRESSURE` and
    `msg.pressure_ref_kpa` is finite and within
    `[SGC_PRESSURE_REF_MIN_KPA, SGC_PRESSURE_REF_MAX_KPA] = [0, 170]`
    (`glove_control_node.cpp:99-107`); otherwise the reference is left unchanged.
  - `kind: 1, mode: 0` ends control (`CTRL_NONE`) and sends `ALL_OFF`.
- Wire actuator commands (`docs/PRESSURE_BRINGUP.md:21-25`):
  `A,seq,command,pulse_ms,deadline_us,crc\n` with
  `0 HOLD, 1 FILL, 2 VENT, 3 PUMP_ON, 4 PUMP_OFF, 5 ALL_OFF`. Pump commands do not
  change valve position; timed FILL/VENT return to HOLD; `ALL_OFF` stops any pulse and
  drives GP16/GP17/GP14 low; the 200 ms communication timeout is unchanged.
- Local safety: overpressure is terminal for that Pico instance; incoming normal
  commands cannot clear that latch; other latches (comm/invalid-pressure/watchdog)
  require an explicit fresh `ALL_OFF` once pressure telemetry is valid again
  (`docs/PRESSURE_BRINGUP.md:27-31`). `PUMP_ON` is rejected until zero calibration
  succeeds, and calibration is RAM-only (must repeat after reboot).

## 6. Not specified in the inspected source

- The reference `main.c` pulse-duration mapping from the PI `output` (fractional
  command in `[-1,1]`) to actual `FILL_MIN_MS..FILL_MAX_MS` / `VENT_MIN_MS..VENT_MAX_MS`
  pulse widths was not traced end-to-end in this document; only the constants
  (`FILL_MIN_MS=5, FILL_MAX_MS=80, VENT_MIN_MS=10, VENT_MAX_MS=100`) were located
  (`reference/rp2040_validated/main.c:213-217`).
