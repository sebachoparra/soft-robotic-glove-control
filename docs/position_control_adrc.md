# LADRC / LESO Position Controller

Sources: `reference/rp2040_validated/main.c` (golden, read-only reference, search terms
`ladrc`, `leso`, `b0`, `z1`, `z2`, `eso`), the ROS 2 port
`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c` /
`soft_glove_core_ctrl.h`, `ros2_ws/src/glove_controller/src/glove_control_node.cpp`, and
`glove_interfaces/msg/ControllerStatus.msg`.

## 1. Constants (reference)

`reference/rp2040_validated/main.c:171-191`:

```
B0_NOMINAL                 2.20f
B0_FALLBACK                B0_NOMINAL
B0_SMOOTH_TAU_S            0.50f
ESO_OMEGA_O                2.50f
ESO_BETA1                 (2.0f * ESO_OMEGA_O)     // = 5.0
ESO_BETA2                 (ESO_OMEGA_O * ESO_OMEGA_O)  // = 6.25
ESO_LOOP_MS                50
ESO_TS_S                   0.050f
LADRC_OMEGA_C              0.40f
LADRC_FF_MODEL_WEIGHT       0.30f
LADRC_FF_MAX_UP_KPA         8.0f
LADRC_FF_MAX_DOWN_KPA       5.0f
LADRC_FF_FULL_ERROR_PCT    10.0f
LADRC_FF_ZERO_ERROR_PCT     2.0f
```

The ROS 2 port defines the identical set with an `SGC_` prefix and identical values
(`ros2_ws/src/soft_glove_core/include/soft_glove_core/soft_glove_core_ctrl.h:8-22`).

`CTRL_ADRC_UKF = 5` is documented in the reference as "LESO/LADRC feedback UKF4: s_hat"
(`reference/rp2040_validated/main.c:253`), i.e. one of the controller modes drives the
LADRC position feedback from the UKF's `s_hat` rather than raw FLEX.

## 2. LESO (Linear Extended State Observer)

Reference `leso_update()` (`reference/rp2040_validated/main.c:1205-1232`); ROS 2 port
`sgc_ctrl_leso_update()` (`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:212-240`),
functionally identical:

```
error  = position_measured_pct - z1_pct
z1_dot = z2_pct_s + active_b0 * pressure_input_kpa + ESO_BETA1 * error
z2_dot = ESO_BETA2 * error
z1_pct   += ESO_TS_S * z1_dot
z2_pct_s += ESO_TS_S * z2_dot
```

This is a standard second-order LESO: `z1` tracks measured position, `z2` tracks the
"total disturbance" (unmodeled dynamics + control-input coupling), integrated with
explicit-Euler step `ESO_TS_S = 0.050` s (50 ms loop, `ESO_LOOP_MS = 50`). Observer
gains `ESO_BETA1 = 2·omega_o = 5.0`, `ESO_BETA2 = omega_o^2 = 6.25` with
`ESO_OMEGA_O = 2.50` (standard ADRC bandwidth-parameterized LESO gain choice).

## 3. `active_b0` scheduling and smoothing

`active_b0` is not held constant — it is low-pass filtered toward a `target_b0` that
comes from the same plant-schedule evaluation described in
`docs/position_control_pi.md` §2. Reference `update_active_b0()`
(`reference/rp2040_validated/main.c:1000-1039`); ROS 2 port
`sgc_ctrl_update_active_b0()` (`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:82-120`):

```
alpha  = ESO_TS_S / (B0_SMOOTH_TAU_S + ESO_TS_S)
new_b0 = old_b0 + alpha * (target_b0 - old_b0)
```
(first-order exponential smoothing toward `target_b0` with time constant
`B0_SMOOTH_TAU_S = 0.50` s). Non-finite or non-positive results fall back to
`B0_FALLBACK = B0_NOMINAL = 2.20`; if the update lands within `1e-5` of the target it
snaps exactly to `target_b0` (`soft_glove_core_ctrl.c:96-107`).

**`target_b0`** is set in the ROS 2 port's `sgc_ctrl_update_plant_schedule()`
(`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:44-46`):

```
target_b0 = 0.80f * SGC_B0_NOMINAL + 0.20f * next.b0;
```

i.e. an 80% nominal / 20% schedule-identified blend of `b0`, where `next.b0` is the
`k/tau` value computed by `plant_schedule_eval()` (see `docs/position_control_pi.md`
§2.1 step 7). The reference `main.c` contains the same line, with the 100%-identified
alternative explicitly commented out:
```c
//target_b0 = next.b0;  // NOLINT(whitespace/comments)
target_b0 = 0.80f * B0_NOMINAL + 0.20f * next.b0;
```
(`reference/rp2040_validated/main.c:960,963-965`, mirrored verbatim in the ROS 2 port at
`soft_glove_core_ctrl.c:41,44-46`). `reference/rp2040_validated/GAIN_SCHEDULING.md:25`
previously contained stale prose claiming the b0 target "no longer mixes 80% nominal and
20% identified"; that line has since been corrected to state the 80/20 blend explicitly,
resolving the discrepancy with the source flagged in earlier review.

When `active_b0` is updated, `z2` is corrected to preserve the estimated derivative
`q_dot_hat = z2 + b0*P_ref` across the b0 change:
```
z2_pct_s += (old_b0 - new_b0) * pressure_ref_kpa
```
(`reference/rp2040_validated/main.c:1032-1038`; ROS 2 port
`soft_glove_core_ctrl.c:110-116`, same comment reproduced).

## 4. LADRC control law

Reference `ladrc_update()` (`reference/rp2040_validated/main.c:1234-1329+`); ROS 2 port
`sgc_ctrl_ladrc_update()` (`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:242-392`).

```
position_error_pct     = q_ref - z1_pct
equivalent_pressure_kpa = -z2_pct_s / active_b0
feedback_correction_kpa = (LADRC_OMEGA_C / active_b0) * position_error_pct
```
(`soft_glove_core_ctrl.c:250-259`, matching `main.c:1241-1250`).

**`P_eq = -z2/b0` was found in source.** The reference comment at
`reference/rp2040_validated/main.c:791-797` (and mirrored in the ROS 2 port at
`soft_glove_core_ctrl.c:537-544`) reads:

```
Inicializacion coherente con q_dot_hat = z2 + b0*u.
Se supone q_dot_hat=0 al entrar al modo ADRC:
    z2 = -b0 * P_ref
Por tanto equivalent_pressure=-z2/b0=P_ref y el primer
update no fuerza un salto artificial de referencia.
```

and it is implemented directly as `r.equivalent_pressure_kpa = -eso_z2_pct_s /
active_b0;` both in the reference (`main.c:1244-1246`) and the ROS 2 port
(`soft_glove_core_ctrl.c:253-255`). This `equivalent_pressure_kpa` term is the LADRC
"disturbance-cancelling" feedforward — the pressure that, given the current estimated
disturbance `z2`, would produce zero net acceleration of the plant.

### 4.1 Feedforward on reference steps

A separate step feedforward, gated by a monotonically-decaying gamma ramp based on
tracking error, is computed once per `SET_Q` (reference-change) event:

```
ff = LADRC_FF_MODEL_WEIGHT * (q_to - q_from) / model_k   // 0.30 * Δq / K
clamped to [-LADRC_FF_MAX_DOWN_KPA, LADRC_FF_MAX_UP_KPA] = [-5, 8] kPa
```
(`sgc_ctrl_compute_adrc_feedforward()`, `soft_glove_core_ctrl.c:51-80`, matching
`main.c` around lines 958-995). The gamma weighting ramps from 0 at
`LADRC_FF_ZERO_ERROR_PCT = 2.0` %-error to 1 at `LADRC_FF_FULL_ERROR_PCT = 10.0`
%-error, and — per the comment "FF monotono por referencia: solo puede decrecer hasta
el siguiente SET_Q" (`soft_glove_core_ctrl.c:317-319`, `main.c:1316-1318`) — gamma can
only decrease (never re-increase) until the next `SET_Q`. Feedforward is disabled (γ=0)
if it points the wrong direction relative to the current position error
(`ff_direction_ok` check, `soft_glove_core_ctrl.c:269-277`).

### 4.2 Combined pressure-reference command

```
pressure_ref_raw_kpa = equivalent_pressure_kpa + feedback_correction_kpa + feedforward_applied_kpa
```
(`soft_glove_core_ctrl.c:340-343`), then passed through the same
`sgc_apply_common_pref_limits()` rate-limiter/saturation described in
`docs/pressure_control.md` §3 (`soft_glove_core_ctrl.c:348-377`).

## 5. ROS 2 telemetry exposure

`glove_interfaces/msg/ControllerStatus.msg` exposes `active_b0` (line 8), `eso_z1_pct`
(line 9), `eso_z2_pct_s` (line 10), and `ff_gamma` (line 11). These are populated
directly from the `SgcCtrlState` fields described above in
`GloveControlNode::publish_status()`:
```cpp
status.active_b0     = ctrl_.active_b0;
status.eso_z1_pct     = ctrl_.eso_z1_pct;
status.eso_z2_pct_s    = ctrl_.eso_z2_pct_s;
status.ff_gamma        = ctrl_.ff_gamma_state;
```
(`ros2_ws/src/glove_controller/src/glove_control_node.cpp:280-285`).
`status.rate_limited`/`status.saturated` are the logical OR of the position-PI and
LADRC result flags (`glove_control_node.cpp:288-291`).

## 6. Bumpless initialization

Both reference `initialize_adrc_bumpless()` (`reference/rp2040_validated/main.c:788-835`)
and ROS 2 `sgc_ctrl_initialize_adrc_bumpless()`
(`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:535-583`) set
`active_b0 = target_b0` (falling back to `B0_FALLBACK` if invalid), then seed
`z1_pct = feedback_pct` and `z2_pct_s = -active_b0 * pressure_ref_kpa` so that the
first LADRC update does not produce an artificial reference jump, per the same
`P_eq = -z2/b0 = P_ref` identity discussed in §4.
