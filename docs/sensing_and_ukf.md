# Sensing and the UKF4 State Estimator

This document describes the **reference (golden) UKF4 algorithm** as implemented in the
frozen firmware under `reference/rp2040_validated/`, and how it relates to the ROS 2
`glove_estimator` node. `reference/rp2040_validated/` is a read-only, frozen reference
(per the project's governance policy); nothing in this document is based on any modification of it.

## 1. Reference algorithm (golden firmware)

Source: `reference/rp2040_validated/ukf_shadow_rp2040.h`, comment block at
`reference/rp2040_validated/ukf_shadow_rp2040.h:1-54`.

### 1.1 State vector

```
x = [s, sdot, eta, bF]^T
```
(`reference/rp2040_validated/ukf_shadow_rp2040.h:11`, and the internal storage comment
at `reference/rp2040_validated/ukf_shadow_rp2040.h:227-230`, where the state array is
named `ukfs_x[UKFS_NX]` with `UKFS_NX = 4`, `reference/rp2040_validated/ukf_shadow_rp2040.h:99,230`).

Meaning of each state, per the header comment and telemetry struct field names
(`reference/rp2040_validated/ukf_shadow_rp2040.h:190-193`):

- `s` (`s_hat`) — estimated flex/position state on a 0..100 grid (bounded by
  `UKFS_S_MIN`/`UKFS_S_MAX` = 0.0 / 100.0, `reference/rp2040_validated/ukf_shadow_rp2040.h:133-134`).
- `sdot` (`v_hat`) — rate of `s` (bounded `UKFS_V_MIN`/`UKFS_V_MAX` = -250/250,
  `reference/rp2040_validated/ukf_shadow_rp2040.h:135-136`).
- `eta` (`eta_hat`) — continuous hysteresis-branch state in [0,1]
  (`UKFS_ETA_MIN`/`UKFS_ETA_MAX`, `reference/rp2040_validated/ukf_shadow_rp2040.h:137-138`);
  `eta = 0` selects the UP major-loop branch and `eta = 1` the DOWN branch of the
  measurement LUT (`reference/rp2040_validated/ukf_model_lut.h:16-19`).
- `bF` (`b_f_hat`) — slow FLEX sensor/session offset/bias, bounded
  `UKFS_BIAS_MIN`/`UKFS_BIAS_MAX` = -80/80 counts
  (`reference/rp2040_validated/ukf_shadow_rp2040.h:139-140`). Per the header comment,
  "FLEX is NOT re-zeroed per run. The state bF estimates the slow sensor/session offset."
  (`reference/rp2040_validated/ukf_shadow_rp2040.h:47-48`).

### 1.2 Measurement model

```
z = [FLEX_raw, theta_signed]^T
F = (1-eta) F_UP(s) + eta F_DOWN(s) + bF
T = (1-eta) T_UP(s) + eta T_DOWN(s)
```
(`reference/rp2040_validated/ukf_shadow_rp2040.h:13-18`). `F_UP`, `F_DOWN`, `T_UP`,
`T_DOWN` are 101-point lookup tables over `s = 0..100` sourced from
`UKF_V2_measurement_curves_dense.csv` and stored as `UKF_MODEL_F_UP`,
`UKF_MODEL_F_DOWN`, `UKF_MODEL_THETA_UP_DEG`, `UKF_MODEL_THETA_DOWN_DEG`
(`reference/rp2040_validated/ukf_model_lut.h:22-111`), interpolated via `ukfs_lut()`
(`reference/rp2040_validated/ukf_shadow_rp2040.h:393-399`). Per-branch measurement
sigma tables (`UKF_MODEL_SIGMA_F_UP`, `UKF_MODEL_SIGMA_F_DOWN`, etc.) are also present
in the LUT header and floored by `UKFS_SIGMA_F_MIN` / `UKFS_SIGMA_T_MIN`
(`reference/rp2040_validated/ukf_shadow_rp2040.h:142-143`).

### 1.3 Process model

```
s(k+1)    = s + dt*sdot
sdot(k+1) = sdot
zeta = 2*eta - 1
dzeta/dt = a1*sdot + a2*|sdot|*zeta + a3*sdot*|zeta| + a4*zeta + a5*(s-50)/50 + a6
bF(k+1) = bF(k) + w_b
```
(`reference/rp2040_validated/ukf_shadow_rp2040.h:20-33`). The six offline-identified
`a1..a6` coefficients for the eta dynamics are `UKFS_ETA_A1..UKFS_ETA_A6`
(`reference/rp2040_validated/ukf_shadow_rp2040.h:124-129`).

### 1.4 UKF tuning (offline-selected, frozen)

From `reference/rp2040_validated/ukf_shadow_rp2040.h:35-42,99-121`:

- `alpha = 0.2`, `beta = 2`, `kappa = 0`, giving `lambda = -3.84`, `c = 0.16`,
  weights `Wm0 = -24`, `Wc0 = -21.04`, `Wi = 3.125`
  (comment + `#define`s at `reference/rp2040_validated/ukf_shadow_rp2040.h:103-116`).
- `sigma_acc = 50`, `sigma_eta_RW = 0.06`, `sigma_bF_RW = 0.005 counts/sqrt(s)`,
  `R_scale = 0.02` (multiplies measurement variances)
  (`reference/rp2040_validated/ukf_shadow_rp2040.h:118-121`).
- Initial covariance: `P0_S = 16`, `P0_V = 225`, `P0_ETA = 0.0625`, `P0_BIAS = 225`
  (`reference/rp2040_validated/ukf_shadow_rp2040.h:148-152`).
- State is 4-dimensional (`UKFS_NX = 4`), with `UKFS_NSIGMA = 2*NX+1 = 9` sigma points
  (`reference/rp2040_validated/ukf_shadow_rp2040.h:99-101`).
- Numerical hygiene: covariance is symmetrized, diagonal-floored
  (`UKFS_P_FLOOR = 1e-7`), pairwise-covariance clamped to 0.995×geometric-mean bound,
  and repaired via diagonal jitter/Cholesky retry before falling back to a diagonal
  covariance (`ukfs_repair_cov4()`, `reference/rp2040_validated/ukf_shadow_rp2040.h:321-389`).

### 1.5 Execution rate

`UKF_SHADOW_LOOP_MS = 50u` (`reference/rp2040_validated/ukf_shadow_rp2040.h:70`) — the
header defines a 50 ms nominal loop period for the shadow UKF on RP2040. Reference
sampling for BNO calibration uses `UKF_SHADOW_REF_SAMPLES = 40u` samples with
`UKF_SHADOW_REF_SAMPLE_DELAY_MS = 20u` between samples
(`reference/rp2040_validated/ukf_shadow_rp2040.h:71-72`). Beyond this header-level
constant, the exact scheduling/call-site of the 50 ms tick inside `main.c` was not
inspected as part of this document; if a different effective rate is enforced by the
main control loop, that is **not specified in this document** — treat the 50 ms figure
as the value declared in `ukf_shadow_rp2040.h`.

### 1.6 BNO055 IMU usage

The header targets two BNO055 IMUs on I2C0 (`UKFS_I2C_PORT = i2c0`, SDA=GP4, SCL=GP5,
addresses `0x28`/`0x29` for BNO1/BNO2) with register-level constants for chip ID,
page/unit-select, operation mode (`NDOF = 0x0C`) and quaternion data registers
(`reference/rp2040_validated/ukf_shadow_rp2040.h:76-95`). Presence/health of each IMU is
tracked (`bno1_present`, `bno2_present`, `bno1_ok`, `bno2_ok`) and both units' quaternions
(`bno1_qw..qz`, `bno2_qw..qz`) are stored in `UKFShadowTelemetry`
(`reference/rp2040_validated/ukf_shadow_rp2040.h:170-184`). `theta_deg` / `theta_valid`
represent a derived signed angle used as the second UKF measurement channel; if theta
is unavailable, "the filter performs prediction only instead of a FLEX-only update,
because FLEX alone cannot reliably separate s, eta and bF"
(`reference/rp2040_validated/ukf_shadow_rp2040.h:50-52`).

The current Pico firmware (`pico_firmware/src/pico_sensors.c`) configures both BNO055
units in **IMUPLUS mode** (`BNO_MODE_IMUPLUS = 0x08`, written to the operation-mode
register at `pico_firmware/src/pico_sensors.c:19,47,55`), not NDOF mode. This differs
from the `NDOF = 0x0C` constant referenced above in the golden reference header
(`reference/rp2040_validated/ukf_shadow_rp2040.h:76-95`). IMUPLUS uses the sensor's
gyroscope+accelerometer fusion without the magnetometer, whereas NDOF adds magnetometer
fusion; no further interpretation of this difference is made here beyond what the
implementation shows.

### 1.7 Explicit exclusions (reference header)

"No pressure, q_ref, PI, ADRC, Direction, Step, or quasi-static metadata enters the
estimator." and "For this first embedded deployment the UKF remains SHADOW ONLY."
(`reference/rp2040_validated/ukf_shadow_rp2040.h:44-49`).

## 2. ROS 2 `glove_estimator` port

Source: `ros2_ws/src/glove_estimator/src/ukf_node.cpp`, and the shared library it calls
into, `ros2_ws/src/soft_glove_core` (headers `soft_glove_core_ukf.h`,
`soft_glove_core_ukf_compose.h`; implementation `soft_glove_core_ukf.c`,
`soft_glove_core_ukf_compose.c`). This document does not claim byte-for-byte equivalence
between the port and the reference header beyond what is directly observable from the
files read; see the project's migration principle ("do not claim equivalence
without tests or direct comparison evidence").

### 2.1 Node structure

`UkfNode` (`ros2_ws/src/glove_estimator/src/ukf_node.cpp:15-153`):

- Subscribes to `glove_interfaces/msg/EstimatorInputFrame` on
  `/pico_bridge/estimator_input` (`ukf_node.cpp:25-29`). This message carries
  `pico_timestamp_us`, `seq`, `flex_filtered_raw`, and per-IMU `bno{1,2}_ok` +
  `bno{1,2}_quat[4]` (`ros2_ws/src/glove_interfaces/msg/EstimatorInputFrame.msg:1-8`) —
  i.e. the FLEX and dual-BNO quaternion inputs that feed the same measurement model
  described in §1.2.
- Publishes `glove_interfaces/msg/EstimatorState` on `/ukf/estimator_state`
  (`ukf_node.cpp:23-24,85-109`) with fields `s_hat`, `v_hat`, `eta_hat`, `b_f_hat`,
  `theta_deg`, `innovation_flex`, `innovation_theta`, `nis`, `sigma_s`, `sigma_v`,
  `sigma_eta`, `sigma_b_f`, `rho_eta_b_f`, `dt_used`, `valid`, `reference_valid`,
  `theta_valid`, `bno1_ok`, `bno2_ok`, `source_seq`, `source_timestamp_us`,
  `estimator_stamp` (`ros2_ws/src/glove_interfaces/msg/EstimatorState.msg:1-23`,
  populated at `ukf_node.cpp:87-109`). These field names directly mirror the reference
  telemetry struct `UKFShadowTelemetry` fields named in §1.1–1.4
  (`s_hat`, `v_hat`, `eta_hat`, `b_f_hat`, `sigma_s`/`sigma_v`/`sigma_eta`/`sigma_b_f`,
  `nis`, `rho_eta_b_f`, `reference_valid`, `theta_valid`,
  `reference/rp2040_validated/ukf_shadow_rp2040.h:190-209`).
- Exposes a `~/calibrate_ukf_reference` service
  (`glove_interfaces/srv/CalibrateUkfReference`) that accumulates a buffer of recent
  BNO samples (`ref_samples_`, default 40, declared as a ROS parameter
  `ukf_node.cpp:20,60-65`) and calls `sgc_ukf_accumulate_reference()`
  (`ukf_node.cpp:126-127`) — the parameter default of 40 matches
  `UKF_SHADOW_REF_SAMPLES = 40u` in the reference header
  (`reference/rp2040_validated/ukf_shadow_rp2040.h:71`).
- Update gating: the node computes filter updates on a fixed grid via
  `kUpdatePeriodUs = 50000u` (50 ms) (`ukf_node.cpp:140`), matching the reference
  header's `UKF_SHADOW_LOOP_MS = 50u` (§1.5). Frames arriving faster than the grid are
  buffered as reference samples but do not trigger a new filter step
  (`ukf_node.cpp:67-80`).
- Estimation itself (sigma points, process/measurement model, Cholesky, gain
  computation) is implemented inside `soft_glove_core` (`sgc_ukf_compose_step()`,
  called at `ukf_node.cpp:81-83`), not in `ukf_node.cpp` directly. This document does
  not re-derive that implementation line-by-line; it is expected to be the ROS 2 port
  of the algorithm in §1, but exact numerical equivalence to
  `reference/rp2040_validated/ukf_shadow_rp2040.h` was **not verified as part of writing
  this document** — see the project's equivalence-testing requirements.

### 2.2 Relationship to the golden reference

- The ROS 2 node consumes the same two measurement channels (FLEX, dual-BNO
  quaternion-derived theta) and produces the same state/telemetry vocabulary
  (`s_hat`, `v_hat`, `eta_hat`, `b_f_hat`, sigmas, `nis`, `rho_eta_b_f`) as the
  reference RP2040 shadow UKF.
  reads sensor data over I2C and runs the filter on-device; the ROS 2 node instead
  receives pre-parsed `EstimatorInputFrame` messages (produced by the Pico bridge /
  firmware layer, not read here) and runs the filter step on the ROS 2 side.
- The `CalibrateUkfReference` service and its 40-sample reference buffer parallel the
  reference header's `UKF_SHADOW_REF_SAMPLES`/`UKF_SHADOW_REF_SAMPLE_DELAY_MS`-driven
  reference-quaternion calibration described at
  `reference/rp2040_validated/ukf_shadow_rp2040.h:71-72`.
- Whether the underlying `sgc_ukf_compose_step()` implementation reproduces the exact
  sigma-point weights, process/measurement equations, and tuning constants of §1.1–1.4
  to numerical tolerance was not verified from source as part of this document; per
  the project's migration policy, that equivalence should be established by dedicated tests/replay
  comparison, not asserted here.
