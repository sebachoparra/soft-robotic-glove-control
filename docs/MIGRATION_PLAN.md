# Soft Glove — RP2040 → ROS 2 Jazzy Migration Plan

**Status:** analysis and planning only. No ROS 2 code exists yet.
**Golden reference:** `reference/rp2040_validated/`, tag `rp2040-golden-v1`.
**Scope rule:** this is a migration and equivalence-validation task. No controller is
retuned, redesigned, simplified or replaced. Every constant and equation below is
transcribed from the reference files, with file and line citations.

Anything not derivable from the reference files is collected in
[§10 Undetermined items](#10-undetermined-items) and is **not** invented here.

---

## 1. Firmware inventory

### 1.1 Files

| File | Lines | Role |
|---|---|---|
| `main.c` | 3516 | Scheduler, sensors, calibration, PI, LADRC/LESO, pressure PI, pneumatics, serial, CSV |
| `ukf_shadow_rp2040.h` | 1392 | UKF4 estimator, BNO055 driver, quaternion math, reference calibration |
| `ukf_model_lut.h` | 200 | Frozen measurement LUTs (F/θ UP+DOWN, σ UP+DOWN), hysteresis axis |
| `plant_schedule.h` | 43 | Continuous plant anchors and gain-scheduling evaluator |
| `CMakeLists.txt` | 48 | Pico SDK 2.3.0, target `glove_ukf_shadow`, C11, USB stdio |
| `GAIN_SCHEDULING.md` | 76 | Scheduling design note (see §11 — partially stale vs. code) |
| `README.md` | 53 | Controller-mode note |
| `BUILD_RP2040.ps1` / `.bat` | 101 / 5 | Windows build helpers |

Build: `pico_stdlib`, `hardware_adc`, `hardware_i2c`. stdio over **USB CDC**, UART disabled
(`CMakeLists.txt:45-46`). All headers are `static inline` / `static` — the firmware is a
single translation unit with **file-scope mutable globals**. This matters for the migration:
block boundaries below are logical, not compilation units.

### 1.2 Arithmetic precision — binding constraint

Every state variable, constant and intermediate in the frozen path is **`float` (IEEE-754
binary32)**. Math calls are the single-precision variants: `logf`, `expf`, `sqrtf`, `atan2f`,
`fabsf`, `fmaxf`, `fminf`. The ROS 2 port **must** use `float` throughout the frozen path and
must not be compiled with `-ffast-math`, `-funsafe-math-optimizations`, or x87 80-bit
intermediates. See §8.2 for the resulting tolerance policy.

### 1.3 Hardware map

| Signal | Pin | Detail |
|---|---|---|
| PUMP | GP16 | `main.c:89`, active high |
| V1 | GP17 | `main.c:90` |
| V2 | GP14 | `main.c:91` |
| Pressure | GP26 / ADC0 | `main.c:93-94` |
| FLEX | GP28 / ADC2 | `main.c:96-97` |
| I²C0 SDA / SCL | GP4 / GP5 | `ukf_shadow_rp2040.h:77-78`, 400 kHz, internal pull-ups |
| BNO055 #1 / #2 | 0x28 / 0x29 | `ukf_shadow_rp2040.h:80-81` |

### 1.4 Valve level table (`main.c:104-114`)

| State | V1 | V2 |
|---|---|---|
| FILL | OFF (0) | OFF (0) |
| HOLD | OFF (0) | ON (1) |
| VENT | ON (1) | ON (1) |

`set_state()` (`main.c:498`) handles only FILL/HOLD/VENT; `STATE_OFF` and `STATE_UNKNOWN`
fall through `default: break;` and **leave the GPIOs unchanged**. Only `all_outputs_off()`
(`main.c:534`) drives PUMP/V1/V2 low and sets `STATE_OFF`.

### 1.5 Main-loop execution order — behaviourally load-bearing

`main()` (`main.c:3126-3513`) runs one `while(true)` loop with six independently gated deadline
counters, all
initialised to the same `now` after calibration (`main.c:3105-3124`). That common origin is
the **only** point at which they are guaranteed to be in phase — see §1.6, which corrects an
earlier version of this document. Within one iteration the order is fixed:

1. `poll_serial_commands()` — every iteration (~1 kHz, `sleep_ms(1)` when idle)
2. `command_abort` check → `emergency_shutdown()`
3. `update_active_pulse(now)` — polled pulse-expiry backstop
4. 10 ms gate; if not due: `sleep_ms(1); continue;`
5. **Sensors**: pressure ADC → vout → kPa → **overpressure trip** → EWMA; FLEX ADC → EWMA → position
6. **UKF4** (50 ms) → `ukf_shadow_update(flex_filtered_raw)` → `update_plant_schedule()`
7. **LESO** (50 ms, ADRC modes only) → `update_active_b0()` → `leso_update(pressure_ref_kpa, q_fb)`
8. **Outer PI / LADRC** (500 ms) → writes `pressure_ref_kpa`
9. **Pressure PI + pulse** (100 ms) → reads `pressure_ref_kpa`, `pressure_filtered`
10. **CSV row** (50 ms, if logging enabled)

Two consequences that any equivalent implementation must preserve **whenever two loops fall
in the same iteration** (which is not always — §1.6):

- **LESO runs *before* the outer loop.** When both fall in the same iteration, `leso_update()`
  and `update_active_b0()` consume the **previous** `pressure_ref_kpa`; the outer loop then
  overwrites it; the pressure PI in the **same** iteration consumes the **new** value.
- **UKF runs before LESO.** When both fall in the same iteration, `update_plant_schedule()`
  has already moved `target_b0` before `update_active_b0()` filters toward it in that tick.

Note that nothing below the 10 ms sample gate executes on a non-sample iteration, so every
period is effectively quantised to the 10 ms grid.

### 1.6 Deadline gating and phase — loops do NOT stay aligned

Each deadline follows the same rule, but only advances **when its own gate is true**:

```c
if (gate && (int32_t)(now - next_X) >= 0) {
    <body>
    next_X += P;
    if ((int32_t)(now - next_X) >= P) next_X = now + P;   /* catch-up clamp */
}
```

| Counter | Gate | Advances while disabled? |
|---|---|---|
| `next_sample` | none | always advances |
| `next_ukf` | none | always advances |
| `next_eso` | `(mode==ADRC ‖ mode==ADRC_UKF) ∧ control_enabled` | **no** — `main.c:3278-3284` |
| `next_outer` | `control_enabled ∧ mode ∈ {PI, PI_UKF, ADRC, ADRC_UKF}` | **no** — `main.c:3325-3336` |
| `next_pressure` | `control_enabled ∧ mode != CTRL_NONE` | **no** — `main.c:3440-3447` |
| `next_log` | `logging_enabled` | **no**, but is *rebased* to `now + 50` on every idle sample tick (`main.c:3502-3512`) |

**A disabled loop's deadline freezes in the past.** When its gate becomes true again, the
behaviour is **a property of the algorithm above, not of any summarising model.** An earlier
revision of this document claimed a fixed three-regime classification keyed on the overdue
amount `D = now − next_X`, including a guaranteed "two-execution burst". **That claim is
withdrawn: it is false.**

Counterexample, reproduced by running the algorithm literally with `P = 500 ms` and a deadline
frozen at `next_X = 0`, gate becoming true at `t = 990 ms`:

```
tick  990: 990 - 0    = 990 ≥ 0  → EXECUTE; next = 500;  990 -  500 = 490 < 500 → no clamp
tick 1000: 1000 - 500 = 500 ≥ 0  → EXECUTE; next = 1000; 1000 - 1000 =  0 < 500 → no clamp
tick 1010: 1010 - 1000 = 10 ≥ 0  → EXECUTE; next = 1500; 1010 - 1500 < 0        → no clamp
tick 1020…1490:                     no execution
tick 1500:                          EXECUTE; next = 2000
```

giving executions at **990, 1000, 1010, 1500** — *three* consecutive executions, not two, and
no rebasing at all. Neighbouring cases differ again: the same frozen deadline with the gate
becoming true at `t = 980` yields `{980, 990, 1000, 1500}`, while at `t = 1000` the clamp does
fire and yields a single execution then `{1000, 1500, 2000}`.

The reason no simple regime model holds is that `now` advances by 10 ms per evaluation while
`next_X` advances by `P` per *execution*, and the clamp is re-tested after **each** single
increment. How many executions occur, and whether the clamp ever fires, depends jointly on the
exact stored deadline and the exact current time.

**No closed-form re-entry model is offered here, because none has been proven against the
firmware.** The rule is the specification. An implementation must execute it; it must not
model it.

Consequences that invalidate any assumption of persistent alignment:

1. **One `now` per iteration.** `now = millis_now()` is read once at the top of the loop
   (`main.c:3128`) and is **not refreshed** afterwards, even though the two ADC reads consume
   ≈1.6 ms. Every deadline comparison in one iteration therefore uses the same timestamp, and
   `experiment_time_ms(now)` in the CSV row is that same value.

2. **All scheduler evaluation sits below the 10 ms sample gate.** If the sample deadline is
   not due, the iteration does `sleep_ms(1); continue;` (`main.c:3147-3155`) and **no** other
   deadline is examined. Consequently every loop can only ever fire at a 10 ms sample tick,
   and all phase offsets between loops are multiples of 10 ms.

3. **`next_ukf` is ungated and must be described separately.** The UKF block
   (`main.c:3247-3272`) has no gate, so its deadline advances on every sample tick where it is
   due and never freezes on account of controller mode or logging state. It is therefore the
   only control-relevant loop that stays on the grid established at `main.c:3105-3124`
   — *except* that it is still subject to the same increment-and-clamp rule whenever the main
   loop itself is blocked (next point). Note also that the UKF block is what calls
   `update_plant_schedule()` at 50 ms, so gain-schedule refresh follows the **UKF** deadline,
   not the controller deadlines.

4. **Blocking sleeps inside command handling stall every deadline.** `CALIBRATE` runs
   `perform_full_calibration(RECAL_VENT_MS)` with `sleep_ms(10000)` (`main.c:894`), and
   `END_RUN` performs `sleep_ms(FINAL_VENT_MS)` = 8000 ms (`main.c:2329`), both from inside
   `poll_serial_commands()`. During those windows the main loop does not turn, so on the next
   iteration `next_sample` and `next_ukf` are ~10 s or ~8 s overdue and run through the
   algorithm above. The one-time reset of all six counters at `main.c:3105-3124` happens only
   **before** the loop is entered, so a mid-run `CALIBRATE` does **not** restore the original
   grid.

5. **`next_eso` freezes whenever the mode is not ADRC/ADRC_UKF, while `next_ukf` does not.**
   After any such interval the two deadlines are related only through the algorithm, and the
   "UKF before LESO" ordering of §1.5 applies only in whichever iterations both happen to be
   due. It is **not** a standing guarantee.

6. **Entering a controller mode from `CTRL_NONE`** makes `next_outer` and `next_pressure` (and
   in ADRC modes `next_eso`) eligible again at the same sample tick. What each deadline then
   does follows from its own stored value; they are not reset by `select_controller_mode()`,
   which has no access to them — they are locals of `main()`.

7. **Switching between already-enabled modes re-aligns nothing.** `PI → ADRC` leaves
   `next_outer`/`next_pressure` where PI left them while `next_eso` re-enters from its frozen
   value. `CTRL_PRESSURE` stops `next_outer` (the outer gate excludes that mode) while
   `next_pressure` keeps advancing, so `PRESSURE → PI` can desynchronise those two as well.

8. **`next_log` is rebased to `now + 50` on every idle sample tick** (`main.c:3502-3512`) —
   the one counter with an explicit rebase outside its own block. The first CSV row after
   `START_LOG` therefore lands ~50 ms later and the log grid is anchored to the `START_LOG`
   instant, not to `t0`.

**Migration consequence (baseline equivalence).** A ROS 2 baseline-equivalent implementation
must reproduce the firmware deadline-update algorithm **literally**: six integer counters, one
`now` sampled once per iteration, a 10 ms gate above all other deadline evaluation, the
`next_X += P` increment, and the `if ((int32_t)(now - next_X) >= P) next_X = now + P` clamp
re-tested after each single increment — inside **one** callback.

**Independent `create_wall_timer()` callbacks are NOT baseline-equivalent.** Free-running ROS
timers never freeze, never accumulate a backlog, never produce consecutive-tick catch-up
executions, and never rebase. Four such timers would hold the loops in fixed relative phase
forever, which the firmware does not do. This is tested by T15 (§8).

**Scope of this requirement after the architecture decision (§6, §13).** Everything in §1.5
and §1.6 governs the **control scheduler inside `glove_control_node`**, which owns the sample,
LESO, outer, pressure and log deadlines and must reproduce the algorithm literally. The UKF
deadline is the one counter that leaves the control node: `ukf_node` is driven by
`EstimatorInputFrame` arrival instead (§7.3). That substitution is a **known deviation** — it
preserves the validated *cadence* and the validated *`dt` semantics* (both derived from the
Pico acquisition clock) but cannot preserve same-process ordering between the estimator update
and its consumers. It is recorded as Claim B in §6.4 and is measured, never assumed.

*Uncertainty:* whether any of this re-phasing materially changed the validated experimental
results is **not determinable from the files** — it depends on the exact command timing of
each recorded run, which is not preserved anywhere in the repository.

---

## 2. Timing and period constants

| Loop | Period | Ts constant | Source |
|---|---|---|---|
| Sample / sensors | 10 ms | — | `SAMPLE_PERIOD_MS` `main.c:121` |
| UKF4 | 50 ms | `dt` measured, fallback 0.050 s | `UKF_SHADOW_LOOP_MS` `ukf_shadow_rp2040.h:70` |
| LESO | 50 ms | `ESO_TS_S = 0.050` | `main.c:182-183` |
| Outer PI / LADRC | 500 ms | `OUTER_TS_S = 0.500` | `main.c:164-165` |
| Pressure PI | 100 ms | `PRESSURE_TS_S = 0.100` | `main.c:200-201` |
| CSV log | 50 ms | — | `LOG_PERIOD_MS` `main.c:122` |
| Reversal lockout | 200 ms | — | `REVERSAL_LOCKOUT_MS` `main.c:219` |

**UKF `dt` is measured, not nominal** (`ukf_shadow_rp2040.h:1321-1331`):
`dt = (now32 - last_update_us) * 1e-6`, with `dt = 0.050` on first call or if
`!isfinite(dt) || dt <= 0 || dt > 0.20`. This is the only loop whose step size depends on
real elapsed time — it is a direct source of host-vs-firmware divergence (§8.1a, §8.6).

ADC acquisition cost per sample tick: `read_adc_average()` (`main.c:547`) does
`sleep_us(10)` + N × `sleep_us(200)`. With `ADC_AVERAGE_SAMPLES = 4` that is **810 µs per
channel**, ~1.62 ms for both — ~16 % of the 10 ms budget. Zero calibration uses 500 samples
≈ 100 ms per channel.

---

## 3. Functional block inventory and target classification

Classification: **A** = move to ROS 2 · **B** = remain on Pico · **C** = split.

The **Runs in** column maps each block onto the approved three-node architecture (§6.1,
§13). `core` means the algorithm lives in the `soft_glove_core` library and is *called by* the
named node — `soft_glove_core` and `glove_interfaces` are **not** runtime nodes.

| # | Block | Class | Runs in | Rationale |
|---|---|---|---|---|
| 1 | Sensor acquisition (ADC timing, averaging) | **B** | Pico | Deterministic 10 ms sampling and `sleep_us` pacing cannot survive a non-RT transport. |
| 2 | Pressure conversion (raw→Vout→kPa) | **B** | Pico | The 100 Hz hard-pressure trip depends on it locally; the zero must live where the trip lives. |
| 3 | Pressure EWMA filter | **B** | Pico | α is defined against a fixed 10 ms cadence; relocating it couples the time constant to link jitter. |
| 4 | FLEX processing (ADC, EWMA, counts→%) | **C** | Pico (ADC+EWMA) · `glove_control_node` (counts→%) | `ukf_node` consumes FLEX in **counts**; only the controller needs percent. |
| 5 | BNO055 acquisition | **B** | Pico | I²C register reads and init sequence are hardware-local. |
| 6 | UKF4 estimator | **A** | `ukf_node` (core) | Dedicated estimator node per the approved decision. **Runs in all four combinations** — see block 9. |
| 7 | Hysteresis / measurement LUT | **A** | `ukf_node` (core) | Inseparable from the UKF measurement model. |
| 8 | UKF reference calibration (q_rel0 averaging) | **C** | Pico streams pairs · `ukf_node` averages and owns `reference_valid` | Sequenced by `glove_control_node` (§7.4). |
| 9 | Gain scheduling (`plant_schedule_eval`) | **A** | `glove_control_node` (core) | Consumes estimator output but **writes controller state**; two of its three firmware call sites are command events (§4.6). **Consumes `s_hat`/`eta_hat` in every mode, including FLEX feedback** (`main.c:945-951`, `GAIN_SCHEDULING.md:29`) — it is *not* gated by `feedback_source` (§6.5). |
| 10 | External position PI | **A** | `glove_control_node` (core) | Internal module, **not** a separate node. |
| 11 | LADRC | **A** | `glove_control_node` (core) | Internal module, **not** a separate node. |
| 12 | LESO observer | **A** | `glove_control_node` (core) | Internal module, **not** a separate node. |
| 13 | b0 smoothing + z2 compensation | **A** | `glove_control_node` (core) | Internal LADRC state; must stay with the LESO. |
| 14 | ADRC step feedforward | **A** | `glove_control_node` (core) | Latched on `SET_Q`, which the controller owns. |
| 15 | P_ref rate limiter + saturation | **A** | `glove_control_node` (core) | Shared by PI and ADRC. |
| 16 | Pressure PI (deadband, anti-windup) | **A** | `glove_control_node` (core) | **One shared inner loop** for all four combinations. |
| 17 | FILL/HOLD/VENT decision | **A** | `glove_control_node` (core) | Control logic derived from the pressure PI sign. |
| 18 | Valve GPIO level mapping | **B** | Pico | Low-level GPIO. |
| 19 | Pulse duration computation | **A** | `glove_control_node` (core) | `magnitude_to_pulse()` is arithmetic on the PI output. |
| 20 | Pulse execution / timing / expiry | **B** | Pico | Precise pulse timing is explicitly a Pico responsibility. |
| 21 | Reversal lockout | **C** | `glove_control_node` decides · Pico guards | Independent *rejecting* guard on Pico (§7.7). |
| 22 | Calibration sequencing (vent → zero → init) | **C** | `glove_control_node` sequences · Pico primitives · `ukf_node` reference | Three-party sequence (§7.4). |
| 23 | Safety / overpressure trip | **C** | Pico (authoritative) · `glove_control_node` (policy) | Redundant check must not replace the local trip. |
| 24 | Communication watchdog | **B** | Pico | **New** — no reference behaviour exists (§10, item 13). |
| 25 | Serial command processing | **C** | `glove_control_node` (semantics) · `pico_bridge_node` (framing) | Command grammar is an experiment-interface concern. |
| 26 | Telemetry / CSV | **C** | Pico emits frames · `glove_control_node` writes the 87-column CSV | A dedicated logger node is **deferred**, not part of the baseline (§6.2). |
| 27 | Supervisory state logic (modes, bumpless init, END_RUN, ABORT) | **A** | `glove_control_node` | No separate supervisor node in the baseline. |
| 28 | **Feedback selection** (`active_position_feedback_pct`) | **A** | `glove_control_node` (core) | New explicit module; firmware behaviour at `main.c:466-483`. Selects `q_feedback` **only** — has no effect on block 9 (§6.5). |

---

## 4. Block detail — constants, state, equations

### 4.1 Sensor acquisition — class B

```
SAMPLE_PERIOD_MS     = 10          main.c:121
ADC_AVERAGE_SAMPLES  = 4           main.c:123
```
`read_adc_average(ch, n)`: `adc_select_input(ch)`, `sleep_us(10)`, then n × (`adc_read()`,
`sleep_us(200)`), returns `sum / n` as `float` (`main.c:547-561`).

### 4.2 Pressure conversion and filtering — class B

```
ADC_REF_VOLTAGE   = 3.3            main.c:136
ADC_MAX_VALUE     = 4095.0         main.c:137
DIVIDER_GAIN      = 1.545          main.c:138
KPA_PER_VOLT      = 50.0           main.c:139
PRESSURE_EWMA_ALPHA = 0.20         main.c:125
PRESSURE_ZERO_SAMPLES = 500        main.c:140
```
```
vout(raw)  = (raw * 3.3 / 4095.0) * 1.545                 main.c:563-570
kPa(raw)   = (vout(raw) - pressure_zero_vout) * 50.0      main.c:572-577
pressure_filtered = 0.20*kPa + 0.80*pressure_filtered     main.c:587-596, 3204
```
State: `pressure_zero_raw`, `pressure_zero_vout`, `pressure_filtered`,
`latest_pressure_raw/_vout/_kpa`.
**The overpressure trip tests `latest_pressure_kpa` (unfiltered)**, `main.c:3191-3202`.

### 4.3 FLEX processing — class C

```
FLEX_SPAN_COUNTS  = 494.794        main.c:133
FLEX_ZERO_SAMPLES = 500            main.c:134
FLEX_EWMA_ALPHA   = 0.20           main.c:126
```
```
q_unclipped(raw) = 100 * (flex_zero_raw - raw) / 494.794   main.c:579-585
flex_filtered_raw = 0.20*raw + 0.80*flex_filtered_raw      main.c:3217
```
**Asymmetry that must be preserved** (`main.c:3224-3236`):
- `latest_position_unclipped` is computed from the **raw** FLEX sample (telemetry only);
- `latest_position_pct` is computed from the **filtered** FLEX sample, then clamped to [0, 100],
  and is the FLEX control feedback;
- the **UKF** receives `flex_filtered_raw` (counts, not %), `main.c:3253-3255`.

### 4.4 BNO055 acquisition — class B

Init sequence (`ukf_shadow_rp2040.h:594-626`): verify CHIP_ID `0xA0`; OPR_MODE ← CONFIG;
`sleep_ms(25)`; PAGE_ID ← 0; UNIT_SEL ← 0x00; PWR_MODE ← 0x00; `sleep_ms(10)`;
SYS_TRIGGER ← 0x00; `sleep_ms(10)`; OPR_MODE ← NDOF (`0x0C`); `sleep_ms(30)`.

Quaternion read: 8 bytes from `QUA_DATA_W_LSB` (`0x20`), little-endian int16, scale
**1/16384**, then normalise; rejected unless all four components are finite
(`ukf_shadow_rp2040.h:628-658`).

### 4.5 UKF4 — class A

State `x = [s, sdot, eta, bF]`, measurement `z = [FLEX_raw, theta_signed]`.

```
UKFS_C     = 0.16      UKFS_WM0 = -24.0    UKFS_WC0 = -21.04    UKFS_WI = 3.125
```
(from α=0.2, β=2, κ=0, n=4 ⇒ λ = −3.84, c = 0.16; verified: Wm0=λ/c, Wc0=λ/c+(1−α²+β), Wi=1/2c)

```
UKFS_SIGMA_ACC     = 50.0      UKFS_SIGMA_ETA_RW  = 0.06
UKFS_SIGMA_BIAS_RW = 0.005     UKFS_R_SCALE       = 0.02
```
η-dynamics coefficients (`ukf_shadow_rp2040.h:124-129`):
```
a1 = -0.03967525   a2 = -0.01119239   a3 =  0.02141572
a4 = -0.01612887   a5 = -0.10246220   a6 = -0.01530813
```
Process model (`ukf_shadow_rp2040.h:676-710`):
```
s⁺    = s + dt*v
v⁺    = v
ζ     = 2η - 1 ;  sN = (s-50)/50
dζ/dt = a1*v + a2*|v|*ζ + a3*v*|ζ| + a4*ζ + a5*sN + a6
ζ⁺    = clamp(ζ + dt*dζ/dt, -1, 1)
η⁺    = 0.5*(ζ⁺ + 1)
bF⁺   = bF
```
Process noise (`ukf_shadow_rp2040.h:712-736`): `Q00=σa²dt⁴/4`, `Q01=Q10=σa²dt³/2`,
`Q11=σa²dt²`, `Q22=σ_η²dt`, `Q33=σ_b²dt`.

Measurement model (`ukf_shadow_rp2040.h:410-433`):
```
F̂ = (1-η)·F_UP(s) + η·F_DOWN(s) + bF
θ̂ = (1-η)·θ_UP(s) + η·θ_DOWN(s)
```
Measurement variance (`ukf_shadow_rp2040.h:435-467`): per-branch σ floors
`SIGMA_F_MIN=5.0`, `SIGMA_T_MIN=1.0` applied **before** blending; variances blended linearly
in η; floored again; then **multiplied by `R_SCALE = 0.02`**.

State constraints (`ukf_shadow_rp2040.h:133-140, 662-674`):
```
s ∈ [0,100]   v ∈ [-250,250]   η ∈ [0,1]   bF ∈ [-80,80]
plus: s≤0 ∧ v<0 ⇒ v=0 ;  s≥100 ∧ v>0 ⇒ v=0
```
Initial covariance (`ukf_shadow_rp2040.h:149-152`): `P0 = diag(16, 225, 0.0625, 225)`.

Numerical guards: `P_FLOOR = 1e-7`, `S_FLOOR = 1e-8`. There are **two independent fallback
ladders**, both of which must be ported:

1. `ukfs_repair_cov4()` (`ukf_shadow_rp2040.h:321-389`) symmetrises, floors diagonals to
   `P_FLOOR`, limits off-diagonals to `±0.995·√(max(Pii·Pjj, P_FLOOR))`, then diagonal-loads
   with jitter `1e-6 × 10ᵏ` for up to 8 attempts, finally falling back to a **diagonal** matrix
   with floor `1e-3`.
2. `ukfs_generate_sigma_points()` (`ukf_shadow_rp2040.h:997-1048`) forms `A = C·P` and
   attempts Cholesky; on failure it adds `1e-5` to `A`'s diagonal and retries; on a second
   failure it abandons Cholesky and uses `L = diag(√max(Aii, P_FLOOR))`.

Two further details that a "clean" refactor would silently change:

- **`repair_cov4` mutates the stored covariance.** `ukfs_generate_sigma_points()` receives
  `ukfs_P` itself and repairs it in place — it is a side effect, not a local copy.
- **The centre sigma point is not re-constrained.** `X[0] = x` is copied verbatim, while every
  `X[1..8]` column pair passes through `ukfs_constrain_state()` (`ukf_shadow_rp2040.h:1031-1047`).

Update path: covariance update is `P = Ppred − K·S·Kᵀ` (not Joseph form),
`ukf_shadow_rp2040.h:1240-1250`.

**Two fallbacks to prediction-only** (`ukfs_prediction_only`, sets `valid=false`,
innovations and NIS to `NaN`):
1. `!theta_valid || !isfinite(theta) || !isfinite(flex)` — a FLEX-only update is
   deliberately **never** performed (`ukf_shadow_rp2040.h:1129-1137`);
2. `!isfinite(det) || det < S_FLOOR` (`ukf_shadow_rp2040.h:1198-1204`).

If `reference_valid == false`, `ukfs_update_filter()` is not called at all and
`valid = false` (`ukf_shadow_rp2040.h:1333-1345`).

**θ is a continuously unwrapped accumulator** (`ukf_shadow_rp2040.h:945-993`) —
stateful and strictly order-dependent:
```
q_rel   = conj(q1) ⊗ q2                          (normalised)
q_delta = conj(q_rel0) ⊗ q_rel                   (normalised, sign-continuous vs. previous)
q_par   = q_delta·n̂ ,  n̂ = (-0.14794015, -0.98835782, -0.03553205)   ukf_model_lut.h:27-31
half    = atan2f(q_par, q_delta.w)
dhalf   = wrap_to_±π(half - half_prev)
half_accum += dhalf ;  θ_deg = 2·half_accum·180/π
```

**LUT inventory — `ukf_model_lut.h` contains eight 101-entry tables** (grid `s = 0…100`,
`UKF_MODEL_DS = 1.0`, `UKF_MODEL_N = 101`), plus one 3-element axis vector and one derived
macro. An earlier version of this document said six; that was wrong — the two
`SIGMA_THETA_*` tables were omitted.

| # | Symbol | Line | Length | Role |
|---|---|---|---|---|
| 1 | `UKF_MODEL_F_UP` | 33 | 101 | FLEX counts, UP (loading) branch — measurement mean `F̂`, `η=0` |
| 2 | `UKF_MODEL_F_DOWN` | 53 | 101 | FLEX counts, DOWN (unloading) branch — `η=1` |
| 3 | `UKF_MODEL_THETA_UP_DEG` | 73 | 101 | Signed joint angle (deg), UP branch — measurement mean `θ̂`, `η=0` |
| 4 | `UKF_MODEL_THETA_DOWN_DEG` | 93 | 101 | Signed joint angle (deg), DOWN branch — `η=1` |
| 5 | `UKF_MODEL_SIGMA_F_UP` | 113 | 101 | FLEX measurement **σ**, UP branch — builds `R[0][0]` |
| 6 | `UKF_MODEL_SIGMA_F_DOWN` | 133 | 101 | FLEX measurement **σ**, DOWN branch |
| 7 | `UKF_MODEL_SIGMA_THETA_UP_DEG` | 153 | 101 | θ measurement **σ** (deg), UP branch — builds `R[1][1]` |
| 8 | `UKF_MODEL_SIGMA_THETA_DOWN_DEG` | 173 | 101 | θ measurement **σ** (deg), DOWN branch |
| — | `UKF_MODEL_N_AXIS` | 27 | 3 | Unit rotation axis `n̂` projecting `q_delta` onto the flexion DOF (§ θ accumulator) — **not** indexed by `s` |
| — | `UKF_MODEL_F0_REFERENCE` | 198 | macro | `= UKF_MODEL_F_UP[0]`; **documentation only, never used in any computation** |

Tables 1–4 are consumed by `ukfs_measurement_model()` (`ukf_shadow_rp2040.h:410-433`);
tables 5–8 by `ukfs_measurement_sigma()` (`ukf_shadow_rp2040.h:435-467`). All four pairs are
blended by the same hysteresis state `η`, but note the **different blend algebra**: means are
blended linearly (`(1-η)·up + η·down`), whereas σ pairs are squared to variances first, then
the **variances** are blended linearly, then floored, then scaled by `R_SCALE`. Blending σ
directly instead of σ² would be a behavioural change.

`UKF_MODEL_N_AXIS` is verified unit-norm to float precision and is used only in
`ukfs_update_theta()`; it is not interpolated.

Interpolation (`ukfs_lut`, `ukf_shadow_rp2040.h:393-408`) is shared by all eight tables:
clamps `s` to [0,100], returns `table[100]` exactly when `s ≥ 100`, otherwise `i = (int)s`,
`a = s − i`, linear blend `table[i] + a*(table[i+1] - table[i])`, with index guards
`i<0 → 0` and `i ≥ N-1 → N-2`.

`UKF_MODEL_F0_REFERENCE` exists but is documentation-only: **FLEX is never re-zeroed**; the
session offset is estimated by `bF` (`ukf_model_lut.h:193-198`, `ukf_shadow_rp2040.h:828-834`).

Reference calibration (`ukf_shadow_rp2040.h:828-941`): 40 samples × 20 ms
(`REF_SAMPLES=40`, `REF_SAMPLE_DELAY_MS=20`), sign-aligned quaternion sum against the first
good sample, normalised; **requires `good ≥ 20`** (`REF_SAMPLES/2`); always calls
`ukf_shadow_reset_filter()` afterwards.

Feedback gate (`main.c:455-464`):
```
ukf4_feedback_available() = valid ∧ reference_valid ∧ theta_valid ∧ isfinite(s_hat)
```

### 4.6 Gain scheduling — class A

Anchors (`plant_schedule.h:8-12`), described in the reference as **rounded** identification
values:

| q | K_up | τ_up | K_down | τ_down |
|---|---|---|---|---|
| 30 | 0.818 | 1.906 | 1.342 | 5.262 |
| 50 | 1.170 | 0.177 | 0.831 | 2.970 |
| 70 | 0.945 | 0.119 | 0.520 | 4.194 |

`plant_schedule_eval(q, η)` (`plant_schedule.h:15-42`):
```
reject if !isfinite(q) || !isfinite(η)
qc = clamp(q, 30, 70)        (edge hold — NOT extrapolation)
h  = clamp(η, 0, 1)
i  = 0; while (i+2 < 3 && qc > ANCHORS[i+1].q) ++i;   ⇒ i = (qc > 50) ? 1 : 0
a  = ANCHORS[i]; b = ANCHORS[i+1]
reject unless b.q > a.q and all four K/τ at both anchors are > 0
w  = (qc - a.q) / (b.q - a.q)
log-linear in q, then log-linear blend across branches in η:
  K   = exp( (1-h)*[(1-w)·ln a.K_up  + w·ln b.K_up ] + h*[(1-w)·ln a.K_down  + w·ln b.K_down ] )
  τ   = exp( (1-h)*[(1-w)·ln a.τ_up  + w·ln b.τ_up ] + h*[(1-w)·ln a.τ_down  + w·ln b.τ_down ] )
Ki  = PLANT_P1 / K              (PLANT_P1 = 0.40, plant_schedule.h:5)
Kp  = Ki * τ
b0  = K / τ
clamped = (qc != q) || (h != η)
reject if any of Kp, Ki, b0 is non-finite or ≤ 0
```
`update_plant_schedule()` (`main.c:945-968`) is a **read-only consumer of the UKF**:
```
schedule_input_valid = ukf4_feedback_available() ∧ plant_schedule_eval(s_hat, eta_hat, &next)
if !schedule_input_valid: return        ← hold last schedule, change nothing
active_kp_position = next.kp ; active_ki_position = next.ki
adrc_model_k = next.k ; adrc_model_tau_s = next.tau ; adrc_b0_identified = next.b0
adrc_model_valid = true ; active_gain_region_id = 0
target_b0 = 0.80*B0_NOMINAL + 0.20*next.b0        ← see §11, divergence D1
```
It explicitly does **not** reset PI error history, LESO states, or the feedforward.

Called from three sites: the 50 ms UKF tick (`main.c:3257`), `apply_reference_change()`
(`main.c:1777`), and `select_controller_mode()` for PI/PI_UKF/ADRC/ADRC_UKF
(`main.c:1826, 1844`).

Pre-first-valid defaults (`main.c:345-346, 856-857`) — the q=30 UP anchor:
```
active_kp_position = 0.40 * 1.906 / 0.818 = 0.93202934
active_ki_position = 0.40 / 0.818        = 0.48899756
```

### 4.7 External position PI — class A

Incremental (velocity) form, `position_pi_update()` (`main.c:1108-1198`):
```
e            = q_ref - q_fb
Δp           = Kp * (e - e_prev)
Δi           = Ki * OUTER_TS_S * e          (OUTER_TS_S = 0.500)
ΔP_ref       = Δp + Δi
P_ref_raw    = P_ref + ΔP_ref
P_ref        = apply_common_pref_limits(P_ref_raw, P_ref, ...)
e_prev       = e
```
Because the form is incremental, a gain change produces no output jump at zero error; with
non-zero error the next increment changes (`GAIN_SCHEDULING.md:53`). Gains are **sampled at
execution time**, so the 50 ms schedule updates are consumed by the 500 ms loop.

Feedback source (`active_position_feedback_pct()`, `main.c:466-483`): `clamp(s_hat, 0, 100)`
in `PI_UKF`/`ADRC_UKF` **when `ukf4_feedback_available()`**, otherwise `latest_position_pct`.

Bumpless init (`initialize_pi_bumpless()`, `main.c:757-786`): `e_prev = q_ref − q_fb`, all
`last_position_pi` P_ref fields seeded to the current `pressure_ref_kpa`.

### 4.8 P_ref rate limiter and saturation — class A

Shared by PI and LADRC (`apply_common_pref_limits`, `main.c:1047-1101`):
```
PREF_RATE_UP_KPA_S   = 10.0   →  max_up_step   = 10.0 * 0.500 =  +5.0 kPa / update
PREF_RATE_DOWN_KPA_S = 12.0   →  max_down_step = 12.0 * 0.500 =  -6.0 kPa / update
PRESSURE_REF_MIN_KPA = 0.0 ;  PRESSURE_REF_MAX_KPA = 170.0
saturated  ⇔ |final - limited| > 1e-6
```
Order is **rate-limit first, then clamp**. `pressure_ref_rate_limited_kpa` is recomputed
separately for telemetry only (`main.c:1156-1180`, `1356-1380`).

### 4.9 LESO — class A

```
ESO_OMEGA_O = 2.50   →  ESO_BETA1 = 2*ω_o = 5.00 ;  ESO_BETA2 = ω_o² = 6.25
ESO_TS_S    = 0.050
```
Forward-Euler (`leso_update`, `main.c:1205-1232`):
```
err    = q_measured - z1
ż1     = z2 + b0_active * P_ref + β1 * err
ż2     = β2 * err
z1    += Ts * ż1
z2    += Ts * ż2
```
**The observer input is `pressure_ref_kpa` (the reference), not the measured pressure**
(`main.c:3298-3301`; confirmed `GAIN_SCHEDULING.md:27`).

b0 smoothing (`update_active_b0`, `main.c:1000-1040`):
```
B0_NOMINAL = B0_FALLBACK = 2.20 ;  B0_SMOOTH_TAU_S = 0.50
α      = Ts / (τ + Ts) = 0.050 / 0.550 = 0.0909090909...
b0_new = b0_old + α*(target_b0 - b0_old)
if !isfinite(b0_new) || b0_new <= 1e-5      → b0_new = 2.20
if |target_b0 - b0_new| < 1e-5              → b0_new = target_b0   (snap)
z2 += (b0_old - b0_new) * P_ref             ← preserves q̇̂ = z2 + b0·P_ref
b0_active = b0_new
```

### 4.10 LADRC — class A

```
LADRC_OMEGA_C           = 0.70
LADRC_FF_MODEL_WEIGHT   = 0.30
LADRC_FF_MAX_UP_KPA     = 8.0
LADRC_FF_MAX_DOWN_KPA   = 5.0
LADRC_FF_FULL_ERROR_PCT = 10.0
LADRC_FF_ZERO_ERROR_PCT =  2.0
```
`ladrc_update(q_ref)` (`main.c:1234-1395`):
```
e_pos    = q_ref - z1
P_eq     = -z2 / b0_active
P_fb     = (ω_c / b0_active) * e_pos
P_raw    = P_eq + P_fb + γ*step_ff
P_ref    = apply_common_pref_limits(P_raw, P_ref, ...)
```
Feedforward envelope γ:
```
if |step_ff| <= 1e-6                          → γ = 0
else if sign(step_ff) != sign(e_pos)          → γ = 0     (direction gate)
else:
    γ_cand = 1                                     if |e_pos| >= 10.0
           = (|e_pos| - 2.0) / (10.0 - 2.0)        if 2.0 < |e_pos| < 10.0
           = 0                                     otherwise
    γ_cand = clamp(γ_cand, 0, 1)
    if γ_cand < γ:  γ = γ_cand                ← monotonically non-increasing until next SET_Q
```
Step feedforward (`compute_adrc_feedforward`, `main.c:970-998`), latched on reference change:
```
if !schedule_input_valid || !isfinite(K) || |K| < 1e-5  → ff = 0
ff = 0.30 * (q_to - q_from) / K ,  clipped to [-5.0, +8.0]
```
Set in `apply_reference_change()` (`main.c:1757-1798`), only when `|Δq| > 0.001`, and
`γ` is re-armed to `1.0` iff `|ff| > 1e-6`. Note it is computed **for every mode**, including PI.

Bumpless init (`initialize_adrc_bumpless()`, `main.c:788-835`):
`b0_active = target_b0` (fallback 2.20 if non-finite or ≤ 1e-5), `z1 = q_fb`,
`z2 = −b0_active · P_ref` ⇒ `P_eq = P_ref`, so the first update causes no artificial jump.

### 4.11 Pressure PI — class A

```
KP_PRESSURE           = 0.060      KI_PRESSURE        = 0.025
PRESSURE_TS_S         = 0.100      PRESSURE_DEADBAND_KPA = 1.0
PRESSURE_I_MIN        = -1.0       PRESSURE_I_MAX     = 1.0
```
`pressure_pi_update(ref, p)` (`main.c:1402-1517`):
```
e      = ref - p
p_term = 0.060 * e

if |e| <= 1.0:                       ← deadband early return
    i_term = pressure_integral       (integral FROZEN, not reset)
    output_unsat = 0 ; output = 0 ; RETURN

cand_I = clamp(pressure_integral + 0.025 * e * 0.100, -1.0, +1.0)

mag_unsat_cand = ( e > 0 ) ?  p_term + max(cand_I, 0)
                           : -p_term + max(-cand_I, 0)
if mag_unsat_cand <= 1.0:  pressure_integral = cand_I     ← conditional integration
i_term = pressure_integral

if e > 0:  mag = p_term + max(I, 0)  ; output_unsat = +mag ; output = +clamp(mag, 0, 1)
else:      mag = -p_term + max(-I,0) ; output_unsat = -mag ; output = -clamp(mag, 0, 1)
```
The sign-split magnitude form and the "commit the integral only if the *candidate* magnitude
does not exceed 1" rule are both behaviourally significant and must be transcribed exactly.

### 4.12 FILL / HOLD / VENT, pulses and reversal lockout — classes A / B / C

```
FILL_MIN_MS = 5    FILL_MAX_MS = 80
VENT_MIN_MS = 10   VENT_MAX_MS = 100
REVERSAL_LOCKOUT_MS = 200
```
```
magnitude_to_pulse(m, lo, hi) = (uint32_t)( lo + clamp(m,0,1) * (hi - lo) )   main.c:1524-1546
```
The `(uint32_t)` cast **truncates toward zero** — not rounding.

`execute_pressure_control(ref, p, now)` (`main.c:1605-1750`):
```
last_pressure_pi = pressure_pi_update(ref, p)
last_pulse_ms    = 0
if pulse_active:                        RETURN            ← no new pulse while one is running
if |e| <= 1.0:  pending_reversal = HOLD ; set_state(HOLD) ; RETURN
requested = (e > 0) ? FILL : VENT

if pending_reversal != HOLD:
    if requested != pending_reversal:
        pending_reversal = requested
        block_until = now + 200 ; set_state(HOLD) ; RETURN     ← re-arm, NO log line
    if (now - block_until) < 0:  set_state(HOLD) ; RETURN      ← still locked out
    pending_reversal = HOLD                                    ← lockout satisfied, fall through
elif last_pulse_direction != HOLD and requested != last_pulse_direction:
    pending_reversal = requested
    block_until = now + 200 ; set_state(HOLD)
    print "#PRESSURE_REVERSAL_LOCKOUT,..."                     ← logged only on first detection
    RETURN

pulse = magnitude_to_pulse(|output|, FILL or VENT range)
last_pulse_direction = requested
start_pulse(requested, pulse, now)
```
Pulse execution (`start_pulse`, `main.c:1548-1585`): cancel any armed alarm, `set_state()`,
`pulse_active = true`, `pulse_end_ms = now + pulse_ms`, `add_alarm_in_ms(...)`. **If the alarm
cannot be armed (`id < 0`), the pulse is abandoned, `pulse_alarm_fail_count++`, and the state
falls back to HOLD** — the valve is never left open on an arming failure. Expiry has two
independent paths: the alarm callback (`main.c:603-618`) and the polled
`update_active_pulse()` (`main.c:1587-1603`); both end in `set_state(STATE_HOLD)`.

### 4.13 Calibration — class C

`perform_full_calibration(vent_ms)` (`main.c:877-935`):
```
control_enabled=false ; controller_mode=NONE ; logging_enabled=false
cancel_active_pulse_alarm() ; pump_off() ; set_state(VENT)
sleep_ms(vent_ms)                             ← INITIAL 10000 / RECAL 10000
calibrate_pressure_zero()                     ← 500-sample ADC average → zero_raw, zero_vout
calibrate_flex_zero()                         ← 500-sample ADC average → flex_zero_raw
initialize_filters_from_current_sensors()     ← seeds EWMA states with an unfiltered sample
ukf_shadow_calibrate_reference(flex_zero_raw) ← arg deliberately UNUSED; 40×20 ms quats
pressure_ref_kpa = clamp(pressure_filtered, 0, 170)
position_ref_pct = latest_position_pct ; previous_q_ref_pct = position_ref_pct
reset_pressure_actuation_state() ; reset_controller_state_to_safe_none()
all_outputs_off()                             ← ends with EVERYTHING OFF
```
```
INITIAL_VENT_MS = 10000   RECAL_VENT_MS = 10000   FINAL_VENT_MS = 8000
```

### 4.14 Safety — class C

```
HARD_PRESSURE_KPA = 200.0
```
Checked every 10 ms against **unfiltered** `latest_pressure_kpa` (`main.c:3191-3202`).
`emergency_shutdown(reason, p)` (`main.c:2443-2478`): disable control and logging, mode←NONE,
`pump_off()`, cancel pulse alarm, `set_state(VENT)`, log `#ABORT`, `sleep_ms(8000)`,
`all_outputs_off()`, print `pulse_alarm_fail_count` and `#TEST_FINISHED`. `main()` then
**returns 1 (overpressure) or 2 (operator abort) — the firmware terminates and does not
recover**.

Additional protections: `PRESSURE_REF_MAX_KPA = 170` clamp on every P_ref path;
`SET_P` and `SET_Q` range validation; `CTRL,PI_UKF` / `CTRL,ADRC_UKF` refused unless
`ukf4_feedback_available()`.

UKF-invalidity fail-safes (these are **hold**, not abort):
- `CTRL_PI_UKF` outer tick (`main.c:3343-3367`): `P_ref = clamp(pressure_filtered, 0, 170)`,
  `previous_position_error = 0`, log `#PI_UKF_HOLD`.
- `CTRL_ADRC_UKF` outer tick (`main.c:3384-3408`): `P_ref = clamp(pressure_filtered, 0, 170)`,
  `ff_gamma_state = 0`, log `#ADRC_UKF_HOLD`.
- `CTRL_ADRC_UKF` LESO tick (`main.c:3288-3295`): observer **not** updated;
  `eso_updated_since_log = false`.

### 4.15 Serial command processing — class C

Line-based, `\n` or `\r` terminated, 128-byte buffer; overflow resets the buffer and emits
`#COMMAND_ERROR,REASON=LINE_TOO_LONG` (`main.c:2366-2436`). Commands (`main.c:1955-2364`):

| Command | Effect |
|---|---|
| `START` | logs `#START_RECEIVED` only — no state change |
| `CTRL,PI` / `CTRL,ADRC` / `CTRL,PRESSURE` | `select_controller_mode()`, pump **on**, HOLD |
| `CTRL,PI_UKF` / `CTRL,ADRC_UKF` | same, **refused** if `!ukf4_feedback_available()` |
| `CTRL,NONE` | disable control, cancel pulse, `all_outputs_off()` |
| `SET_Q,<pct>` | validated 0…100; `apply_reference_change()` |
| `SET_P,<kPa>` | **requires** `CTRL_PRESSURE`; validated against [0, 170] (see §11, D2) |
| `MARK,<text>` | timestamped event line; rejects empty text |
| `START_LOG` | **re-emits the CSV header**, then enables logging |
| `STOP_LOG` | disables logging |
| `STATUS` | `#STATUS` + `#UKF4_STATUS` lines |
| `CALIBRATE` | `perform_full_calibration(RECAL_VENT_MS)` → ends CTRL=NONE, outputs off |
| `VENT` / `HOLD` | disable control, mode←NONE, pump off, set state |
| `END_RUN` | disable all, pump off, VENT, `sleep_ms(8000)`, `all_outputs_off()` |
| `ABORT` | sets `command_abort`; handled at the top of the next loop iteration |

`select_controller_mode()` (`main.c:1800-1893`) — note the ordering:
- PI / PI_UKF → `update_plant_schedule()` then `initialize_pi_bumpless()`
- ADRC / ADRC_UKF → **`ff_gamma_state = 0` first** (an old reference must not re-trigger the
  feedforward), then `update_plant_schedule()`, then `initialize_adrc_bumpless()`
- PRESSURE → `q_ref = latest_position_pct`, `P_ref = clamp(pressure_filtered, 0, 170)`,
  `ff_gamma_state = 0`, reset actuation and diagnostics
- **all** modes then `pump_on()` and `set_state(STATE_HOLD)`

### 4.16 Telemetry — class C

87-column CSV at 20 Hz (count verified programmatically from `print_csv_header`,
`main.c:2485-2579`) plus `#`-prefixed event lines. Startup metadata
(`print_startup_metadata`, `main.c:2851-3026`) emits every timing constant, controller gain,
safety threshold and all three `#PLANT_ANCHOR` rows — this is the machine-readable contract
the ROS 2 side should reproduce.

**Telemetry-only / dead state** (must be carried for CSV parity, must not gain behaviour):
`previous_q_ref_pct` (written in 4 places, never read), `active_gain_region_id` (always 0),
`adrc_b0_identified`, `scheduled_plant.{q,eta,clamped}`, `latest_position_unclipped`,
`outer_updated_since_log`, `eso_updated_since_log`, `pressure_updated_since_log`.

---

## 5. Dependency graph

### 5.1 Firmware dataflow (as validated)

```
                    ┌──────────────────────── 10 ms ────────────────────────┐
  ADC pressure ─► vout ─► kPa ─┬─► [OVERPRESSURE TRIP @200 kPa]
                               └─► EWMA(0.20) ─► pressure_filtered ─┐
                                                                    │
  ADC flex ─► EWMA(0.20) ─► flex_filtered_raw ─┬─► q_pct (clamped)  │
                                               │                    │
                    ┌──── 50 ms ───────────────┘                    │
  BNO1,BNO2 ─► q_rel ─► q_delta ─► θ_accum ──┐                      │
                                             ▼                      │
                                   ┌──────────────────┐             │
                                   │      UKF4        │             │
                                   │ s, sdot, η, bF   │             │
                                   └────────┬─────────┘             │
                          s_hat, eta_hat    │  s_hat (if UKF modes) │
                                            ▼                       │
                                 plant_schedule_eval                │
                                  K, τ ─► Kp, Ki, b0                │
                                            │                       │
                    ┌───────────────────────┼───────────┐           │
                    ▼ (50 ms)               ▼ (500 ms)  ▼ (on SET_Q)│
             update_active_b0        ┌─────────────┐  step_ff       │
                    │                │  PI  or     │    │           │
                    ▼                │  LADRC      │◄───┘           │
              leso_update ──► z1,z2 ─►             │                │
                    ▲                └──────┬──────┘                │
                    │ P_ref (previous)      │ P_ref_raw             │
                    │                       ▼                       │
                    │              rate limiter ±5/−6               │
                    │              clamp [0, 170]                   │
                    │                       │                       │
                    └───────────────────────┤ P_ref                 │
                                            ▼            ◄──────────┘
                    ┌──── 100 ms ──── pressure PI (deadband 1 kPa) ──┐
                    │                       │ output ∈ [-1, +1]      │
                    │                       ▼                        │
                    │            reversal lockout (200 ms)           │
                    │                       ▼                        │
                    │            magnitude_to_pulse → ms             │
                    │                       ▼                        │
                    └──── FILL / HOLD / VENT ─► V1, V2 GPIO ─► plant ┘
```

Explicit cross-block dependencies:

| Consumer | Depends on | Note |
|---|---|---|
| UKF4 | `flex_filtered_raw`, both BNO quaternions, `q_rel0`, `dt` | Needs **both** IMUs; θ is a running accumulator |
| Gain scheduling | `s_hat`, `eta_hat`, `ukf4_feedback_available()` | Holds last value on invalid input |
| PI | `Kp`, `Ki`, `q_fb`, `e_prev`, `P_ref` | Gains read at execution time |
| LESO | `b0_active`, **`P_ref`**, `q_fb` | Reference-driven, not measurement-driven |
| LADRC | `z1`, `z2`, `b0_active`, `ω_c`, `step_ff`, `γ` | `P_eq` couples to b0 through `z2` |
| `update_active_b0` | `target_b0`, `P_ref` | Mutates `z2` to preserve `q̇̂` |
| step_ff | `adrc_model_k`, `schedule_input_valid`, `Δq_ref` | Latched on `SET_Q`, all modes |
| Rate limiter | `P_ref` (previous) | Shared by PI and LADRC |
| Pressure PI | `P_ref`, `pressure_filtered`, `pressure_integral` | Integral frozen inside deadband |
| Pulse / reversal | PI `output`, `last_pulse_direction`, `pulse_active`, `now` | Blocked while a pulse runs |
| Safety | `latest_pressure_kpa` (**unfiltered**) | 10 ms cadence |

**Cycles requiring care when distributing:** `P_ref → LESO → LADRC → P_ref` (delayed via the
50 ms observer) and `b0 → z2 → P_eq → P_ref → LESO`. Both are closed through shared globals,
and the delay around each loop is **not a fixed number of ticks** — it depends on whether the
LESO and outer deadlines happen to fall in the same iteration, which depends on the run's
command history (§1.6). Splitting these across ROS nodes with independent timers would change
behaviour — hence the one-control-node rule of §6.3. The "previous `P_ref`" annotation in the
diagram above therefore means *the value in the global at the moment the LESO block runs*,
which is the outer loop's previous output only when the two coincide.

### 5.2 ROS 2 dataflow (approved baseline)

```
                                   RP2040 Pico
          ADC ─ FLEX ─ BNO055 ─ GPIO ─ pulse timing ─ watchdog ─ 200 kPa trip
                                        │  ▲
                        frames (10/50 ms)│  │ ActuatorCommand
                                        ▼  │
                              ┌──────────────────────┐
                              │  pico_bridge_node    │   PicoStatus ──►
                              └───┬──────────────┬───┘
                 SensorFrame      │              │   EstimatorInputFrame
                 (10 ms)          │              │   (50 ms: flex_filtered_raw
                                  │              │    + both quaternions)
              ┌───────────────────┘              └──────────┐
              │                                             ▼
              │                                  ┌──────────────────────┐
              │                                  │      ukf_node        │
              │                                  │  soft_glove_core/ukf │
              │                                  └──────────┬───────────┘
              │                                             │ EstimatorState
              │  FLEX position                              │ (s_hat, v_hat, eta_hat,
              │  candidate                                  │  b_f_hat, valid flags,
              │  (q_pct from                                │  diagnostics, source_seq,
              │   flex_filtered_raw)                        │  source_timestamp_us)
              │                                             │
              │                          ┌──────────────────┴────────────────┐
              │                          │                                   │
              │                   position candidate                 SCHEDULING STATE
              │                   (s_hat, only if                    (s_hat, eta_hat,
              │                    feedback_source=ukf)               validity) — used in
              │                          │                            ALL FOUR modes
              ▼                          ▼                                   │
        ┌──────────────────────────────────────────────────────────────┐     │
        │                     glove_control_node                       │     │
        │  ┌────────────────────────────────────────────────────────┐  │     │
        │  │ age / sequence check (§7.6)                            │  │     │
        │  │ feedback selector — chooses q_feedback ONLY            │  │     │
        │  │        feedback_source = flex | ukf                     │  │     │
        │  └───────────────────────────┬────────────────────────────┘  │     │
        │                      q_feedback                              │     │
        │  gain scheduling ────────────┤ ◄──────────────────────────────────┘
        │  K, τ → Kp, Ki, b0, step_ff  │                               │
        │  (ALWAYS from EstimatorState,│                               │
        │   independent of             ▼                               │
        │   feedback_source) controller_type = pi | adrc               │
        │                     PI  ──┐   ┌── LADRC + LESO               │
        │                           ▼   ▼                              │
        │                        P_ref_raw                             │
        │                            │                                 │
        │            ONE shared rate limiter (±5 / −6, clamp [0,170])  │
        │                            ▼                                 │
        │                          P_ref                               │
        │                            ▼                                 │
        │            ONE shared pressure PI (deadband 1 kPa)           │
        │                            ▼                                 │
        │       reversal lockout → magnitude_to_pulse → FILL/HOLD/VENT │
        │  supervisory logic · calibration sequencing · 87-column CSV  │
        └──────────────────────────────┬───────────────────────────────┘
                                       │ ActuatorCommand
                                       ▼
                              pico_bridge_node ──► Pico
```

Boundaries crossed that the firmware did not cross, each a **measurement obligation**, not an
equivalence claim (§6.4 Claim B):

| Boundary | What it delays | Firmware counterpart |
|---|---|---|
| Pico → bridge → controller | `pressure_filtered`, `pulse_active` | Direct global read, zero latency |
| Pico → bridge → `ukf_node` | UKF measurement inputs | Synchronous I²C read inside `ukf_shadow_update()` |
| `ukf_node` → controller | `s_hat`, `eta_hat`, validity | Direct read of `ukfs_telem` in the same iteration |
| controller → bridge → Pico | Actuator command | Direct `gpio_put()` + `add_alarm_in_ms()` |

The `ukf_node` → controller hop is the one with no firmware analogue at all, and it carries
**two logically distinct signals**:

| Signal from `EstimatorState` | Consumed by | In which modes |
|---|---|---|
| **Scheduling state** — `s_hat`, `eta_hat`, validity | `plant_schedule_eval` → `Kp`, `Ki`, `K`, `τ`, `b0`, `step_ff` | **All four** |
| **Position candidate** — `s_hat` | Feedback selector → `q_feedback` | Only `feedback_source = ukf` |

`feedback_source` gates the **second row only**. The first row is unconditional: the validated
firmware indexes the shared plant model by `s_hat` and `eta_hat` *even in* `CTRL,PI` and
`CTRL,ADRC` (`main.c:945-951`; `GAIN_SCHEDULING.md:29`). `ukf_node` therefore runs in every
baseline combination. §6.5, §7.6 and §7.8 govern this hop; T23–T25 measure it.

---

## 6. ROS 2 architecture — approved baseline

The architecture below is a **user-approved decision taken before implementation** (recorded in
full in §13). It supersedes the earlier five-package / four-node sketch.

### 6.1 Runtime node topology — exactly three nodes

| Node | Package | Responsibility |
|---|---|---|
| `pico_bridge_node` | `pico_bridge` | Talk to the RP2040: receive sensor/acquisition frames, publish them, accept `ActuatorCommand` from the controller and forward it, expose Pico status and safety/communication status |
| `ukf_node` | `glove_estimator` | Thin ROS wrapper around the validated UKF4 in `soft_glove_core`; consumes estimator inputs, publishes `EstimatorState` |
| `glove_control_node` | `glove_controller` | Feedback selection, gain scheduling, PI **or** ADRC outer loop, LESO, the single shared pressure loop, pneumatic/state-command path, supervisory logic, calibration sequencing, CSV |

`soft_glove_core` and `glove_interfaces` are a **library** and an **interface package**. They
are not runtime nodes and never appear in a launch graph.

No `experiment_manager`, GUI, logger or visualisation node exists in the baseline. The
87-column CSV is a **module inside `glove_control_node`**; promoting it to a node is deferred.

### 6.2 Package layout

```
ros2_ws/src/
  glove_interfaces/             # .msg / .srv only — NOT a node
    msg/  SensorFrame.msg  EstimatorInputFrame.msg  EstimatorState.msg
          ActuatorCommand.msg  PicoStatus.msg  ControllerStatus.msg
    srv/  CalibrateSensors.srv  CalibrateUkfReference.srv  SelectMode.srv

  soft_glove_core/              # NO ROS, NO Pico dependencies — NOT a node
    include/soft_glove_core/
      ukf_model_lut.h           # byte-identical copy of the golden LUT (8 tables)
      plant_schedule.h          # byte-identical copy
      ukf4.h / ukf4.c           # SDK calls removed, algorithm untouched
      feedback_select.h/.c      # active_position_feedback_pct() semantics (§6.5)
      position_pi.h/.c
      ladrc.h/.c                # LADRC + LESO + b0 smoothing + feedforward
      pressure_pi.h/.c
      pref_limiter.h/.c
      pulse_logic.h/.c          # magnitude_to_pulse + reversal FSM
      pneumatic.h/.c            # FILL/HOLD/VENT + valve level mapping
      supervisor.h/.c           # mode transitions + bumpless init/reset semantics
      conversions.h/.c          # ADC↔kPa, counts↔%, EWMA
      scheduler.h/.c            # 10 ms tick order (§1.5) + deadline algorithm (§1.6)
    CMakeLists.txt              # plain CMake library, float32, no fast-math

  pico_bridge/                  # pico_bridge_node
  glove_estimator/              # ukf_node
  glove_controller/             # glove_control_node + launch/ + config/

tests/equivalence/              # host-side goldens and replay harness (§8)
pico_fw/                        # new Pico firmware (NOT the golden reference)
```

Rationale for `soft_glove_core`: **equivalence cannot be tested through ROS nodes.** Isolating
the frozen algorithms in a dependency-free library lets every constant and equation be
unit-tested against host-compiled reference code, and keeps every node a thin, reviewable
wrapper. This is a packaging decision, not a control change.

*Open point (§10 item 16):* launch files and parameter YAML are placed in
`glove_controller/launch` and `glove_controller/config` because the approved scaffold lists
five packages and a bringup package is not among them. A separate `glove_bringup` package is
the conventional ROS 2 alternative and is deferred pending user preference.

### 6.3 Why the control path remains ONE node — finding retained

This was established before the architecture decision and **remains in force**. The scheduler,
gain scheduling, observer, outer loop and pressure loop share mutable state and a strict
intra-iteration ordering (§1.5, §5.1). There must be **no** separate runtime node for the PI
controller, the ADRC controller, the LESO, the pressure PI, gain scheduling or the pneumatic
state logic. They are internal modules of `glove_control_node`, backed by `soft_glove_core`.

`glove_control_node` runs **one** callback on a 10 ms period that reproduces the firmware
deadline-update algorithm literally (§1.6): integer counters, one `now` per iteration, the
10 ms gate above all deadline evaluation, `next_X += P`, and the clamp re-tested after each
single increment. **Independent `create_wall_timer()` callbacks for the control loops are NOT
baseline-equivalent** and are prohibited.

### 6.4 The UKF as a separate node — what it buys and what it costs

The approved decision places the UKF in its own node. The mathematics stays in
`soft_glove_core`, so `ukf_node` is only a wrapper: it deserialises inputs, calls the
estimator, and publishes `EstimatorState`. This keeps the estimator independently testable
without ROS 2 (§8) and keeps a heavy, fixed-cost computation off the control callback.

It also introduces a boundary that **did not exist in the firmware**, where
`ukf_shadow_update()` was a direct function call inside the same 10 ms iteration as the
controller, writing to shared globals that the controller read with zero latency. Crossing a
DDS/executor boundary adds serialisation, transport and scheduling latency, and removes the
guarantee that the estimator update and its consumers occur in a fixed order within one tick.

Two claims must therefore be kept strictly apart for the rest of this document:

| | Claim | How it is established |
|---|---|---|
| **A** | **Algorithmic equivalence of the UKF** — for a given input sequence (`flex_filtered_raw`, quaternions, `theta_valid`, `dt`) and a given initial state, the ported estimator reproduces the firmware estimator | Tier 1/2 golden fixtures, `soft_glove_core` tested **without ROS 2** (T1–T5) |
| **B** | **System-level timing equivalence after ROS 2 integration** — the controller consumes estimator state at the same effective cadence and age as the firmware did | **Not claimed.** Measured experimentally, bounded, and reported (T23–T25, Stage 7) |

**Claim A is achievable and is a hard gate. Claim B is not claimed and must never be asserted
from code structure.** Every statement about cross-node timing in this plan is a measurement
obligation, not an equivalence assertion.

### 6.5 Controller type and feedback source — two independent selections

`glove_control_node` exposes two orthogonal parameters:

```
controller_type : pi | adrc     # which external controller computes P_ref
feedback_source : flex | ukf    # which signal is used as q_feedback — AND NOTHING ELSE
```

> ### ⚠ `feedback_source` selects the position feedback only. It does not disable the UKF.
>
> `ukf_node` runs in **all four** baseline combinations. `feedback_source` chooses which signal
> is handed to the external PI/ADRC controller as `q_feedback`. It has **no effect** on gain
> scheduling, which consumes the UKF-derived scheduling state `s_hat` / `eta_hat` in every mode.
>
> This is validated firmware behaviour, not an inference:
>
> - `update_plant_schedule()` reads `ukf_shadow_get()` and gates on `ukf4_feedback_available()`
>   unconditionally, with no reference to `controller_mode` (`main.c:945-951`);
> - `GAIN_SCHEDULING.md:29` states it explicitly — *"El modelo compartido se indexa por s_hat y
>   eta_hat **incluso en CTRL,PI y CTRL,ADRC**. Esos dos modos conservan FLEX como
>   realimentación de control, pero el scheduling ahora también depende de disponer de UKF
>   válido. **No se introduce un detector alternativo de dirección.**"*;
> - the validated operating procedure requires `gs_valid = 1` before a trial begins and says
>   plainly *"No comenzar pruebas del nuevo scheduling con gs_valid=0"*
>   (`GAIN_SCHEDULING.md:11, 33`).
>
> What the UKF supplies to a FLEX-feedback run is therefore not decorative. Through
> `plant_schedule_eval` it sets `Kp`, `Ki`, `K`, `τ` and `b0` — and through `b0` it sets
> `target_b0`, hence `active_b0`, hence the LESO and the whole LADRC output (§4.6, §4.9). It
> also gates the ADRC step feedforward, which is latched with `adrc_model_k` and
> `schedule_input_valid` on every `SET_Q` regardless of mode (`main.c:1782-1783`).
>
> **Do not describe `feedback_source = flex` as meaning `ukf_node` is unnecessary.** An earlier
> revision of this document did; that was wrong and is corrected throughout (§13.7).

Giving four combinations that correspond in intent to the firmware's modes:

| `controller_type` | `feedback_source` | Firmware mode | Firmware enum |
|---|---|---|---|
| `pi` | `flex` | `CTRL,PI` | `CTRL_PI = 1` |
| `pi` | `ukf` | `CTRL,PI_UKF` | `CTRL_PI_UKF = 4` |
| `adrc` | `flex` | `CTRL,ADRC` | `CTRL_ADRC = 2` |
| `adrc` | `ukf` | `CTRL,ADRC_UKF` | `CTRL_ADRC_UKF = 5` |

The firmware's remaining modes are not combinations: `CTRL,PRESSURE` (`= 3`, open outer loop)
and `CTRL,NONE` (`= 0`) are separate supervisory states, carried as such.

**There are not four controllers.** There is one pipeline with two selectors, and one
unconditional scheduling input that neither selector gates:

```
   EstimatorState ─┬──► SCHEDULING STATE (s_hat, eta_hat, validity)
                   │         │  ALL FOUR MODES — not gated by feedback_source
                   │         ▼
                   │    plant_schedule_eval ──► Kp, Ki, K, τ, b0 ──► (PI gains, b0, step_ff)
                   │                                                          │
                   └──► position candidate s_hat ──┐                          │
                                                   │                          │
      FLEX q_pct ──────────────────────────────────┤                          │
                                                   ▼                          │
      feedback_source ──► feedback selector ──► q_feedback                     │
                                                    │                          │
                          controller_type ──► PI  ─┤ ◄────── gains ────────────┘
                                              or   ├──► P_ref_raw
                                              ADRC ─┘
                                                    │
                                    ONE shared rate limiter (±5 / −6, clamp [0,170])
                                                    │
                                                  P_ref
                                                    │
                                    ONE shared pressure PI (deadband 1 kPa)
                                                    │
                                    ONE shared pneumatic / pulse / reversal path
                                                    │
                                              ActuatorCommand
```

This mirrors the firmware exactly: `apply_common_pref_limits()`, `pressure_pi_update()` and
`execute_pressure_control()` are single implementations shared by both outer controllers
(`main.c:1047`, `1402`, `1605`). Duplicating them per combination is prohibited.

**Feedback selection semantics** are fixed by `active_position_feedback_pct()`
(`main.c:466-483`) and must be transcribed, not reinterpreted:

```
if feedback_source == ukf and ukf4_feedback_available():
        q_feedback = clamp(s_hat, 0, 100)
else:   q_feedback = latest_position_pct        # FLEX percent
```

with `ukf4_feedback_available() = valid ∧ reference_valid ∧ theta_valid ∧ isfinite(s_hat)`
(`main.c:455-464`).

**Critical subtlety — the FLEX branch is not a control fallback.** When
`feedback_source = ukf` and the estimator is unavailable, the firmware's *control path* never
reaches this accessor: the outer loop takes its fail-safe branch instead (`main.c:3343-3367`,
`3384-3408`) and the LESO is not updated at all (`main.c:3288-3295`). The accessor's FLEX
return is reached only from telemetry, `#STATUS`, and the bumpless initialisers. Implementing
the `else` branch as a silent control fallback would be a **behavioural change**. See §7.8.

**A UKF-disabled mode is outside the validated baseline.** Because `eta_hat` exists only as a
UKF state and the firmware introduces *no alternative direction detector*
(`GAIN_SCHEDULING.md:29`), running without the estimator would require **defining new
scheduling inputs — above all `eta`** — which is a control-design change, not a migration.
`plant_schedule_eval` has no meaning without an `eta` to blend the UP/DOWN branches with
(`plant_schedule.h:35-36`). Such a mode is therefore recorded as a **possible post-baseline
design change requiring a new validated reference**, and is **not** an available baseline mode
(§13.7, §10 item 21).

### 6.6 Changing controller type or feedback source is a controlled transition

For baseline equivalence these are **not** hot-swappable parameters. In the firmware every
mode entry runs `select_controller_mode()` (`main.c:1800-1893`), whose per-mode ordering is
load-bearing (§4.15):

- `pi` → `update_plant_schedule()` then `initialize_pi_bumpless()`
- `adrc` → **`ff_gamma_state = 0` first**, then `update_plant_schedule()`, then
  `initialize_adrc_bumpless()`
- `CTRL_PRESSURE` → `q_ref = latest_position_pct`, `P_ref = clamp(pressure_filtered, 0, 170)`,
  `ff_gamma_state = 0`, reset actuation and diagnostics
- **all** modes then `pump_on()` and `set_state(STATE_HOLD)`

and entry into a `ukf` feedback source is **refused** unless `ukf4_feedback_available()` holds
(`main.c:1990-2002`, `2024-2036`).

Baseline rule: `controller_type` and `feedback_source` are set at launch/configuration time and
applied through the same validated transition sequence. If dynamic switching is supported
later, it must be implemented as an explicit `SelectMode` **state transition** that executes
the full reset/initialisation semantics above — never as a bare parameter assignment.
`ros2 param set` on these two parameters must therefore be rejected or routed through the
transition, not silently honoured. Tested by T16 and T26.

### 6.7 Parameters

Every constant in §2 and §4 becomes a declared, **read-only** parameter with the firmware value
as its default, loaded from a single YAML in `glove_controller/config`. Read-only prevents
accidental retuning. `controller_type` and `feedback_source` are the two exceptions: they are
configurable, but only through the transition of §6.6.

A startup log mirroring `print_startup_metadata()` (`main.c:2851-3026`) makes the active values
auditable and diffable against the firmware's own startup banner.

Launch: **one** parameterised launch file taking `controller_type` and `feedback_source` as
arguments. Four duplicated launch files are prohibited — there is no technical reason for them,
and they would drift.

---

## 7. Node responsibilities and interfaces

### 7.1 Pico responsibilities (unchanged by the architecture decision)

- 10 ms ADC acquisition (4-sample average, identical `sleep_us` pacing)
- Pressure raw → Vout → kPa conversion and the 0.20 EWMA
- FLEX raw acquisition and the 0.20 EWMA
- BNO055 init and quaternion register reads at 50 ms
- Valve GPIO level mapping and pump GPIO
- **Precise pneumatic pulse timing** via hardware alarm, including the arming-failure fallback
  to HOLD
- **Local emergency safety**: 200 kPa trip on unfiltered kPa → pump off, VENT, latch
- **Local communication watchdog** (new, §10 item 13)
- Calibration primitives: 500-sample ADC averages on request; streaming quaternion pairs
- Independent guards: reject a pulse while one is active, clamp durations to
  [5, 80] / [10, 100] ms, enforce the 200 ms reversal interval

**No high-level controller logic moves back to the Pico.** The Pico executes; it does not
decide. Its guards reject invalid commands and report the rejection; they never substitute a
control decision of their own.

### 7.2 `pico_bridge_node`

- Owns the physical link to the RP2040 (framing, checksums, resynchronisation, reconnect)
- Receives sensor/acquisition frames and republishes them as `SensorFrame` (10 ms) and
  `EstimatorInputFrame` (50 ms)
- Receives `ActuatorCommand` from `glove_control_node` and forwards it to the Pico
- Publishes `PicoStatus`: link state, watchdog state, safety latch, `pulse_alarm_fail_count`,
  frame-loss and sequence-gap counters
- Performs **no** control computation, no unit conversion beyond decoding, and no filtering.
  It is an I/O node.

### 7.3 `ukf_node`

- Subscribes to `EstimatorInputFrame`; calls the validated UKF4 in `soft_glove_core`;
  publishes `EstimatorState`
- Owns `q_rel0`, `reference_valid`, the θ unwrap accumulator, `ukfs_x` and `ukfs_P`
- Serves `CalibrateUkfReference` (collect 40 good quaternion pairs, average, reset the filter)
- Preserves the validated equations, the eight LUTs, initial covariance, **both** numerical
  fallback ladders, the unconstrained centre sigma point, both prediction-only paths and the
  `P = P−KSKᵀ` update form (§4.5)
- **Does no control.** It does not evaluate gain scheduling, does not select feedback and does
  not know which controller is active.

**`ukf_node` is required in all four baseline combinations** (§6.5). It is never optional and
is never omitted from a baseline launch: even with `feedback_source = flex`, continuous gain
scheduling consumes its `s_hat` / `eta_hat` output. The node is unaware of this — it publishes
the same `EstimatorState` regardless — but the launch graph is not optional.

Update cadence: one UKF step per `EstimatorInputFrame`, i.e. driven by **frame arrival**, not
by a local ROS timer. This is the closest available analogue of the firmware's ungated 50 ms
deadline (§1.6 consequence 3) and avoids a free-running timer sampling a jittery input stream.
`dt` is computed from consecutive **Pico acquisition timestamps**, never from ROS receive time
(§7.6).

### 7.4 `glove_control_node`

One node, one 10 ms callback (§6.3), containing as internal modules: feedback selection, gain
scheduling, position PI, LADRC + LESO + b0 smoothing + feedforward, the shared P_ref rate
limiter, the shared pressure PI, the pneumatic/pulse/reversal path, supervisory state logic,
calibration sequencing, the experiment interface (`SET_Q`, `SET_P`, `MARK`, `START_LOG`,
`STOP_LOG`, `CALIBRATE`, `END_RUN`, `ABORT`) and the 87-column CSV.

Calibration is a three-party sequence the controller drives: it commands VENT and the 10 s
wait, calls `CalibrateSensors` on `pico_bridge_node` for the 500-sample zeros, then calls
`CalibrateUkfReference` on `ukf_node`, then restores the outputs-off end state
(`perform_full_calibration`, §4.13). The ordering of that sequence is validated behaviour and
must be preserved across the node boundaries.

### 7.5 Interface contract — `glove_interfaces`

**Pico → bridge → all, 10 ms** — `SensorFrame`:
`pico_timestamp_us`, `seq`, `pressure_raw`, `pressure_vout`, `pressure_kpa`,
`pressure_filtered_kpa`, `flex_raw`, `flex_filtered_raw`, `pneumatic_state`, `pulse_active`,
`pulse_ms_remaining`, `pump_on`, `safety_latched`, `pulse_alarm_fail_count`, `watchdog_state`.

**Pico → bridge → `ukf_node`, 50 ms** — `EstimatorInputFrame`:
`pico_timestamp_us`, `seq`, `flex_filtered_raw`, `bno1_ok`, `bno1_quat`, `bno2_ok`,
`bno2_quat`.

> This frame is deliberately **combined**. In the firmware, `ukf_shadow_update(flex_filtered_raw)`
> reads both BNOs synchronously *inside* the call (`ukf_shadow_rp2040.h:1266-1283`), so the
> quaternions and the FLEX value always come from the same 10 ms tick. Shipping them in one
> frame reproduces that pairing exactly and removes any need for `ukf_node` to guess which
> `SensorFrame` a quaternion pair belongs to. Pairing by timestamp across two topics is the
> alternative and is **rejected** as a source of avoidable ambiguity.
>
> *Fidelity detail:* the firmware samples `time_us_32()` **after** the I²C reads
> (`ukf_shadow_rp2040.h:1321`), so the measured `dt` includes I²C time. The Pico must stamp
> `pico_timestamp_us` at the equivalent point for `dt` to match.

**`ukf_node` → `glove_control_node`, 50 ms** — `EstimatorState`:
`s_hat`, `v_hat`, `eta_hat`, `b_f_hat`, `valid`, `reference_valid`, `theta_valid`,
`theta_deg`, `innovation_flex`, `innovation_theta`, `nis`, `sigma_s`, `sigma_v`, `sigma_eta`,
`sigma_b_f`, `rho_eta_b_f`, `dt_used`, plus provenance: `source_seq` (the
`EstimatorInputFrame.seq` consumed), `source_timestamp_us` (the Pico acquisition stamp) and
`estimator_stamp` (ROS time at publication).

**`glove_control_node` → bridge → Pico** — `ActuatorCommand`:
`command` ∈ {HOLD, FILL, VENT, PUMP_ON, PUMP_OFF, ALL_OFF}, `pulse_ms`, `seq`, `deadline_us`.

**Services:** `CalibrateSensors` (pressure/flex zeros), `CalibrateUkfReference` (q_rel0),
`SelectMode` (controller_type, feedback_source — the controlled transition of §6.6).

### 7.6 Sequence, timestamp and estimator-state age

Every frame carries the **Pico acquisition clock**, so `dt` and data age are reconstructed from
the acquisition timeline rather than from ROS receive time (critical for the UKF, §2, §8.6).

`glove_control_node` must, for every `EstimatorState` it consumes, be able to determine:

| Quantity | Derivation |
|---|---|
| **Origin** | `source_seq` → exactly which `EstimatorInputFrame` produced this estimate |
| **Acquisition age** | `latest SensorFrame.pico_timestamp_us − EstimatorState.source_timestamp_us` — age in the Pico's own clock, immune to host clock offset |
| **Pipeline latency** | `now(ROS) − estimator_stamp` — transport + executor contribution |
| **Continuity** | gaps or repeats in `source_seq`; a repeat means the controller is re-reading an estimate it already used |

These four are published in `ControllerStatus` and logged in the CSV extension, so every run
carries its own evidence about Claim B (§6.4). **No implementation may consume `EstimatorState`
without evaluating its age.**

### 7.7 The pulse-interlock problem — explicit design constraint


`execute_pressure_control()` returns early when `pulse_active` is true (`main.c:1619-1620`).
On the Pico this is a zero-latency read of a local flag. Across a link it is a **stale**
value: with `FILL_MAX_MS = 80` and a 100 ms loop, a pulse can still be running when the next
decision is computed, and the ROS 2 side may not yet know.

Constraint for the implementation: ROS 2 must maintain a **predictive** `pulse_active` model
from `(pulse_start_seq, pulse_ms)` of the command it issued, reconciled against the Pico's
reported flag; the Pico's own rejection of an overlapping pulse is a *guard*, and any
rejection must be reported and must be treated as a **test failure** during equivalence
validation — it means the two models diverged. The same rule applies to the Pico's duration
clamp and reversal-interval guard: they must never fire when ROS 2 is behaving correctly.

### 7.8 Invalid and stale estimator state — policy

Two cases must be kept apart, because one has validated behaviour and the other does not.

**Case 1 — estimator reports invalid. Validated behaviour exists; transcribe it.**
`ukf4_feedback_available()` is false. The firmware response is fully specified and is a
**hold**, not an abort:

| Site | Firmware behaviour | Source |
|---|---|---|
| Outer loop, `pi` + `ukf` | `P_ref = clamp(pressure_filtered, 0, 170)`; `previous_position_error = 0`; log `#PI_UKF_HOLD`; PI **not** executed | `main.c:3343-3367` |
| Outer loop, `adrc` + `ukf` | `P_ref = clamp(pressure_filtered, 0, 170)`; `ff_gamma_state = 0`; log `#ADRC_UKF_HOLD`; LADRC **not** executed | `main.c:3384-3408` |
| LESO tick, `adrc` + `ukf` | Observer **not** updated; `eso_updated_since_log = false`; `update_active_b0()` still runs | `main.c:3286-3295` |
| Mode entry | `ukf` feedback source **refused** at selection time | `main.c:1990-2002`, `2024-2036` |
| Gain scheduling | `schedule_input_valid = false` → **hold last schedule**, change nothing | `main.c:945-951` |

Note what this is *not*: it is **not** a silent fall back to FLEX. The accessor's FLEX branch
is not reached from the control path (§6.5). Implementing invalid-UKF as "use FLEX instead"
would be a behavioural change and is prohibited.

**Case 2 — estimator state is stale. NO validated behaviour exists. New design decision.**
In the firmware the UKF was a same-process function call, so `valid` was always the current
tick's value and staleness was not representable. Across a node boundary an `EstimatorState`
can be arbitrarily old while still carrying `valid = true`.

> ⚠ **New design decision — requires user approval before implementation (§10 item 17,
> §13.7 choice 4).**
> Detection is not optional: §7.6 already requires the controller to evaluate age. What to *do*
> about it is the open part. The **proposed** policy, chosen because it reuses validated
> behaviour rather than inventing new behaviour, is:
>
> - treat `age > max_estimator_age_ms` as **equivalent to `valid = false`**, so Case 1's
>   validated hold applies unchanged;
> - publish the staleness in `ControllerStatus` and log it as a distinct event
>   (`#UKF_STALE`) so it is never confused with a genuine estimator invalidity;
> - do **not** extrapolate, hold-last-good, or substitute FLEX.
>
> `max_estimator_age_ms` has **no firmware counterpart** and cannot be derived from the
> reference. It must be set from measured pipeline latency (Stage 7) and approved explicitly.
> Until it is approved, the baseline runs with `feedback_source = flex`, which needs no
> estimator at all.

**Scope of Case 1 differs by mode — but `ukf_node` is required in both.**
An earlier revision of this document claimed that with `feedback_source = flex` the controller
must run correctly with `ukf_node` absent. **That was wrong and is withdrawn** (§13.7).
`ukf_node` runs in all four baseline combinations (§6.5). What differs is *which parts of the
controller* an invalid estimator affects:

| | `feedback_source = flex` | `feedback_source = ukf` |
|---|---|---|
| `q_feedback` | FLEX percent — **unaffected** by UKF validity | `s_hat` — **requires** validity |
| Gain scheduling (`Kp`, `Ki`, `K`, `τ`, `b0`, `step_ff`) | **Affected** — `schedule_input_valid = false` → hold last schedule | **Affected** — identically |
| Outer loop execution | **Continues**, on the held/default schedule | **Holds** per the table above |
| LESO update | **Continues** | **Not updated** (`adrc`) |

With an invalid or absent estimator and `feedback_source = flex`, the controller therefore
keeps running but the schedule never refreshes: before any valid sample it uses the
pre-first-valid q=30 UP anchor defaults (`main.c:345-346`, `main.c:856-857`), and after one it
holds the last valid schedule (`main.c:951`). Both are validated firmware behaviours, not
fallbacks.

**This is a degraded condition, not a supported operating point.** The validated procedure
requires `gs_valid = 1` before a trial and states *"No comenzar pruebas del nuevo scheduling
con gs_valid=0"* (`GAIN_SCHEDULING.md:11, 33`). `glove_control_node` must therefore surface
`schedule_input_valid` prominently in `ControllerStatus` and refuse to report a run as valid
while it is false. Running FLEX feedback with a dead estimator is an **anomaly to be detected
and reported**, not a mode to be designed for.

---

## 8. Deterministic equivalence tests (before any hardware testing)

Per the project's migration policy, deterministic tests are preferred over subjective judgement for every property that
can be tested.

### 8.1 What the existing CSV can and cannot support

An earlier version of this document proposed sample-by-sample replay against recorded CSV
logs as the primary equivalence gate. **That is not achievable with the 87-column CSV**, for
three independent reasons grounded in `print_csv_row()` (`main.c:2581-2844`):

**(a) The UKF time base is absent.** `ukf_shadow_update()` derives `dt` from `time_us_32()`
deltas (`ukf_shadow_rp2040.h:1321-1331`). The CSV's only clock is `t_ms`, an **integer
millisecond** `%lu` column sampled at 50 ms. The microsecond deltas that actually drove the
filter are never recorded and cannot be reconstructed.

**(b) The log cadence is 5× coarser than the state it would have to drive.** Sensors, the
EWMA filters and the overpressure check run at 10 ms; the CSV is emitted at 50 ms
(`LOG_PERIOD_MS`). **Four of every five sensor samples are never logged**, so the EWMA input
sequence — and therefore `pressure_filtered` and `flex_filtered_raw`, which feed the pressure
PI and the UKF respectively — cannot be replayed from the log.

**(c) Printed precision is below float32 for most columns.** Verified by pairing the 87 header
names with the 87 format specifiers programmatically:

| Column | Format | Recoverable exactly? |
|---|---|---|
| `pressure_raw`, `flex_raw` | `%.2f` | **Yes** — `read_adc_average` returns `sum/4` with `sum` an integer, so values are exact multiples of 0.25 and `%.2f` is lossless. *This holds only because `ADC_AVERAGE_SAMPLES = 4`.* |
| `flex_filtered_raw` | `%.2f` | No — EWMA output on values ≈1100–1740 gives ~6 significant digits vs. ~7.2 for float32 |
| `pressure_filtered_kpa`, `eso_z1_pct`, `ukf_s_pct`, `ukf_v_pct_s`, `ukf_theta_deg`, `ukf_bf_counts` | `%.3f` | No |
| `ukf_eta` | `%.5f` | No — 1e-5 resolution on [0,1] vs. float32 eps ≈ 6e-8 |
| `bno1_q*`, `bno2_q*` | `%.6f` | No — these are **post-normalisation** floats, not the raw int16/16384 values, so the exact filter input is unrecoverable |
| `eso_z2_pct_s`, `pi_kp`, `pi_ki`, `adrc_b0_active` | `%.6f` | Not in general — absolute, not significant, digits |
| `t_ms` | `%lu` | Integer ms only |

**(d) The missing samples make internal state genuinely ambiguous, not merely imprecise.**
Tightening tolerances does not help, because the unlogged inputs are not recoverable at any
precision. Concretely, with `FLEX_EWMA_ALPHA = 0.20` and a common starting state of 1500
counts, these two 5-sample input histories — of which the CSV records **only the fifth** —
both log `flex_raw = 1700`:

```
history A: 1500, 1500, 1500, 1500, 1700  → flex_filtered_raw = 1540.000
history B: 1900, 1850, 1800, 1750, 1700  → flex_filtered_raw = 1687.008
```

a divergence of **147 counts**, ≈30 % of `FLEX_SPAN_COUNTS` (494.794). The same argument
applies to `pressure_filtered`, which feeds the pressure PI, and propagates from there into
the UKF, the scheduler and every controller state.

**Conclusion.** The CSV can show that a ROS 2 run is *consistent with* a recorded firmware run.
It **cannot** establish equivalence of internal controller or estimator state, at any
tolerance, because the inputs that produced that state were never recorded. Exact deterministic
equivalence must therefore come from Tier 2 golden fixtures with complete input sequences,
initialisation and timing — never from the CSV.

### 8.2 Tiered equivalence strategy

| Tier | Method | Claim | Tolerance |
|---|---|---|---|
| **1** | Pure functions driven by **synthesised** inputs, compared against a host build of the unmodified golden sources | Exact equivalence | **0 ULP**, except libm (below) |
| **2** | Module- and scheduler-level replay driven by **firmware-generated golden vectors** captured at full precision | Exact equivalence | **0 ULP**, except libm |
| **3** | **Observational cross-check** against existing 87-column CSV logs | Gross consistency and experimental plausibility only — **no equivalence claim of any kind** | Qualitative; exact only for discrete/boolean columns |
| **4** | **Hardware** side-by-side against the golden firmware | Behavioural agreement | Statistical; bounds stated in advance |
| **5** | **ROS 2 integration** across the three nodes (§8.7) | Correct wiring, data provenance and estimator-state age (Claim B, §6.4) — **never** algorithmic equivalence | Structural assertions + measured latency bounds |

Tier 1 and Tier 2 are where equivalence is *established*. Tier 4 can *falsify* it. **Tier 3 is
not an equivalence test at all** — by §8.1(d) the recorded logs do not observe the inputs that
determine internal state, so no tolerance makes it one. It is retained because it is still
useful for catching gross errors, unit mistakes, inverted signs, wrong state sequences and
implausible ranges; it is never cited as evidence of equivalence.

**libm carve-out (applies to Tiers 1 and 2).** The only legitimate exact-tier divergence
sources are `logf`, `expf`, `sqrtf` and `atan2f`, which may differ by a few ULP between ARM
newlib and host glibc. Affected: `plant_schedule_eval` (log/exp), the UKF Cholesky and σ
computations (`sqrtf`), and θ accumulation (`atan2f`). Tolerance there is **≤ 4 ULP per
call**, and the tolerance propagated through a full UKF step must be **measured and
documented**, never assumed. A test that needs a loosened tolerance to pass is a finding, not
a pass. If bit-exactness across toolchains proves unattainable for the log/exp path, the
fallback is to pin a single shared implementation of those four functions into
`soft_glove_core` — a portability decision, not a control change, and one requiring explicit
user approval.

### 8.3 Tier 2 golden-vector fixtures (new work — currently missing)

Because no existing artefact carries sufficient precision, the required fixtures must be
**generated from the golden firmware sources themselves**. They do not exist yet and are a
Stage 0 deliverable.

**Generation method — no modification of `reference/rp2040_validated/`.** The reference is a
single translation unit of `static` functions, so a host harness in `tests/equivalence/` can:

- `#include` `plant_schedule.h` and `ukf_model_lut.h` directly (no SDK dependency);
- compile `main.c` and `ukf_shadow_rp2040.h` against **SDK stub headers** (Stage 0, §10 item 2)
  with `-Dmain=fw_main` and supply its own `main()`, giving the harness access to every
  `static` function without editing a reference file;
- stub `time_us_32`/`time_us_64`/`sleep_ms`/`sleep_us` with a **deterministic virtual clock**,
  and the I²C and ADC calls with table-driven playback.

This makes `dt` an *input* rather than a measured quantity, which is precisely what makes
exact replay possible.

**Fixtures to generate** (all numeric values serialised as C99 `%a` hex-float, which is
lossless for float32):

| Fixture | Contents | Feeds |
|---|---|---|
| `F1_lut.vec` | All 101 grid points + interior/edge probes for all **eight** tables | T1 |
| `F2_schedule.vec` | Dense `(q, η)` sweep incl. anchors, segment boundary at q=50, clamp regions, invalid inputs | T2 |
| `F3_ukf_step.vec` | Per-step `(flex_raw, theta_valid, theta, dt)` → full `x`, `P`, innovations, NIS, `valid` | T4 |
| `F4_theta.vec` | Raw int16 quaternion pairs (pre-normalisation) over a ±360° sweep, plus `q_rel0` | T5 |
| `F5_ctrl_step.vec` | Per-tick controller inputs → PI, LESO, LADRC, pressure-PI internals | T6, T8–T10 |
| `F6_sched_trace.vec` | Command/enable timeline → the exact sample ticks at which each loop fired | T15 |
| `F7_pulse_fsm.vec` | Pressure-PI output + `now` sequences → state, pulse ms, lockout transitions | T11, T12 |

`F4` deliberately stores the **raw int16 registers**, not normalised floats, because
normalisation is part of the code under test (§8.1(c)).

*Uncertainty:* the fixtures are only as authoritative as the SDK stubs. Stub fidelity —
particularly the `time_us_32()` wrap behaviour at 2³² µs ≈ 71.6 min and `add_alarm_in_ms`
semantics — **must be reviewed against the Pico SDK before the fixtures are trusted**, and
that review is not yet done.

### 8.4 Test suite

Tier is given per test. T1–T20 establish equivalence; T21–T22 can only falsify it (T21 not even
that — §8.1d); **T23–T31 are ROS 2 integration tests introduced by the architecture decision**
(§6, §13) and are covered separately in §8.7.

**Everything in T1–T20 must remain runnable without ROS 2.** The `soft_glove_core` library has
no ROS dependency, so the UKF, the PI path, the ADRC/LESO path, gain scheduling, the pressure
PI, the scheduler, feedback selection and the controller reset/initialisation semantics are all
unit-testable against golden fixtures with no node, no executor and no DDS in the loop. The
architecture decision does **not** weaken any Tier-1/Tier-2 requirement.

| ID | Target | Property |
|---|---|---|
| T1 | `ukfs_lut` | **Tier 1/2 (F1).** All 101 grid points exact for **all eight** tables; midpoints match linear blend; `s<0` and `s≥100` clamp; `s=100` returns `table[100]` exactly; index guards at `i<0` / `i≥N-1`. Separately: the eight tables are byte-identical to the golden header, `N_AXIS` is unit-norm, and σ pairs blend as **variances**, not σ |
| T2 | `plant_schedule_eval` | Reproduces K/τ at all 6 anchors; log-linear in q and η; edge-hold at q<30 / q>70; η clamp; `clamped` flag; `Ki=0.40/K`, `Kp=Ki·τ`, `b0=K/τ`; rejects NaN/Inf and non-positive results |
| T3 | UKF weights & noise | `Wm0=-24`, `Wc0=-21.04`, `Wi=3.125`, `c=0.16`; `Q(dt)` matches the dt⁴/4, dt³/2, dt², dt forms |
| T4 | UKF full step | **Tier 2 (F3).** Golden vectors with `dt` supplied as an input; covers **both** fallback ladders (`repair_cov4` jitter + the sigma-point `1e-5`/diagonal retry), the unconstrained centre sigma point, **both** prediction-only paths, the `P = P−KSKᵀ` form, and the in-place mutation of the stored `P` |
| T5 | θ accumulator | Continuity across ±π wrap; sign-flip handling via `quat_dot < 0`; unwrap over a full ±360° sweep; order dependence (replay must be sequential) |
| T6 | Position PI | `Δp = Kp·(e−e_prev)`, `Δi = Ki·0.5·e`; no output jump on gain change at zero error; bumpless init seeds `e_prev` and all P_ref fields |
| T7 | Rate limiter | +5.0 / −6.0 kPa per update; rate-limit **before** clamp; clamp [0, 170]; `saturated` at the 1e-6 threshold; `rate_limited` flag |
| T8 | LESO + b0 | Euler update with β1=5.0, β2=6.25, Ts=0.050; α=0.0909090909; the `1e-5` snap; **invariant `z2 + b0·P_ref` preserved across a b0 change** (replicate the reference's 1000-change sweep); **pins `target_b0 = 0.80*B0_NOMINAL + 0.20*next.b0` exactly** (`main.c:963-966`) so any drift toward the stale `GAIN_SCHEDULING.md` formulation fails loudly — §11 D1 |
| T9 | LADRC | `P_eq=−z2/b0`, `P_fb=(0.70/b0)·e`; γ piecewise map at 2.0/10.0; γ monotonically non-increasing until `SET_Q`; direction gate; FF clip [−5, +8]; `ff=0.30·Δq/K` |
| T10 | Pressure PI | Deadband early return **freezes** the integral and outputs 0; conditional integration on candidate magnitude > 1.0; sign-split form; clamp [0,1] then signed |
| T11 | `magnitude_to_pulse` | Truncation (not rounding); ranges 5–80 and 10–100; endpoints at m=0 and m=1 |
| T12 | Reversal FSM | Every transition in §4.12, including pending re-arm (no log line) and lockout satisfaction; 200 ms boundary; deadband clears `pending_reversal` |
| T13 | Valve mapping | The §1.4 table; `STATE_OFF`/`STATE_UNKNOWN` leave GPIOs unchanged; `all_outputs_off()` drives all low |
| T14 | Safety | Trip at ≥200 kPa on **unfiltered** kPa; shutdown sequence order; exit codes 1 / 2 |
| T15 | **Scheduler algorithm** | **Tier 2 (F6).** Differential test: the ported scheduler and the host-built firmware scheduler are driven with the **same** command/enable timeline and must produce **identical execution-tick sets** for all six counters. Must cover: one `now` per iteration; the 10 ms gate above all deadline evaluation; deadlines frozen while gated off; the ungated `next_ukf` (and that it carries `update_plant_schedule()`); consecutive-tick catch-up **including the 990/1000/1010/1500 case and its 980 and 1000 neighbours**; the blocking `CALIBRATE`/`END_RUN` stalls; `next_log` idle rebasing. Asserted against recorded tick sets, **not** against any regime model |
| T16 | Mode transitions | `select_controller_mode()` ordering per mode; `CTRL,*_UKF` refusal when UKF invalid; the three UKF-invalidity fail-safes of §4.14 |
| T16b | **Feedback selection** | **Tier 1.** `active_position_feedback_pct()` semantics (`main.c:466-483`) for all four `controller_type`×`feedback_source` combinations: `ukf` + available → `clamp(s_hat,0,100)`; `ukf` + unavailable → accessor returns FLEX **but the control path must not call it** (§6.5); `flex` → always `latest_position_pct`; `ukf4_feedback_available()` truth table over all 16 flag combinations |
| T16c | **Controller reset / init semantics** | **Tier 1.** `select_controller_mode()` per-mode ordering (§6.6): `pi` → schedule-then-bumpless; `adrc` → `ff_gamma=0` **first**, then schedule, then bumpless; `PRESSURE` → q_ref/P_ref reseed + actuation reset; all modes end `pump_on()` + HOLD. Asserts that a `controller_type`/`feedback_source` change executes the **full** transition, never a bare assignment |
| T17 | Calibration | Full sequence of §4.13; `flex_at_zero_raw` argument remains unused; `good ≥ 20` quorum; filters re-seeded; ends outputs-off |
| T18 | Conversions | kPa and position formulas; EWMA; raw-vs-filtered asymmetry of §4.3 |
| T19 | CSV parity | 87 columns, exact names, order and `printf` precision; startup metadata equals the firmware banner |
| T20 | **Closed-loop replay, exact** | **Tier 2.** Full `q_ref` sequence (e.g. 20→40→60→80→90→80→60→40→20 %, `README.md:31`) driven through the stubbed host build with a **virtual clock and a deterministic plant stub**, so `dt` and every sensor sample are inputs. Compares all §8.5 variables at 10 ms resolution. This is the primary closed-loop gate and needs **no** recorded hardware data |
| T21 | **CSV observational cross-check** *(not an equivalence test)* | **Tier 3.** Against a recorded 87-column log, check **gross consistency only**: discrete/boolean columns (`state`, `pulse_ms`, `controller`, `ukf_valid`, `gs_valid`, `rate_limited`, `saturated`) for plausible sequences; continuous columns for trend, sign, range and order of magnitude; `#` event lines for a coherent command narrative. **Must not** attempt to reconstruct EWMA, UKF or controller state, and **must not** be reported as evidence of equivalence — §8.1(d). Failure is informative; success proves nothing about determinism. **Requires a recorded log — see §10 item 1** |
| T22 | **Hardware side-by-side** | **Tier 4.** Same rig, same command sequence, golden firmware vs. ROS 2 stack. Statistical comparison of §8.5 variables with bounds declared before the run |

### 8.5 Equivalence variables (per the project's migration policy)

`s_hat`, `eta_hat`, `b_f_hat`, scheduled `K`, scheduled `tau`, `Kp`, `Ki`, `b0`
(`target_b0` and `active_b0` separately), LESO `z1`/`z2`, `P_ref` (raw, rate-limited and
final), pressure-controller output (`output_unsat` and `output`), `pneumatic_state`,
`pulse_duration`. Add: `theta_deg`, `nis`, `schedule_input_valid`, `ff_gamma`,
`ff_applied_kpa`.

Which tier each variable can be checked at:

| Variable | Tier 1/2 — exact equivalence | Tier 3 — observational only |
|---|---|---|
| `s_hat`, `eta_hat`, `b_f_hat`, `v_hat`, `theta_deg`, `nis` | yes | trend / range / sign |
| scheduled `K`, `tau`, `Kp`, `Ki`, `b0_identified`, `target_b0`, `active_b0` | yes | trend / range |
| LESO `z1`, `z2` | yes | trend / range |
| `P_ref` raw / rate-limited / final | yes | trend / range; rate-limit slope visible |
| pressure PI `output_unsat`, `output` | yes | range / sign |
| `pneumatic_state`, `pulse_ms` | yes | **sequence comparable** (discrete, logged exactly) |
| `schedule_input_valid`, `ukf_valid`, `theta_valid`, `rate_limited`, `saturated` | yes | **sequence comparable** (boolean, logged exactly) |
| `ff_gamma`, `ff_applied_kpa` | yes | trend; latch/decay shape visible |
| `pressure_filtered`, `flex_filtered_raw` | yes | **not comparable** — driving inputs unobserved (§8.1d) |

The discrete and boolean columns are the only Tier 3 signals recorded losslessly, so they are
the most useful for spotting gross errors. Even there, a matching sequence is evidence of
plausibility, not of equivalence: the same sequence is reachable from different internal
trajectories.

### 8.6 Known equivalence risks to quantify, not assume

1. **UKF `dt`** is measured wall-clock. Bit-equality is achievable **only** when `dt` is
   supplied as an input (Tier 2). It cannot be reconstructed from the CSV, which carries
   integer milliseconds at 50 ms (§8.1a). Hardware comparison (T22) is therefore statistical
   by construction, not by choice.
2. **Transport latency** on `pressure_filtered` and the actuator command shifts the effective
   sample instant of the 100 ms pressure loop. Must be measured and bounded before hardware
   testing; it is a *new* error source with no counterpart in the reference.
3. **libm ULP differences** (§8.2).
4. **`pulse_active` staleness** (§7.4).
5. **`repair_cov4` is a mutating side effect** on the stored covariance — any refactor that
   "cleans this up" into a local copy changes behaviour.
6. **Scheduler re-phasing** (§1.6). The relative phase of the UKF, LESO, outer and pressure
   loops depends on the *command history* of a run, not on a fixed grid. Two runs with
   identical `q_ref` sequences but different operator timing can produce different loop
   interleavings. Any Tier 4 comparison must therefore reproduce the **command timeline**, not
   just the reference sequence — and the command timeline is not recorded in the CSV beyond
   `t_ms`-stamped `#` event lines. This is a further reason Tier 3 cannot support an
   equivalence claim: the log does not even pin down the schedule that produced it.
7. **`time_us_32()` wraps every ≈71.6 minutes.** `ukfs_last_update_us` is a `uint32_t` and the
   subtraction is unsigned, so a single wrap is handled correctly; but a run longer than one
   wrap period has never been shown to be exercised, and the `dt > 0.20` guard would mask a
   misbehaviour rather than surface it. Long-run behaviour is **untested in the reference**.

### 8.7 ROS 2 integration tests (introduced by the architecture decision)

These test the **wiring**, not the mathematics. They cannot establish algorithmic equivalence —
that is T1–T20's job on `soft_glove_core` with no ROS in the loop — and they must never be
cited in its place. Their purpose is Claim B (§6.4): that the three-node system delivers the
right data, to the right module, with known age.

| ID | Target | Property |
|---|---|---|
| T23 | **FLEX → `glove_control_node`** | `SensorFrame` at 10 ms drives the sample deadline; `flex_filtered_raw` → percent conversion matches T18; the control scheduler's execution-tick set matches T15 under real message arrival |
| T24 | **`EstimatorInputFrame` → `ukf_node` → `glove_control_node`** | End-to-end: a recorded input sequence fed through the bridge produces `EstimatorState` whose `s_hat`/`eta_hat`/flags match the T4 golden vectors for the same inputs, and arrives at the controller intact. Any mismatch is a wiring/serialisation defect, since T4 already proved the algorithm |
| T25 | **Sequence / timestamp consistency** | `source_seq` is monotonic with no unreported gaps; `source_timestamp_us` is strictly increasing; `dt_used` equals consecutive Pico stamp deltas, **not** ROS receive deltas; all four §7.6 quantities are computable for every consumed message; repeats are detected |
| T26 | **PI + FLEX** | Combination runs; outer loop uses `latest_position_pct`; entry executes the §6.6 `pi` transition; shared rate limiter and shared pressure PI are the same code objects as in T27–T29 |
| T27 | **PI + UKF** | Outer loop uses `clamp(s_hat,0,100)`; entry **refused** when `ukf4_feedback_available()` is false; invalid mid-run triggers the validated `#PI_UKF_HOLD` branch, not a FLEX fallback |
| T28 | **ADRC + FLEX** | LESO updates from `latest_position_pct`; entry executes `ff_gamma=0` → schedule → bumpless in that order |
| T29 | **ADRC + UKF** | LESO uses `s_hat`; invalid mid-run → observer **not** updated while `update_active_b0()` still runs (`main.c:3286-3295`); `#ADRC_UKF_HOLD` on the outer tick |
| T30 | **`feedback_source` gates `q_feedback` ONLY** | **Tier 1 + Tier 5.** For all four combinations, assert that switching `feedback_source` changes the signal reaching `q_feedback` and **nothing else**: `Kp`, `Ki`, `K`, `τ`, `b0`, `target_b0` and `step_ff` must be **bit-identical** between `pi`+`flex` and `pi`+`ukf` (and between the two `adrc` variants) when driven by the same `EstimatorState` sequence. Asserts `ukf_node` is present and publishing in **all four** launches, and that gain scheduling consumes `s_hat`/`eta_hat` in every one (`main.c:945-951`, `GAIN_SCHEDULING.md:29`) |
| T30b | **Degraded-estimator behaviour by mode** | **Tier 5.** With `ukf_node` stopped mid-run: `feedback_source = flex` → outer loop **continues** on the held schedule, `schedule_input_valid` goes false and is surfaced in `ControllerStatus`, run is flagged not-valid per `GAIN_SCHEDULING.md:33`; `feedback_source = ukf` → validated hold branch. Asserts this is **reported as an anomaly**, not silently absorbed (§7.8) |
| T31 | **Stale / invalid estimator detection** | Injected delay, dropped frames and frozen `EstimatorState`: staleness is detected per §7.6, reported in `ControllerStatus`, logged as `#UKF_STALE` distinctly from genuine invalidity, and — **once the §10 item 17 policy is determined from measurement** — routed to the validated hold. Until then this test asserts detection and reporting only; **UKF-feedback hardware operation stays unauthorised** (§13.7 choice 4) |

**Scheduling assertion (T26–T29).** All four combinations must run `ukf_node` and must show
gain scheduling consuming `s_hat`/`eta_hat`, with `gs_valid` reaching 1 before the controlled
portion of the test begins (`GAIN_SCHEDULING.md:11, 33`). A test that reaches its assertions
with `gs_valid = 0` is **invalid**, not passing.

**Shared-path assertion (T26–T29).** All four combinations must exercise **one** rate limiter,
**one** pressure PI and **one** pneumatic path. The test asserts this structurally — e.g. by
instrumenting call counts on the single shared instances — so that a future duplication of the
inner loop per combination fails the suite rather than passing silently.

---

## 9. Migration stages in dependency order

Each stage has a gate that must pass before the next begins. Per the project's migration policy, a maximum of
two automatic implementation-review cycles per delegated task.

**Stage 0 — Test infrastructure and golden-vector generation.**
Pico SDK stubs (as `GAIN_SCHEDULING.md:66-72` describes but does not provide), a
deterministic virtual clock, the `-Dmain=fw_main` host harness of §8.3, the seven fixtures
`F1`–`F7`, and a ULP comparison harness in `tests/equivalence/`. **No reference file is
edited.**
*Gate:* the unmodified golden sources compile and run on host; `F1`–`F7` are reproducible
bit-for-bit across two runs and two compilers; **the SDK stubs have been reviewed against the
real Pico SDK** (timer wrap, alarm semantics) and the review is written down. Until that
review exists, every downstream "exact" result is provisional.

**Stage 1 — `soft_glove_core`: pure functions.**
LUT, `plant_schedule`, conversions, EWMA, `magnitude_to_pulse`, rate limiter.
*Gate:* T1, T2, T7, T11, T18 pass at 0 ULP (libm carve-out per §8.2).

**Stage 2 — UKF4.**
Port the estimator with SDK calls removed and the algorithm untouched.
*Gate:* T3, T4, T5 pass; propagated tolerance measured and documented.

**Stage 3 — Controllers.**
Position PI, LESO, b0 smoothing, LADRC, feedforward, pressure PI.
*Gate:* T6, T8, T9, T10 pass.

**Stage 4 — Scheduler and supervisory logic.**
The 10 ms tick with exact ordering **and the per-counter gating/rebasing rule of §1.6** (not
fixed-phase timers); mode transitions; bumpless initialisation; calibration sequencing; safety
policy; reversal FSM.
*Gate:* T12, T13, T14, T15, T16, T17 pass; **T20 (exact closed-loop replay) passes** — this
is the primary closed-loop gate and is not blocked on any recorded data.

**Stage 5 — ROS 2 wrapping: `glove_control_node`.**
`glove_interfaces`, then the single control node wrapping `soft_glove_core`: one 10 ms
callback, the literal deadline algorithm, feedback selector, `controller_type` /
`feedback_source` parameters and the §6.6 transition, the **unchanged 87-column** CSV module
(§13.7 choice 1), and `ControllerStatus`/rosbag2 diagnostics for provenance (§13.7 choice 2).
Launch and YAML live in `glove_controller` (§13.7 choice 3). **No algorithm code here.**
*Gate:* T19 passes; T16b, T16c pass; a run driven by replayed `SensorFrame` messages reproduces
the Stage 4 outputs **exactly** (same golden fixtures, now through the node); T23 passes.

**Stage 6 — `ukf_node` and `pico_bridge_node`.**
The estimator wrapper and the I/O node. Still no new algorithm code — `ukf_node` calls the
Stage 2 estimator unchanged.
*Gate:* T24 and T25 pass; **T30 passes** — `feedback_source` demonstrably gates `q_feedback`
only, with scheduling outputs bit-identical across the two feedback sources; T30b passes.
`ukf_node` is part of **every** baseline launch (§6.5).

**Stage 7 — Pico firmware (new).**
Acquisition, GPIO, pulse timing, local safety trip, **communication watchdog**, link framing,
and the combined 50 ms `EstimatorInputFrame` with its acquisition timestamp taken at the point
equivalent to `ukf_shadow_rp2040.h:1321`.
*Gate:* pulse-duration accuracy measured on hardware; safety trip verified; watchdog fail-safe
verified. **Blocked on the §10 item 13 decision.**

**Stage 8 — Integration, latency characterisation and the four combinations.**
End-to-end bring-up of the three nodes.
*Gate:* T26–T29 pass (all four `controller_type` × `feedback_source` combinations, with the
scheduling and shared-path assertions); T30, T30b, T31 detection and reporting pass;
**measured pipeline latency and estimator-state age distributions documented**, feeding the
§10 item 17 determination of `max_estimator_age_ms`; **executor and QoS behaviour explicitly
tested and documented** (§13.7 choice 5, §10 item 18). T21 may be run as a sanity check but
gates nothing (§8.1d). **Claim B (§6.4) is characterised here, not asserted.**

**Hardware authorisation gates set here:** UKF-feedback hardware operation requires the
`max_estimator_age_ms` determination (§13.7 choice 4); any hardware-equivalence claim requires
the executor/QoS documentation (§13.7 choice 5). Neither is satisfied by this stage passing
its functional tests alone.

**Stage 9 — Hardware equivalence.**
Side-by-side runs against the golden firmware on the same rig, comparing §8.5 variables (T22),
with the command timeline reproduced, not just the reference sequence (§8.6 item 6).
*Gate:* documented comparison evidence within pre-declared bounds. **No equivalence claim may
be made before this stage**, and the claim it supports is statistical agreement on hardware
within pre-declared bounds — the exactness claim rests entirely on Stages 1–4.

---

## 10. Undetermined items

Not derivable from `reference/rp2040_validated/`. None of these are guessed anywhere above.

1. **No recorded reference dataset, and the CSV format is insufficient for exact replay even
   when one is supplied.** No capture exists in the repository, so T21 cannot run. Separately
   — and independently of availability — the 87-column format lacks the microsecond time base,
   logs at 5× the sensor period, and prints most columns below float32 precision (§8.1).
   Even with a log in hand, T21 remains an **observational cross-check only** (§8.1d), so this
   gap blocks no equivalence claim. *Useful to have: at least one validated run's 87-column CSV
   together with its `#` event lines, so the command timeline can be reconstructed (§8.6
   item 6).* Exact closed-loop equivalence (T20) does **not** depend on this and is reached via
   Tier 2 fixtures instead.
2. **Host test stubs absent.** `GAIN_SCHEDULING.md:66-72` references `tests/stubs` and
   `tests/test_schedule.c`; neither is present. Stage 0 must recreate them.
3. **LUT source data absent.** `UKF_V2_measurement_curves_dense.csv`
   (`ukf_model_lut.h:6-7`) is not in the repository. The LUT arrays must be treated as the
   primitive reference and copied byte-for-byte.
4. **Plant identification data absent.** `GAIN_SCHEDULING.md:17` states the anchors are
   *rounded* values and the full-precision CSV is not included. Anchors cannot be refined.
5. **On-target timing unmeasured.** `GAIN_SCHEDULING.md:41` states Pico latency "requires
   measurement". No `exec_us` / `max_exec_us` figures exist. Real-time feasibility of the split
   is therefore unverified.
6. **Valve hardware semantics.** Whether V1/V2 are NO or NC, and what the level table
   physically means, is not stated — only the GPIO levels are. Do not infer pneumatic topology.
7. **Pressure sensor part and transfer function.** Only `DIVIDER_GAIN = 1.545` and
   `KPA_PER_VOLT = 50.0` are given; no datasheet, no validity range, no temperature behaviour.
8. **`FLEX_SPAN_COUNTS = 494.794` provenance** and its re-measurement procedure are not
   documented.
9. **BNO055 calibration status is never read.** `CALIB_STAT` (`0x35`) is not polled; NDOF
   fusion quality at any moment is unknown, and the firmware has no notion of it.
10. **Host protocol unspecified.** USB CDC is used, but the capture script
    `pico_ukf4_pi_adrc_feedback_test.py` (`README.md:37`) is not in the repository, and no
    framing, baud, flow-control or reconnection semantics are defined.
11. **No recovery after emergency shutdown.** `main()` returns 1 or 2 and the firmware stops.
    What the hardware should do next (halt, reset, await operator) is undefined. The ROS 2
    supervisor's post-trip policy is therefore a **decision required from the user**.
12. **Target platform unspecified.** Raspberry Pi model, kernel/RT configuration, DDS
    implementation and QoS, and the physical Pico↔Pi transport (USB CDC, UART, SPI) are not
    determined by any reference file.
13. **The communication watchdog has no reference behaviour.** The firmware contains **no**
    watchdog of any kind. On a host disconnect today the Pico simply keeps its last state with
    the pump on. The target architecture mandates one, so its timeout and fail-safe action
    (most plausibly: pump off + VENT + latch, mirroring `emergency_shutdown`) are **new design
    and require explicit user approval.** Not specified here.
14. **`START` is a no-op** beyond printing `#START_RECEIVED`. Whether ROS 2 should give it
    meaning is a product decision, not a migration one.
15. **`global_start_ms` is set before calibration** (`main.c:3091`), so `t_ms` in the CSV
    includes the 10 s initial vent. Whether ROS 2 timestamps should share this origin is
    undefined.
16. *(Resolved — see §13.7 choice 3.)* Launch and YAML configuration remain in
    `glove_controller`; no `glove_bringup` package in the initial scaffold.

### New items introduced by the architecture decision

17. **`max_estimator_age_ms` is deliberately NOT defined yet.** Staleness was not representable
    in a same-process estimator (§7.8 Case 2), so no firmware value exists and none may be
    invented. **Measure ROS 2 estimator latency and jitter first** (Stage 8), then determine
    the threshold. Until it is measured and approved, **UKF-feedback hardware operation is not
    authorised** (§13.7 choice 4). This does *not* gate FLEX-feedback runs — but `ukf_node`
    still runs in those, and its scheduling output is still consumed (§6.5).
18. *(Partly resolved — see §13.7 choice 5.)* Initial deployment is **separate processes with a
    single-threaded executor**. QoS profiles for `SensorFrame`, `EstimatorInputFrame`,
    `EstimatorState` and `ActuatorCommand` remain to be chosen, and executor/QoS behaviour must
    be **explicitly tested and documented** before any hardware-equivalence claim.
19. **Composed (single-process) deployment remains an available lever, not a plan.** It would
    cut serialisation and transport cost while preserving the approved node separation. It is
    reserved for the case where Stage 8 latency proves too high; it is not assumed, and
    adopting it would change the timing characterisation and require re-measurement.
20. *(Resolved — see §13.7 choices 1 and 2.)* The legacy 87-column CSV stays byte-compatible;
    new metadata goes to ROS 2 diagnostics / rosbag2.
21. **A UKF-disabled operating mode has no validated definition.** It would require defining new
    scheduling inputs — above all `eta`, which exists only as a UKF state and for which the
    firmware provides *no alternative direction detector* (`GAIN_SCHEDULING.md:29`). This is a
    **possible post-baseline design change**, outside the migration, and would require a new
    validated reference per the project's migration policy. It is **not** an available baseline mode.

---

## 11. Reference-internal divergences observed

These are recorded, **not corrected**. The project's migration policy makes the firmware the behavioural
reference; where prose disagrees with code, the code governs.

**D1 — b0 target mixing.** `GAIN_SCHEDULING.md:25` states *"El objetivo b0 ya no mezcla 80 %
nominal y 20 % identificado."* The code does exactly that:
```c
//target_b0 = next.b0;                              main.c:960  (commented out)
target_b0 = 0.80f * B0_NOMINAL + 0.20f * next.b0;   main.c:963-966
```
**Resolved for the baseline migration: the executable validated firmware is authoritative.**
The baseline ROS 2 port reproduces `main.c` exactly — `target_b0 = 0.80*B0_NOMINAL +
0.20*next.b0` — and does **not** substitute the conflicting description in
`GAIN_SCHEDULING.md`. This is **not** an open tuning decision and must not be re-litigated
during implementation; the project's migration policy makes the validated firmware the behavioural reference, and
the prose is simply stale with respect to the code it describes.

The discrepancy is recorded here because it is the single highest-risk item for a reader who
works from the design note rather than the source: doing so would silently produce a different
controller. Any future decision to adopt the `GAIN_SCHEDULING.md` formulation would be a
**controller change**, outside the scope of this migration, and would require a new validated
reference per the project's migration policy. Test T8 pins the 80/20 expression so that a drift toward the prose
version fails loudly.

**D2 — `SET_P` error string.** The message reads `RANGE_0_120_KPA` (`main.c:2103`) while the
validated range is `[PRESSURE_REF_MIN_KPA, PRESSURE_REF_MAX_KPA] = [0, 170]`
(`main.c:2098-2100`). The *check* is correct; only the string is stale. Reproduce the check;
the string is a telemetry-parity question.

**D3 — Dead state variables.** `previous_q_ref_pct` is assigned in four places and never read;
`active_gain_region_id` is permanently 0. They must be carried for CSV parity but must not be
given behaviour.

---

## 12. Summary of the migration principle

For the same input sequence, the ROS 2 implementation must reproduce the validated RP2040
implementation across all §8.5 variables:

- **exactly** (0 ULP, libm carve-out per §8.2) wherever inputs and state initialisation are
  fully specified — Tiers 1 and 2, driven by synthesised inputs and firmware-generated golden
  vectors, including the exact closed-loop replay T20;
- **to pre-declared statistical bounds** on hardware — Tier 4.

Tier 3 (recorded 87-column CSV) is explicitly **not** part of this chain. Because the log
observes one sample in five and never records the inputs that determine internal state, it can
demonstrate gross consistency and experimental plausibility only, at any tolerance (§8.1d).

**After the three-node architecture decision (§13), one further separation is permanent.**
Equivalence is a property of the **algorithms in `soft_glove_core`**, established without ROS 2
in the loop (Claim A, §6.4). It is *not* a property of the integrated node graph: crossing
DDS/executor boundaries adds latency and removes the same-process ordering the firmware relied
on. Tier 5 integration tests (§8.7) verify wiring, provenance and estimator-state age — they
**cannot** substitute for Tier 1/2, and no passing integration test may be reported as evidence
of equivalence. **Exact system-level timing equivalence across node boundaries is not claimed
anywhere in this plan.**

"Same input sequence" includes the **command timeline**, because loop phase depends on it
(§1.6, §8.6 item 6). Equivalence may not be claimed from code review, structural similarity,
or plausibility. Tiers 3 and 4 can only falsify equivalence; they cannot establish it.

---

---

## 13. Approved architecture decision (user-approved, pre-implementation)

Recorded verbatim in effect, not as a proposal. Taken **before** any ROS 2 code was written.
This section is the authority for §3, §5.2, §6, §7, §8.7, §9 and §10 items 16–20. The audit
history in the two audit sections below is preserved unchanged; this decision post-dates it and
supersedes only the node-topology sketch that the earlier §6 contained.

### 13.1 The decision

The baseline ROS 2 system uses **three primary runtime nodes**:

1. `pico_bridge_node`
2. `ukf_node`
3. `glove_control_node`

`soft_glove_core` and `glove_interfaces` are a library and an interface package. They are
**not** runtime nodes. No experiment-manager, GUI, logger or visualisation node is part of the
baseline.

### 13.2 What changed relative to the pre-decision plan

| Area | Before | After |
|---|---|---|
| Node count | 4 (`bridge`, `control`, `supervisor`, `telemetry`) | **3** (`pico_bridge_node`, `ukf_node`, `glove_control_node`) |
| UKF placement | A module inside the single control node | **Its own node**, wrapping `soft_glove_core` |
| Supervisor | Separate `supervisor_node` | **Module inside `glove_control_node`** |
| Telemetry/CSV | Separate `telemetry_node` | **Module inside `glove_control_node`**; logger node deferred |
| Packages | 7 (`soft_glove_*`) | **5** (`glove_interfaces`, `soft_glove_core`, `pico_bridge`, `glove_estimator`, `glove_controller`) |
| Mode selection | One `controller_mode` mirroring the firmware enum | **Two orthogonal parameters**: `controller_type` × `feedback_source`, where `feedback_source` gates `q_feedback` **only** (§13.6) |

### 13.3 What did NOT change — findings retained in full

- **The control path remains one node** (§6.3). No separate runtime node for the PI, the ADRC,
  the LESO, the pressure PI, gain scheduling or the pneumatic state logic. The earlier
  "one control node, not five" finding is retained and restated.
- **The literal deadline algorithm** (§1.6) and the prohibition on independent
  `create_wall_timer()` callbacks for the control scheduler.
- **All Tier-1/Tier-2 golden-fixture requirements** (§8.2, §8.3). The architecture decision adds
  Tier 5 integration tests; it removes nothing.
- **Every frozen control constant and equation** in §2 and §4.
- **Gain scheduling's unconditional dependence on UKF state** in all modes (§6.5, §13.6) — the
  architecture decision does not and cannot decouple it.
- The A1–A4 and B1–B3 audit corrections.

### 13.4 The cost this decision introduces, stated plainly

Separating the UKF into its own node puts a DDS/executor boundary where the firmware had a
direct function call inside the same 10 ms iteration (§6.4). This is accepted deliberately, in
exchange for independent testability and a bounded control callback. The consequence is
recorded as a permanent two-claim split:

- **Claim A — algorithmic equivalence of the UKF:** achievable, a hard gate, established by
  Tier 1/2 fixtures on `soft_glove_core` **without ROS 2**.
- **Claim B — system-level timing equivalence after integration:** **not claimed**. Measured,
  bounded and reported (T23–T25, Stage 8).

**Exact system timing equivalence across node boundaries is not asserted anywhere in this
plan, and must not be asserted from code structure.** Where same-process ordering can no longer
be guaranteed — specifically the estimator-update-to-consumer ordering that the firmware closed
within one iteration — §7.6 requires the controller to measure the age and origin of every
estimator state it consumes, and §8.7 requires that measurement to be exercised.

### 13.5 Supported operating combinations

Four, from two orthogonal selections, sharing **one** inner loop (§6.5):

| `controller_type` | `feedback_source` | Firmware equivalent |
|---|---|---|
| `pi` | `flex` | `CTRL,PI` |
| `pi` | `ukf` | `CTRL,PI_UKF` |
| `adrc` | `flex` | `CTRL,ADRC` |
| `adrc` | `ukf` | `CTRL,ADRC_UKF` |

Plus the two non-combination supervisory states `CTRL,PRESSURE` and `CTRL,NONE`.

Changing either selection is a **controlled state transition** executing the validated
`select_controller_mode()` reset/initialisation semantics (§6.6), never a bare parameter write.

### 13.6 Clarification — `feedback_source` vs. gain-scheduling state source

Approved before Stage 1, correcting an error in the first revision of this architecture record.

**Two distinct consumers of UKF output must not be conflated:**

| | Position feedback source | Gain-scheduling state source |
|---|---|---|
| Selected by | `feedback_source` | **nothing — always the UKF** |
| Signal | `s_hat` **or** FLEX percent | `s_hat` **and** `eta_hat` (+ validity) |
| Consumer | External PI / ADRC `q_feedback` | `plant_schedule_eval` → `Kp`, `Ki`, `K`, `τ`, `b0`, `step_ff` |
| Varies by mode | Yes | **No — identical in all four** |

Therefore, for the baseline migration:

- **`ukf_node` runs for all four controller/feedback combinations.**
- `pi`+`flex` and `adrc`+`flex` use **FLEX** as `q_feedback`.
- `pi`+`ukf` and `adrc`+`ukf` use **`s_hat`** as `q_feedback`.
- **Continuous plant/gain scheduling consumes the validated UKF scheduling state in all four
  modes.**

`feedback_source = flex` does **not** disable the UKF and must never be described as making
`ukf_node` unnecessary. Grounding: `update_plant_schedule()` gates on
`ukf4_feedback_available()` with no reference to `controller_mode` (`main.c:945-951`);
`GAIN_SCHEDULING.md:29` states the shared model is indexed by `s_hat` and `eta_hat` *even in*
`CTRL,PI` and `CTRL,ADRC` and that no alternative direction detector is introduced; and the
validated procedure requires `gs_valid = 1` before a trial (`GAIN_SCHEDULING.md:11, 33`).

**Post-baseline only:** a mode in which the UKF is completely disabled would require a **new
definition of the scheduling inputs, especially `eta`**, and is therefore **outside the
validated baseline migration**. It is recorded as a possible post-baseline design change
requiring a new validated reference — **not** an available baseline mode (§10 item 21).

### 13.7 Additional approved choices

| # | Choice | Consequence in this plan |
|---|---|---|
| 1 | **Keep the legacy 87-column CSV unchanged** for historical/experimental compatibility | T19 continues to assert exactly 87 columns, names, order and `printf` precision. No provenance columns are appended. §10 item 20 resolved. |
| 2 | **Use ROS 2 diagnostics / rosbag2 for new metadata** — `source_seq`, timestamps, estimator age, transport timing | §7.6 quantities are published in `ControllerStatus` and recorded via rosbag2, **not** folded into the CSV. T25 asserts against the bag, not the CSV. |
| 3 | **No `glove_bringup` package in the initial scaffold** | Launch files and YAML stay in `glove_controller/launch` and `glove_controller/config` (§6.2). §10 item 16 resolved. |
| 4 | **Do not invent `max_estimator_age_ms` yet** — measure ROS 2 estimator latency/jitter first, then determine the threshold | §7.8 Case 2 stays explicitly unapproved. **UKF-feedback hardware operation is not authorised** until the measurement (Stage 8) and approval are done. T31 asserts detection and reporting only. §10 item 17 refined. |
| 5 | **Initial runtime deployment: separate processes, simple single-threaded execution model** | Three OS processes, single-threaded executor each. Composition stays an unused lever (§10 item 19). **Executor and QoS behaviour must still be explicitly tested and documented before any hardware-equivalence claim** (§10 item 18, Stage 8). |

Choices 4 and 5 are the two that gate hardware work: neither a UKF-feedback hardware run nor a
hardware-equivalence claim is authorised until the corresponding measurement and documentation
exist.

### 13.8 Scope boundary

This decision concerns **software architecture only**. It changes no equation, no constant, no
gain, no timing parameter and no control law. The Pico's responsibilities are unchanged, and no
high-level controller logic moves back to it. Anything in §7.8 Case 2 marked as a new design
decision remains **unapproved** and must not be implemented until the user approves it.

---

## Audit Corrections Incorporated

An earlier revision of this document was reviewed against
`reference/rp2040_validated/`. Each finding below is listed with its firmware grounding and
the resolution applied. No controller behaviour was changed, and no file outside this document
was modified.

### A1 — Scheduler timing semantics: loops do not stay phase-aligned

**Finding.** The plan asserted that because the deadline counters share an origin and the
periods are integer multiples of 10 ms, the loops "tick together" at coincident boundaries,
and stated LESO-before-outer and UKF-before-LESO as unconditional consequences.

**Grounding.** `next_eso` (`main.c:3278-3284`), `next_outer` (`main.c:3325-3336`) and
`next_pressure` (`main.c:3440-3447`) are each advanced **only inside their own gated block**,
so a disabled loop's deadline freezes. `next_ukf` (`main.c:3247`) and `next_sample` are
ungated and never freeze. `next_log` is additionally rebased to `now + 50` on every idle
sample tick (`main.c:3502-3512`). The catch-up clamp
`if ((int32_t)(now - next_X) >= P) next_X = now + P` fires iff the loop is **≥ 2P** overdue.

**Resolution (first audit).** Added **§1.6**, tabulating every gate and establishing that
disabled deadlines freeze. §1.5 now presents the two ordering rules as conditional on both
loops firing in the same iteration. §5, §6.2, §8.6 (new item 6), T15 and Stage 4 were updated
to match.

**⚠ Superseded in part by B1 below.** The three-regime re-entry model introduced by this
resolution was itself wrong and has since been withdrawn.

**Remaining uncertainty (marked in §1.6):** whether any re-phasing materially affected the
validated experimental results is not determinable — it depends on per-run operator command
timing, which the repository does not preserve.

### A2 — LUT inventory: eight tables, not six

**Finding.** The plan said `ukf_model_lut.h` contains "6 tables × 101 entries".

**Grounding.** The header defines **eight** 101-entry tables (`ukf_model_lut.h:33, 53, 73, 93,
113, 133, 153, 173`) plus the 3-element `UKF_MODEL_N_AXIS` (line 27) and the
`UKF_MODEL_F0_REFERENCE` macro (line 198). The two omitted tables were
`UKF_MODEL_SIGMA_THETA_UP_DEG` and `UKF_MODEL_SIGMA_THETA_DOWN_DEG`.

**Resolution.** §4.5 now carries a full ten-row inventory with line numbers, lengths and
roles, splitting the four measurement-mean tables (consumed by `ukfs_measurement_model`) from
the four σ tables (consumed by `ukfs_measurement_sigma`), and separating `N_AXIS` — which is
not indexed by `s` — from the interpolated tables. It also records that mean pairs blend
**linearly** while σ pairs are squared to variances first and the **variances** blend, then
floor, then scale by `R_SCALE`; blending σ directly would be a behavioural change.
`F0_REFERENCE` is marked as documentation-only. T1 was extended to all eight tables, to
byte-identity of the table data, to the unit-norm property of `N_AXIS` (verified: ‖n̂‖ = 1.0 in
float32) and to the variance-vs-σ blend.

### A3 — Replay and equivalence testing: CSV is insufficient for exact replay

**Finding.** The plan made sample-by-sample CSV replay (old T20) the primary closed-loop gate
and treated it as an exactness check, blocked only on data availability.

**Grounding.** Three independent defects, all verified against `print_csv_row`
(`main.c:2581-2844`) by pairing the 87 header names with the 87 format specifiers
programmatically: (a) the UKF `dt` comes from `time_us_32()` deltas
(`ukf_shadow_rp2040.h:1321-1331`) and the CSV's only clock is integer-millisecond `t_ms`;
(b) sensors and both EWMAs run at 10 ms while the CSV is emitted at 50 ms, so four of every
five samples are unlogged; (c) most columns print below float32 precision —
`flex_filtered_raw` at `%.2f`, `ukf_s_pct` at `%.3f`, `ukf_eta` at `%.5f`, and the quaternion
columns at `%.6f` **after** normalisation.

**Resolution.** §8 was restructured:
- **§8.1** documents exactly what the CSV can and cannot support, with a per-column
  recoverability table. It records the one positive result: `pressure_raw` and `flex_raw` *are*
  lossless at `%.2f`, because `read_adc_average` returns `sum/4` over integer reads — but only
  while `ADC_AVERAGE_SAMPLES = 4`.
- **§8.2** introduces four tiers: exact claims only at Tiers 1–2, statistical bounds at Tier 4,
  with the libm carve-out retained and a named fallback (pinning shared
  `logf`/`expf`/`sqrtf`/`atan2f`) requiring user approval. *(Tier 3's status was revised again
  by B2 below.)*
- **§8.3** specifies seven firmware-generated golden-vector fixtures (`F1`–`F7`) and a
  generation method that compiles the **unmodified** golden sources on host via SDK stubs,
  `-Dmain=fw_main` and a virtual clock — making `dt` an input, which is what makes exact replay
  possible. `F4` stores **raw int16** quaternion registers because normalisation is code under
  test.
- **§8.5** gains a per-variable table of which tier can check it.
- The old T20 was split into **T20** (exact Tier 2 closed-loop replay, needs no recorded data,
  now the Stage 4 gate), **T21** (CSV cross-check) and **T22** (Tier 4 hardware).
- §10 item 1 now separates *availability* from *sufficiency*, and asks for the `#` event lines
  alongside any CSV so the command timeline can be reconstructed.

**⚠ Superseded in part by B2 below.** This resolution still treated T21 as a tolerance-based
equivalence check. It is not one.

**Remaining uncertainty (marked in §8.3):** the fixtures are only as authoritative as the SDK
stubs; stub fidelity for timer wrap and alarm semantics must be reviewed against the real Pico
SDK before any "exact" result is trusted. This is now a Stage 0 gate.

### A4 — Further corrections found while re-auditing

| # | Correction | Grounding |
|---|---|---|
| A4.1 | §4.5 previously described only one numerical fallback ladder. There are **two**: `repair_cov4`'s jitter ladder, and a separate Cholesky retry inside sigma-point generation (`+1e-5` diagonal, then `L = diag(√max(Aii, P_FLOOR))`) | `ukf_shadow_rp2040.h:1014-1029` |
| A4.2 | The **centre sigma point is not re-constrained**: `X[0] = x` is copied verbatim while every `X[1..8]` passes through `ukfs_constrain_state()`. A symmetric "clean" implementation would differ | `ukf_shadow_rp2040.h:1031-1047` |
| A4.3 | Segment selection in `plant_schedule_eval` was described loosely. With `PLANT_ANCHOR_COUNT = 3` the loop admits only `i ∈ {0,1}`, i.e. `i = (qc > 50) ? 1 : 0`. The per-anchor positivity rejection is now stated | `plant_schedule.h:22-27` |
| A4.4 | Added `time_us_32()` **wrap at ≈71.6 min** as an equivalence risk: a single unsigned wrap is handled, but long-run behaviour is untested in the reference and the `dt > 0.20` guard would mask a fault rather than surface it | `ukf_shadow_rp2040.h:1321-1331` |
| A4.5 | §1.5 now notes that nothing below the 10 ms sample gate runs on a non-sample iteration, so **every period is quantised to the 10 ms grid** — the reason re-phasing offsets are always multiples of 10 ms | `main.c:3147-3169` |
| A4.6 | §6.2 now states the concrete failure mode: four `create_wall_timer` instances would stay in phase forever and would not be equivalent; the counters must be integers advanced inside one 10 ms callback | §1.6 |

### Findings reviewed and **not** changed

- The **87-column count** was re-verified programmatically (87 header names, 87 format
  specifiers) and is unchanged.
- All constants in §2 and §4 were re-checked against the firmware; no numerical errors were
  found beyond those listed above.

*(The D1 b0 divergence was listed here as still requiring user confirmation. That is now
settled — see B3 below.)*

---

## Second Audit — Corrections Incorporated

A second independent review found two further material problems and settled one
source-of-truth question. Findings A1–A4 above are retained except where explicitly marked
superseded.

### B1 — Scheduler semantics: the three-regime model was wrong

**Finding.** The §1.6 introduced by A1 replaced one oversimplification with another: a fixed
three-regime classification keyed on the overdue amount `D`, including a guaranteed
"two-execution burst" for `P ≤ D < 2P`.

**Grounding — counterexample reproduced.** With `P = 500 ms`, a deadline frozen at
`next_X = 0` and the gate becoming true at `t = 990 ms`, running the firmware rule literally
gives executions at **990, 1000, 1010, 1500** — *three* consecutive executions and **no**
rebasing. The claimed model predicts two. Neighbouring cases diverge further: gate-on at
`t = 980` gives `{980, 990, 1000, 1500}`; gate-on at `t = 1000` clamps and gives a single
execution, `{1000, 1500, 2000}`. The cause is that `now` advances 10 ms per *evaluation*
while `next_X` advances `P` per *execution*, with the clamp re-tested after each single
increment — so the outcome depends jointly on the exact stored deadline and the exact current
time, and admits no simple regime classification.

**Resolution.** §1.6 no longer normalises the behaviour:
- the three-regime table is **deleted** and explicitly withdrawn as false;
- the literal algorithm is presented as the specification, with the 990/1000/1010/1500 trace
  shown step by step and the 980 and 1000 neighbours contrasted;
- the section states plainly that **no closed-form re-entry model is offered, because none has
  been proven against the firmware** — the rule must be executed, not modelled;
- the consequence list was rewritten to record: one `now` read per iteration and never
  refreshed (`main.c:3128`); all deadline evaluation sitting below the 10 ms sample gate, so
  every loop fires only on a 10 ms tick; `next_ukf` described **separately** as ungated,
  never frozen by mode or logging state, and the carrier of `update_plant_schedule()`; the
  blocking `sleep_ms(10000)` in `CALIBRATE` and `sleep_ms(8000)` in `END_RUN` stalling every
  deadline mid-run, with the one-time counter reset at `main.c:3105-3124` occurring only
  *before* the loop is entered; and `next_log`'s idle rebasing as the sole explicit rebase
  outside a gated block;
- the migration requirement is restated as **reproduce the deadline-update algorithm
  literally** — six integer counters, one `now` per iteration, the 10 ms gate above all
  deadline evaluation, `next_X += P`, and the clamp re-tested after each increment, inside
  **one** callback — with **independent `create_wall_timer()` callbacks named as NOT
  baseline-equivalent**, because free-running timers never freeze, never accumulate backlog,
  never produce consecutive-tick catch-up, and never rebase;
- **T15** was rewritten as a *differential* test: the ported scheduler and the host-built
  firmware scheduler are driven with the same command timeline and must produce identical
  execution-tick sets, asserted against recorded ticks (including the 990/980/1000 cases) and
  **not** against any regime model.

### B2 — T21 is an observational cross-check, not an equivalence test

**Finding.** T21 was still described as a tolerance-based equivalence check on the 87-column
CSV, with per-column tolerances derived from `printf` precision.

**Grounding.** Acquisition runs at 10 ms and the CSV logs at 50 ms, so **four of every five
acquisition samples are unobserved**. Precision is therefore not the binding constraint —
observability is. Demonstrated with `FLEX_EWMA_ALPHA = 0.20` from a common state of 1500
counts: histories `A = (1500,1500,1500,1500,1700)` and `B = (1900,1850,1800,1750,1700)` both
log `flex_raw = 1700` at the logged tick, yet yield filtered states of **1540.000** and
**1687.008** — a 147-count divergence, ≈30 % of `FLEX_SPAN_COUNTS`. Tightening a tolerance
cannot recover an input that was never recorded, and the ambiguity propagates from
`pressure_filtered` into the pressure PI, and from `flex_filtered_raw` into the UKF and every
downstream controller state.

**Resolution.**
- New **§8.1(d)** states the ambiguity with the worked EWMA counterexample, and the section
  conclusion now says the CSV **cannot** establish equivalence of internal state at any
  tolerance.
- **§8.2** reclassifies Tier 3 as *observational cross-check — no equivalence claim of any
  kind*, and notes that Tier 3 cannot even falsify determinism the way Tier 4 can.
- **T21** is retitled *CSV observational cross-check (not an equivalence test)* and restricted
  to gross consistency: discrete/boolean sequences, trends, ranges, sign, and a coherent `#`
  event narrative. It is explicitly forbidden from reconstructing EWMA, UKF or controller
  state, and from being reported as equivalence evidence.
- **§8.5**'s per-variable table replaces "tolerance only" with observational qualifiers, marks
  `pressure_filtered`/`flex_filtered_raw` as **not comparable**, and notes that even a matching
  discrete sequence is plausibility evidence only, since the same sequence is reachable from
  different internal trajectories.
- **Stage 7** no longer gates on T21; **§10 item 1** notes the gap blocks no equivalence claim;
  **§8.6** items 1 and 6 and **§12** were updated so exact equivalence rests solely on T20
  Tier 2 golden fixtures with complete input sequences, initialisation and timing.

### B3 — b0 source of truth: settled, not an open decision

**Finding.** §11 D1 left the `target_b0` discrepancy as something to "confirm before Stage 3",
which wrongly framed a settled question as an open tuning decision.

**Grounding.** `main.c:963-966` executes `target_b0 = 0.80f * B0_NOMINAL + 0.20f * next.b0`,
with the alternative `target_b0 = next.b0` present but commented out at `main.c:960`.
`GAIN_SCHEDULING.md:25` describes the mixing as removed. The project's migration policy designates the validated
firmware — not its prose — as the behavioural reference.

**Resolution.** §11 D1 now states that for the baseline migration the **executable validated
firmware is authoritative**: the port reproduces `0.80 * B0_NOMINAL + 0.20 * next.b0` exactly
and does not substitute the `GAIN_SCHEDULING.md` description. The discrepancy remains
documented — it is the highest-risk trap for anyone working from the design note — but it is
explicitly **not** an open tuning decision and must not be re-litigated during implementation.
Adopting the prose formulation would be a controller change, outside migration scope, and
would require a new validated reference. **T8** now pins the 80/20 expression so any drift
toward the prose version fails loudly.

### A4 findings retained

All six A4 corrections remain in force and were re-checked against the firmware during this
pass: both Cholesky/fallback ladders (A4.1), the unconstrained centre sigma point (A4.2), the
exact `plant_schedule` segment selection (A4.3), the `time_us_32()` wrap risk (A4.4), the
10 ms timing quantisation (A4.5, now also §1.6 consequence 2), and the
single-callback/single-counter ROS 2 equivalence architecture (A4.6, now strengthened by B1).
