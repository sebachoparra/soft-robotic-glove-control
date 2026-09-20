# Position PI Controller and Gain Scheduling

Sources: `reference/rp2040_validated/plant_schedule.h`,
`reference/rp2040_validated/GAIN_SCHEDULING.md`, `reference/rp2040_validated/main.c`
(golden, read-only reference), and the ROS 2 port
`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c` /
`soft_glove_core_plant_schedule` (via `soft_glove_core.h`).

## 1. Position PI structure (reference)

The reference position PI is an **incremental** (velocity-form) PI on the outer
position loop, producing an increment to the pressure reference rather than an
absolute pressure command. In the ROS 2 port, `sgc_ctrl_position_pi_update()`
(`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:122-210`) implements:

```
error            = q_ref - q
p_increment_kpa  = active_kp_position * (error - previous_error)
i_increment_kpa  = active_ki_position * SGC_OUTER_TS_S * error
delta_pref_kpa   = p_increment_kpa + i_increment_kpa
pressure_ref_raw_kpa = pressure_ref_kpa + delta_pref_kpa
```
(lines 131-155), followed by rate-limiting/saturation via
`sgc_apply_common_pref_limits()` (lines 160-166; see `docs/pressure_control.md` §3) and
storage of the resulting `pressure_ref_kpa` and updated `previous_position_error`
(lines 200-207). `SGC_OUTER_TS_S` corresponds to the 500 ms outer-loop period
(`SGC_OUTER_LOOP_MS = 500`,
`ros2_ws/src/soft_glove_core/include/soft_glove_core/soft_glove_core_ctrl.h:8`), which
matches `GAIN_SCHEDULING.md`'s statement that "PI usa las ganancias disponibles en cada
ejecución externa (500 ms)" (`reference/rp2040_validated/GAIN_SCHEDULING.md:22`).

Default/initial gains before any scheduling input is valid, both in the reference
(`active_kp_position = PLANT_P1 * 1.906f / 0.818f`,
`active_ki_position = PLANT_P1 / 0.818f`, `reference/rp2040_validated/main.c:345-346,856-857`)
and in the ROS 2 port (`sgc_ctrl_state_init()`,
`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:7-15`, same expression using
`SGC_PLANT_P1`). `1.906` and `0.818` are the `tau_up`/`k_up` values of the `q=30` anchor
(§2 below), i.e. this is the 30 kPa UP-branch model evaluated with the PI-design formulas
of §3.

## 2. Gain scheduling: plant anchors

`reference/rp2040_validated/plant_schedule.h:5-13`:

```c
#define PLANT_P1 0.40f
typedef struct { float q, k_up, tau_up, k_down, tau_down; } PlantAnchor;
static const PlantAnchor PLANT_ANCHORS[] = {
    {30.0f, 0.818f, 1.906f, 1.342f, 5.262f},
    {50.0f, 1.170f, 0.177f, 0.831f, 2.970f},
    {70.0f, 0.945f, 0.119f, 0.520f, 4.194f}
};
```

Three anchors at `q = 30, 50, 70` kPa, each with an UP-branch (`k_up`, `tau_up`) and
DOWN-branch (`k_down`, `tau_down`) first-order plant model `G(s) = K/(tau*s+1)`. Per
`GAIN_SCHEDULING.md:17`, these are "rounded identification values" (the full-precision
identification CSV is not included), so no nine-digit-exact match against that CSV is
claimed.

### 2.1 `plant_schedule_eval()` — interpolation algorithm

`reference/rp2040_validated/plant_schedule.h:15-42`:

1. Clamp `q` to `[PLANT_ANCHORS[0].q, PLANT_ANCHORS[last].q] = [30, 70]` (line 18-19).
   Values outside `[30,70]` retain the nearest anchor's model — per
   `GAIN_SCHEDULING.md:31`, "Fuera de las anclas [30,70] se retiene el extremo: 20 usa
   el modelo de 30 y 80 el de 70," and this is explicitly *not* claimed valid outside
   the experimental range `[20,80]`.
2. Clamp `eta` to `[0,1]` (line 20).
3. Locate the bracketing anchor pair `a`, `b` by linear scan (line 21-23).
4. Validate positivity of `k_up`, `k_down`, `tau_up`, `tau_down` for both anchors
   (line 24-27) — returns `false` (invalid) otherwise.
5. Linear-interpolate **in log space** between anchors by fractional position
   `w = (q-a.q)/(b.q-a.q)`:
   `ku = (1-w)ln(a.k_up) + w·ln(b.k_up)`, similarly `kd`, `tu`, `td` (lines 28-32).
6. Mix UP/DOWN branches (also in log space) by `eta`:
   `k   = exp((1-eta)*ku + eta*kd)`
   `tau = exp((1-eta)*tu + eta*td)` (lines 34-36).
7. Derive PI gains and `b0` from the mixed model using the fixed closed-loop pole
   `PLANT_P1 = 0.40`:
   ```
   ki = PLANT_P1 / k
   kp = ki * tau
   b0 = k / tau
   ```
   (line 37). This is a pole-cancellation PI design: for `G(s)=K/(tau·s+1)` and
   `C(s)=Kp+Ki/s`, choosing `Kp = tau*Ki` cancels the plant pole and yields the
   closed loop `T(s) = p1/(s+p1)` with `p1 = PLANT_P1 = 0.40` — stated explicitly in
   `GAIN_SCHEDULING.md:49`: "el PI continuo C(s)=Kp+Ki/s cancela algebraicamente el
   polo y da T(s)=p1/(s+p1)." The associated 2%-settling time is
   `-ln(0.02)/0.40 = 9.780 s`, described there as "una propiedad nominal del modelo
   congelado, no una garantía del sistema programado, discretizado y neumático real."
8. Validate finiteness/positivity of `kp`, `ki`, `b0` (line 38-39) before returning.

The ROS 2 port evaluates the same table via `sgc_plant_schedule_eval()` (declared in
`soft_glove_core.h`, called from `sgc_ctrl_update_plant_schedule()`,
`ros2_ws/src/soft_glove_core/src/soft_glove_core_ctrl.c:25-49`), which — on success —
sets `active_kp_position = next.kp`, `active_ki_position = next.ki`, and records
`adrc_model_k`, `adrc_model_tau_s`, `adrc_b0_identified = next.b0`
(`soft_glove_core_ctrl.c:33-40`); see `docs/position_control_adrc.md` for how `next.b0`
feeds LADRC's `target_b0`. If scheduling input is invalid (UKF not available, or
`plant_schedule_eval` itself fails), `schedule_input_valid` is left false and the
**previous** schedule (gains) is retained, matching `GAIN_SCHEDULING.md:33`: "Si el UKF
no es válido o s/eta no son finitos, gs_valid=0 y se conserva el último scheduling."

### 2.2 Scheduling input

Per `GAIN_SCHEDULING.md:19,29`, the schedule is evaluated once per UKF update (50 ms)
using `s_hat` and `eta_hat` read-only from the estimator ("main.c lee s_hat y eta_hat
sin escribir al estimador"); it is used for **both** PI and ADRC/LADRC modes, and
requires a valid UKF regardless of which controller (PI or ADRC) is feeding back from
FLEX. `sgc_ctrl_update_plant_schedule()` takes `ukf_available, s_hat, eta_hat` directly
as arguments (`soft_glove_core_ctrl.c:25-28`), consistent with this description.

## 3. Not specified in inspected source

- The exact call site/cadence at which `main.c` invokes `plant_schedule_eval()` on the
  RP2040 (i.e. whether it is literally coupled to the 50 ms UKF tick inside `main.c`)
  was not traced beyond the `GAIN_SCHEDULING.md` narrative description cited above.
- `PI_GAIN_ID`/`pi_gain_id=0"` logging semantics ("scheduling continuo") are mentioned
  in `GAIN_SCHEDULING.md:43` but their exact CSV column wiring was not inspected here.
