// Copyright 2026 Sebastian Parra
#ifndef SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_SCHED_H_
#define SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_SCHED_H_
#include "soft_glove_core/soft_glove_core_ctrl.h"
#define SGC_SAMPLE_PERIOD_MS 10
#define SGC_UKF_SHADOW_LOOP_MS 50
#define SGC_LOG_PERIOD_MS 50
#define SGC_FILL_MIN_MS 5
#define SGC_FILL_MAX_MS 80
#define SGC_VENT_MIN_MS 10
#define SGC_VENT_MAX_MS 100
#define SGC_REVERSAL_LOCKOUT_MS 200
#define SGC_HARD_PRESSURE_KPA 200.0f
typedef enum
{
  SGC_STATE_UNKNOWN = 99,
  SGC_STATE_VENT = -1,
  SGC_STATE_HOLD = 0,
  SGC_STATE_FILL = 1,
  SGC_STATE_OFF = 2
} SgcPneumaticState;
typedef enum
{
  SGC_CTRL_NONE = 0,
  SGC_CTRL_PI = 1,
  SGC_CTRL_ADRC = 2,
  SGC_CTRL_PRESSURE = 3,
  SGC_CTRL_PI_UKF = 4,
  SGC_CTRL_ADRC_UKF = 5
} SgcControllerMode;
typedef struct
{
  uint32_t next_sample, next_ukf, next_eso, next_outer, next_pressure, next_log;
  bool started;
} SgcSchedState;
typedef struct
{
  SgcPneumaticState pneumatic_state;
  bool pulse_active;
  uint32_t pulse_end_ms;
  uint32_t last_pulse_ms;
  SgcPneumaticState last_pulse_direction;
  SgcPneumaticState pending_reversal_direction;
  uint32_t reversal_block_until_ms;
  SgcControllerMode controller_mode;
  bool control_enabled;
  bool logging_enabled;
  bool command_abort;
  bool outer_updated_since_log;
  bool eso_updated_since_log;
  bool pressure_updated_since_log;
  float position_ref_pct;
  float previous_q_ref_pct;
} SgcSupState;
typedef struct
{
  uint32_t now_ms;
  float pressure_kpa;
  float pressure_filtered;
  float flex_filtered_raw;
  float latest_position_pct;
  bool cmd_abort;
  bool set_enable; bool enable_value;
  bool set_mode; SgcControllerMode mode_value;
  bool set_reset;
  bool set_logging; bool logging_value;
  bool set_q; float q_value;
} SgcTickInput;
typedef struct
{
  bool set_valves; SgcPneumaticState valve_cmd;
  bool arm_pulse; uint32_t arm_pulse_ms;
  bool cancel_pulse;
  bool pump_off;
  bool outputs_off;
  bool shutdown;
  int exit_code;
  bool ran_sample, ran_ukf, ran_eso, ran_outer, ran_pressure, ran_log;
} SgcActions;
void sgc_sched_init(SgcSchedState *s, uint32_t now);
void sgc_sup_init(SgcSupState *sup);
void sgc_sup_set_state(SgcSupState *sup, SgcPneumaticState state, SgcActions *out);
void sgc_sup_start_pulse(
  SgcSupState *sup, SgcPneumaticState state,
  uint32_t pulse_ms, uint32_t now_ms, SgcActions *out);
void sgc_sup_update_active_pulse(SgcSupState *sup, uint32_t now_ms, SgcActions *out);
void sgc_sup_execute_pressure_control(
  SgcSupState *sup, SgcCtrlState *ctrl,
  float reference, float pressure, uint32_t now_ms, SgcActions *out);
void sgc_sup_emergency_shutdown(
  SgcSupState *sup, SgcCtrlState *ctrl,
  const char *reason, float value, SgcActions *out);
float sgc_sup_active_position_feedback_pct(
  SgcControllerMode mode,
  const SgcUkfTelemetry *telem, float latest_position_pct);
void sgc_sup_select_controller_mode(
  SgcSupState *sup, SgcCtrlState *ctrl,
  SgcUkfState *ukf, SgcControllerMode new_mode, const SgcTickInput *in, SgcActions *out);
void sgc_sup_apply_reference_change(
  SgcSupState *sup, SgcCtrlState *ctrl,
  SgcUkfState *ukf, float new_q_ref, const SgcTickInput *in);
void sgc_sup_reset_to_safe_none(
  SgcSupState *sup, SgcCtrlState *ctrl,
  const SgcTickInput *in);
int sgc_sched_tick(
  SgcSchedState *sched, SgcSupState *sup,
  SgcUkfState *ukf, SgcCtrlState *ctrl, const SgcTickInput *in, SgcActions *out);
#endif  // SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_SCHED_H_
