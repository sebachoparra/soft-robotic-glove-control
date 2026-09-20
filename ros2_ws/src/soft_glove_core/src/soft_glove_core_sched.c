// Copyright 2026 Sebastian Parra
#include "soft_glove_core/soft_glove_core_sched.h"
#include <math.h>
#include <string.h>
void sgc_sched_init(SgcSchedState *s, uint32_t now)
{
  s->next_sample = now;
  s->next_ukf = now;
  s->next_eso = now;
  s->next_outer = now;
  s->next_pressure = now;
  s->next_log = now;
  s->started = true;
}
void sgc_sup_init(SgcSupState *sup)
{
  memset(sup, 0, sizeof(*sup));
  sup->pneumatic_state = SGC_STATE_UNKNOWN;
  sup->last_pulse_direction = SGC_STATE_HOLD;
  sup->pending_reversal_direction = SGC_STATE_HOLD;
  sup->controller_mode = SGC_CTRL_NONE;
}
void sgc_sup_set_state(SgcSupState *sup, SgcPneumaticState state, SgcActions *out)
{
  sup->pneumatic_state = state;
  if (state == SGC_STATE_FILL || state == SGC_STATE_HOLD || state == SGC_STATE_VENT) {
    out->set_valves = true;
    out->valve_cmd = state;
  }
}
void sgc_sup_start_pulse(
  SgcSupState *sup, SgcPneumaticState state,
  uint32_t pulse_ms, uint32_t now_ms, SgcActions *out)
{
  if (sup->pulse_active) {out->cancel_pulse = true;}
  sgc_sup_set_state(sup, state, out);
  sup->pulse_active = true;
  sup->pulse_end_ms = now_ms + pulse_ms;
  out->arm_pulse = true;
  out->arm_pulse_ms = pulse_ms;
}
void sgc_sup_update_active_pulse(SgcSupState *sup, uint32_t now_ms, SgcActions *out)
{
  if (!sup->pulse_active) {return;}
  if ((int32_t)(now_ms - sup->pulse_end_ms) >= 0) {
    sup->pulse_active = false;
    out->cancel_pulse = true;
    sgc_sup_set_state(sup, SGC_STATE_HOLD, out);
  }
}
void sgc_sup_execute_pressure_control(
  SgcSupState *sup, SgcCtrlState *ctrl,
  float reference, float pressure, uint32_t now_ms, SgcActions *out)
{
  ctrl->last_pressure_pi = sgc_ctrl_pressure_pi_update(ctrl, reference, pressure);
  sup->last_pulse_ms = 0;
  if (sup->pulse_active) {return;}
  if (fabsf(ctrl->last_pressure_pi.error) <= SGC_PRESSURE_DEADBAND_KPA) {
    sup->pending_reversal_direction = SGC_STATE_HOLD;
    sgc_sup_set_state(sup, SGC_STATE_HOLD, out);
    return;
  }
  SgcPneumaticState requested_state =
    (ctrl->last_pressure_pi.error > 0.0f) ? SGC_STATE_FILL : SGC_STATE_VENT;
  if (sup->pending_reversal_direction != SGC_STATE_HOLD) {
    if (requested_state != sup->pending_reversal_direction) {
      sup->pending_reversal_direction = requested_state;
      sup->reversal_block_until_ms = now_ms + SGC_REVERSAL_LOCKOUT_MS;
      sgc_sup_set_state(sup, SGC_STATE_HOLD, out);
      return;
    }
    if ((int32_t)(now_ms - sup->reversal_block_until_ms) < 0) {
      sgc_sup_set_state(sup, SGC_STATE_HOLD, out);
      return;
    }
    sup->pending_reversal_direction = SGC_STATE_HOLD;
  } else if (sup->last_pulse_direction != SGC_STATE_HOLD &&  // NOLINT(readability/braces)
    requested_state != sup->last_pulse_direction)
  {
    sup->pending_reversal_direction = requested_state;
    sup->reversal_block_until_ms = now_ms + SGC_REVERSAL_LOCKOUT_MS;
    sgc_sup_set_state(sup, SGC_STATE_HOLD, out);
    return;
  }
  if (requested_state == SGC_STATE_FILL) {
    uint32_t pulse = sgc_magnitude_to_pulse(ctrl->last_pressure_pi.output,
      SGC_FILL_MIN_MS, SGC_FILL_MAX_MS);
    sup->last_pulse_ms = pulse;
    sup->last_pulse_direction = SGC_STATE_FILL;
    sgc_sup_start_pulse(sup, SGC_STATE_FILL, pulse, now_ms, out);
    return;
  }
  uint32_t pulse = sgc_magnitude_to_pulse(-ctrl->last_pressure_pi.output,
    SGC_VENT_MIN_MS, SGC_VENT_MAX_MS);
  sup->last_pulse_ms = pulse;
  sup->last_pulse_direction = SGC_STATE_VENT;
  sgc_sup_start_pulse(sup, SGC_STATE_VENT, pulse, now_ms, out);
}
void sgc_sup_emergency_shutdown(
  SgcSupState *sup, SgcCtrlState *ctrl,
  const char *reason, float value, SgcActions *out)
{
  (void)ctrl; (void)reason; (void)value;
  sup->control_enabled = false;
  sup->logging_enabled = false;
  sup->controller_mode = SGC_CTRL_NONE;
  out->pump_off = true;
  if (sup->pulse_active) {sup->pulse_active = false; out->cancel_pulse = true;}
  sgc_sup_set_state(sup, SGC_STATE_VENT, out);
  out->outputs_off = true;
  sup->pneumatic_state = SGC_STATE_OFF;
  out->shutdown = true;
}
float sgc_sup_active_position_feedback_pct(
  SgcControllerMode mode,
  const SgcUkfTelemetry *telem, float latest_position_pct)
{
  if (mode == SGC_CTRL_PI_UKF || mode == SGC_CTRL_ADRC_UKF) {
    if (sgc_ctrl_ukf_feedback_available(telem)) {
      return sgc_clampf(telem->s_hat, 0.0f, 100.0f);
    }
  }
  return latest_position_pct;
}
void sgc_sup_select_controller_mode(
  SgcSupState *sup, SgcCtrlState *ctrl,
  SgcUkfState *ukf, SgcControllerMode new_mode, const SgcTickInput *in, SgcActions *out)
{
  sup->controller_mode = new_mode;
  sup->control_enabled = new_mode != SGC_CTRL_NONE;
  ctrl->pressure_ref_kpa = sgc_clampf(ctrl->pressure_ref_kpa,
    SGC_PRESSURE_REF_MIN_KPA, SGC_PRESSURE_REF_MAX_KPA);
  if (new_mode == SGC_CTRL_PI || new_mode == SGC_CTRL_PI_UKF) {
    sgc_ctrl_update_plant_schedule(ctrl, sgc_ctrl_ukf_feedback_available(&ukf->telem),
      ukf->telem.s_hat, ukf->telem.eta_hat);
    sgc_ctrl_initialize_pi_bumpless(ctrl, sup->position_ref_pct,
      sgc_sup_active_position_feedback_pct(new_mode, &ukf->telem, in->latest_position_pct));
  } else if (new_mode == SGC_CTRL_ADRC || new_mode == SGC_CTRL_ADRC_UKF) {
    ctrl->ff_gamma_state = 0.0f;
    sgc_ctrl_update_plant_schedule(ctrl, sgc_ctrl_ukf_feedback_available(&ukf->telem),
      ukf->telem.s_hat, ukf->telem.eta_hat);
    sgc_ctrl_initialize_adrc_bumpless(ctrl, sup->position_ref_pct,
      sgc_sup_active_position_feedback_pct(new_mode, &ukf->telem, in->latest_position_pct));
  } else if (new_mode == SGC_CTRL_PRESSURE) {
    sup->position_ref_pct = in->latest_position_pct;
    sup->previous_q_ref_pct = sup->position_ref_pct;
    ctrl->pressure_ref_kpa = sgc_clampf(in->pressure_filtered,
      SGC_PRESSURE_REF_MIN_KPA, SGC_PRESSURE_REF_MAX_KPA);
    ctrl->ff_gamma_state = 0.0f;
    sgc_ctrl_reset_pressure_pi(ctrl);
    bool had_pulse = sup->pulse_active;
    sup->pulse_active = false;
    sup->last_pulse_ms = 0;
    sup->last_pulse_direction = SGC_STATE_HOLD;
    sup->pending_reversal_direction = SGC_STATE_HOLD;
    sup->reversal_block_until_ms = 0u;
    if (had_pulse) {out->cancel_pulse = true;}
    sgc_sup_set_state(sup, SGC_STATE_HOLD, out);
    sgc_ctrl_reset_outer_diagnostics(ctrl);
    sup->outer_updated_since_log = false;
    sup->eso_updated_since_log = false;
  }
  sgc_sup_set_state(sup, SGC_STATE_HOLD, out);
}
void sgc_sup_apply_reference_change(
  SgcSupState *sup, SgcCtrlState *ctrl,
  SgcUkfState *ukf, float new_q_ref, const SgcTickInput *in)
{
  (void)in;
  new_q_ref = sgc_clampf(new_q_ref, 0.0f, 100.0f);
  float old_q_ref = sup->position_ref_pct;
  sup->previous_q_ref_pct = old_q_ref;
  sup->position_ref_pct = new_q_ref;
  sgc_ctrl_update_plant_schedule(ctrl, sgc_ctrl_ukf_feedback_available(&ukf->telem),
    ukf->telem.s_hat, ukf->telem.eta_hat);
  sgc_ctrl_latch_step_feedforward(ctrl, old_q_ref, new_q_ref);
}
void sgc_sup_reset_to_safe_none(
  SgcSupState *sup, SgcCtrlState *ctrl,
  const SgcTickInput *in)
{
  sup->controller_mode = SGC_CTRL_NONE;
  sup->control_enabled = false;
  sup->position_ref_pct = in->latest_position_pct;
  sup->previous_q_ref_pct = sup->position_ref_pct;
  ctrl->pressure_ref_kpa = sgc_clampf(in->pressure_filtered,
    SGC_PRESSURE_REF_MIN_KPA, SGC_PRESSURE_REF_MAX_KPA);
  ctrl->active_gain_region_id = 0;
  ctrl->active_kp_position = SGC_PLANT_P1 * 1.906f / 0.818f;
  ctrl->active_ki_position = SGC_PLANT_P1 / 0.818f;
  ctrl->schedule_input_valid = false;
  ctrl->target_b0 = SGC_B0_FALLBACK;
  ctrl->active_b0 = SGC_B0_FALLBACK;
  ctrl->adrc_model_k = 0.0f;
  ctrl->adrc_model_tau_s = 0.0f;
  ctrl->adrc_b0_identified = SGC_B0_FALLBACK;
  ctrl->adrc_model_valid = false;
  ctrl->step_ff_kpa = 0.0f;
  ctrl->ff_gamma_state = 0.0f;
  sgc_ctrl_initialize_pi_bumpless(ctrl, sup->position_ref_pct, in->latest_position_pct);
  sgc_ctrl_initialize_adrc_bumpless(ctrl, sup->position_ref_pct, in->latest_position_pct);
  sgc_ctrl_reset_pressure_pi(ctrl);
  sup->pulse_active = false;
  sup->last_pulse_ms = 0;
  sup->last_pulse_direction = SGC_STATE_HOLD;
  sup->pending_reversal_direction = SGC_STATE_HOLD;
  sup->reversal_block_until_ms = 0u;
  sup->pneumatic_state = SGC_STATE_HOLD;
  sgc_ctrl_reset_outer_diagnostics(ctrl);
  sup->outer_updated_since_log = false;
  sup->eso_updated_since_log = false;
}
int sgc_sched_tick(
  SgcSchedState *sched, SgcSupState *sup,
  SgcUkfState *ukf, SgcCtrlState *ctrl, const SgcTickInput *in, SgcActions *out)
{
  memset(out, 0, sizeof(*out));
  uint32_t now = in->now_ms;
  if (!sched->started) {sgc_sched_init(sched, now);}
  if (in->set_q) {sgc_sup_apply_reference_change(sup, ctrl, ukf, in->q_value, in);}
  if (in->set_mode) {sgc_sup_select_controller_mode(sup, ctrl, ukf, in->mode_value, in, out);}
  if (in->set_reset) {sgc_sup_reset_to_safe_none(sup, ctrl, in);}
  if (in->set_logging) {sup->logging_enabled = in->logging_value;}
  if (in->cmd_abort) {sup->command_abort = true;}
  if (sup->command_abort) {
    sgc_sup_emergency_shutdown(sup, ctrl, "OPERATOR_ABORT", in->pressure_filtered, out);
    out->exit_code = 2;
    return 2;
  }
  sgc_sup_update_active_pulse(sup, now, out);
  if ((int32_t)(now - sched->next_sample) < 0) {return 0;}
  out->ran_sample = true;
  sched->next_sample += SGC_SAMPLE_PERIOD_MS;
  if ((int32_t)(now - sched->next_sample) >= SGC_SAMPLE_PERIOD_MS) {
    sched->next_sample = now + SGC_SAMPLE_PERIOD_MS;
  }
  if (in->pressure_kpa >= SGC_HARD_PRESSURE_KPA) {
    sgc_sup_emergency_shutdown(sup, ctrl, "OVERPRESSURE", in->pressure_kpa, out);
    out->exit_code = 1;
    return 1;
  }
  if ((int32_t)(now - sched->next_ukf) >= 0) {
    out->ran_ukf = true;
    /* The Stage 2 state is supplied by acquisition; BNO input is outside Stage 4. */
    sgc_ctrl_update_plant_schedule(ctrl, sgc_ctrl_ukf_feedback_available(&ukf->telem),
      ukf->telem.s_hat, ukf->telem.eta_hat);
    sched->next_ukf += SGC_UKF_SHADOW_LOOP_MS;
    if ((int32_t)(now - sched->next_ukf) >= (int32_t)SGC_UKF_SHADOW_LOOP_MS) {
      sched->next_ukf = now + SGC_UKF_SHADOW_LOOP_MS;
    }
  }
  if ((sup->controller_mode == SGC_CTRL_ADRC || sup->controller_mode == SGC_CTRL_ADRC_UKF) &&
    sup->control_enabled && (int32_t)(now - sched->next_eso) >= 0)
  {
    out->ran_eso = true;
    sgc_ctrl_update_active_b0(ctrl);
    if (sup->controller_mode == SGC_CTRL_ADRC_UKF &&
      !sgc_ctrl_ukf_feedback_available(&ukf->telem))
    {
      sup->eso_updated_since_log = false;
    } else {
      sgc_ctrl_leso_update(ctrl, ctrl->pressure_ref_kpa,
        sgc_sup_active_position_feedback_pct(sup->controller_mode,
          &ukf->telem, in->latest_position_pct));
      sup->eso_updated_since_log = true;
    }
    sched->next_eso += SGC_ESO_LOOP_MS;
    if ((int32_t)(now - sched->next_eso) >= SGC_ESO_LOOP_MS) {
      sched->next_eso = now + SGC_ESO_LOOP_MS;
    }
  }
  if (sup->control_enabled && (sup->controller_mode == SGC_CTRL_PI ||
    sup->controller_mode == SGC_CTRL_PI_UKF || sup->controller_mode == SGC_CTRL_ADRC ||
    sup->controller_mode == SGC_CTRL_ADRC_UKF) && (int32_t)(now - sched->next_outer) >= 0)
  {
    out->ran_outer = true;
    if (sup->controller_mode == SGC_CTRL_PI || sup->controller_mode == SGC_CTRL_PI_UKF) {
      if (sup->controller_mode == SGC_CTRL_PI_UKF &&
        !sgc_ctrl_ukf_feedback_available(&ukf->telem))
      {
        ctrl->pressure_ref_kpa = sgc_clampf(in->pressure_filtered,
          SGC_PRESSURE_REF_MIN_KPA, SGC_PRESSURE_REF_MAX_KPA);
        ctrl->previous_position_error = 0.0f;
      } else {
        ctrl->last_position_pi = sgc_ctrl_position_pi_update(ctrl, sup->position_ref_pct,
          sgc_sup_active_position_feedback_pct(sup->controller_mode,
            &ukf->telem, in->latest_position_pct));
        sup->outer_updated_since_log = true;
      }
    } else if (sup->controller_mode == SGC_CTRL_ADRC ||  // NOLINT(readability/braces)
      sup->controller_mode == SGC_CTRL_ADRC_UKF)
    {
      if (sup->controller_mode == SGC_CTRL_ADRC_UKF &&
        !sgc_ctrl_ukf_feedback_available(&ukf->telem))
      {
        ctrl->pressure_ref_kpa = sgc_clampf(in->pressure_filtered,
          SGC_PRESSURE_REF_MIN_KPA, SGC_PRESSURE_REF_MAX_KPA);
        ctrl->ff_gamma_state = 0.0f;
      } else {
        ctrl->last_ladrc = sgc_ctrl_ladrc_update(ctrl, sup->position_ref_pct);
        sup->outer_updated_since_log = true;
      }
    }
    sched->next_outer += SGC_OUTER_LOOP_MS;
    if ((int32_t)(now - sched->next_outer) >= SGC_OUTER_LOOP_MS) {
      sched->next_outer = now + SGC_OUTER_LOOP_MS;
    }
  }
  if (sup->control_enabled && sup->controller_mode != SGC_CTRL_NONE &&
    (int32_t)(now - sched->next_pressure) >= 0)
  {
    out->ran_pressure = true;
    sgc_sup_execute_pressure_control(sup, ctrl, ctrl->pressure_ref_kpa,
      in->pressure_filtered, now, out);
    sup->pressure_updated_since_log = true;
    sched->next_pressure += SGC_PRESSURE_LOOP_MS;
    if ((int32_t)(now - sched->next_pressure) >= SGC_PRESSURE_LOOP_MS) {
      sched->next_pressure = now + SGC_PRESSURE_LOOP_MS;
    }
  }
  if (sup->logging_enabled && (int32_t)(now - sched->next_log) >= 0) {
    out->ran_log = true;
    sup->outer_updated_since_log = false;
    sup->eso_updated_since_log = false;
    sup->pressure_updated_since_log = false;
    sched->next_log += SGC_LOG_PERIOD_MS;
    if ((int32_t)(now - sched->next_log) >= SGC_LOG_PERIOD_MS) {
      sched->next_log = now + SGC_LOG_PERIOD_MS;
    }
  } else if (!sup->logging_enabled) {
    sched->next_log = now + SGC_LOG_PERIOD_MS;
  }
  return 0;
}
