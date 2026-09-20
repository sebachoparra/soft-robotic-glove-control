// Copyright 2026 Sebastian Parra

#include "soft_glove_core/soft_glove_core_ctrl.h"
#include <math.h>
#include <string.h>

void sgc_ctrl_state_init(SgcCtrlState *st)
{
  memset(st, 0, sizeof(*st));
  st->active_kp_position = SGC_PLANT_P1 * 1.906f / 0.818f;
  st->active_ki_position = SGC_PLANT_P1 / 0.818f;
  st->active_b0 = SGC_B0_FALLBACK;
  st->target_b0 = SGC_B0_FALLBACK;
  st->adrc_b0_identified = SGC_B0_FALLBACK;
}
bool sgc_ctrl_ukf_feedback_available(const SgcUkfTelemetry *t)
{
  return
    t->valid &&
    t->reference_valid &&
    t->theta_valid &&
    isfinite(t->s_hat);
}

void sgc_ctrl_update_plant_schedule(
  SgcCtrlState *st, bool ukf_available, float s_hat,
  float eta_hat)
{
  SgcPlantSchedule next;
  st->schedule_input_valid = ukf_available &&
    sgc_plant_schedule_eval(s_hat, eta_hat, &next);
  if (!st->schedule_input_valid) {return;}
  st->scheduled_plant = next;
  st->active_gain_region_id = 0;
  st->active_kp_position = next.kp;
  st->active_ki_position = next.ki;
  st->adrc_model_k = next.k;
  st->adrc_model_tau_s = next.tau;
  st->adrc_b0_identified = next.b0;
  st->adrc_model_valid = true;
    //target_b0 = next.b0;  // NOLINT(whitespace/comments)

    /* Scheduling continuo con la regularizacion original. */
  st->target_b0 =
    0.80f * SGC_B0_NOMINAL +
    0.20f * next.b0;

    /* Do not reset PI output/error history, LESO, or feedforward here. */
}

float sgc_ctrl_compute_adrc_feedforward(
  float q_from,
  float q_to,
  float model_k,
  bool valid_nominal
)
{
  if (
    !valid_nominal ||
    !isfinite(model_k) ||
    fabsf(model_k) < 1e-5f)
  {
    return 0.0f;
  }

  float ff =
    SGC_LADRC_FF_MODEL_WEIGHT *
    (q_to - q_from) /
    model_k;

  if (ff > SGC_LADRC_FF_MAX_UP_KPA) {
    ff = SGC_LADRC_FF_MAX_UP_KPA;
  }

  if (ff < -SGC_LADRC_FF_MAX_DOWN_KPA) {
    ff = -SGC_LADRC_FF_MAX_DOWN_KPA;
  }

  return ff;
}

void sgc_ctrl_update_active_b0(SgcCtrlState *st)
{
  const float alpha =
    SGC_ESO_TS_S /
    (SGC_B0_SMOOTH_TAU_S + SGC_ESO_TS_S);

  const float old_b0 =
    st->active_b0;

  float new_b0 =
    old_b0 +
    alpha *
    (st->target_b0 - old_b0);

  if (
    !isfinite(new_b0) ||
    new_b0 <= 1e-5f)
  {
    new_b0 = SGC_B0_FALLBACK;
  }

  if (
    fabsf(st->target_b0 - new_b0) <
    1e-5f)
  {
    new_b0 = st->target_b0;
  }

    /*
     * Conserva:
     *     q_dot_hat = z2 + b0 * P_ref
     */
  st->eso_z2_pct_s +=
    (old_b0 - new_b0) *
    st->pressure_ref_kpa;

  st->active_b0 =
    new_b0;
}

SgcPositionPIResult sgc_ctrl_position_pi_update(
  SgcCtrlState *st,
  float q_ref,
  float q
)
{
  SgcPositionPIResult r;
  memset(&r, 0, sizeof(r));

  r.error =
    q_ref - q;

  r.previous_error =
    st->previous_position_error;

  r.p_increment_kpa =
    st->active_kp_position *
    (r.error - st->previous_position_error);

  r.i_increment_kpa =
    st->active_ki_position *
    SGC_OUTER_TS_S *
    r.error;

  r.delta_pref_kpa =
    r.p_increment_kpa +
    r.i_increment_kpa;

  r.pressure_ref_previous_kpa =
    st->pressure_ref_kpa;

  r.pressure_ref_raw_kpa =
    st->pressure_ref_kpa +
    r.delta_pref_kpa;

  bool rate_limited = false;
  bool saturated = false;

  float final_pref =
    sgc_apply_common_pref_limits(
            r.pressure_ref_raw_kpa,
            st->pressure_ref_kpa,
            &rate_limited,
            &saturated
    );

    /*
     * Reconstruye el valor intermedio solo para diagnostico.
     */
  const float max_up_step =
    SGC_PREF_RATE_UP_KPA_S * SGC_OUTER_TS_S;

  const float max_down_step =
    SGC_PREF_RATE_DOWN_KPA_S * SGC_OUTER_TS_S;

  r.pressure_ref_rate_limited_kpa =
    r.pressure_ref_raw_kpa;

  if (
    r.pressure_ref_rate_limited_kpa >
    st->pressure_ref_kpa + max_up_step)
  {
    r.pressure_ref_rate_limited_kpa =
      st->pressure_ref_kpa + max_up_step;
  } else if (  // NOLINT(readability/braces)
    r.pressure_ref_rate_limited_kpa <
    st->pressure_ref_kpa - max_down_step)
  {
    r.pressure_ref_rate_limited_kpa =
      st->pressure_ref_kpa - max_down_step;
  }

  r.rate_limited =
    rate_limited;

  r.saturated =
    saturated;

  st->pressure_ref_kpa =
    final_pref;

  r.pressure_ref_kpa =
    st->pressure_ref_kpa;

  st->previous_position_error =
    r.error;

  return r;
}

void sgc_ctrl_leso_update(
  SgcCtrlState *st,
  float pressure_input_kpa,
  float position_measured_pct
)
{
  float error =
    position_measured_pct -
    st->eso_z1_pct;

  float z1_dot =
    st->eso_z2_pct_s +
    st->active_b0 *
    pressure_input_kpa +
    SGC_ESO_BETA1 *
    error;

  float z2_dot =
    SGC_ESO_BETA2 *
    error;

  st->eso_z1_pct +=
    SGC_ESO_TS_S *
    z1_dot;

  st->eso_z2_pct_s +=
    SGC_ESO_TS_S *
    z2_dot;
}

SgcLADRCResult sgc_ctrl_ladrc_update(
  SgcCtrlState *st,
  float q_ref
)
{
  SgcLADRCResult r;
  memset(&r, 0, sizeof(r));

  r.position_error_pct =
    q_ref - st->eso_z1_pct;

  r.equivalent_pressure_kpa =
    -st->eso_z2_pct_s /
    st->active_b0;

  r.feedback_correction_kpa =
    (SGC_LADRC_OMEGA_C / st->active_b0) *
    r.position_error_pct;

  r.feedforward_step_kpa =
    st->step_ff_kpa;

  const float abs_error =
    fabsf(
            r.position_error_pct
    );

  const bool ff_direction_ok =
    (
    st->step_ff_kpa > 0.0f &&
    r.position_error_pct > 0.0f
    ) ||
    (
    st->step_ff_kpa < 0.0f &&
    r.position_error_pct < 0.0f
    );

  if (
    fabsf(st->step_ff_kpa) <=
    1e-6f)
  {
    st->ff_gamma_state = 0.0f;
  } else if (!ff_direction_ok) {
    st->ff_gamma_state = 0.0f;
  } else {
    float gamma_candidate = 0.0f;

    if (
      abs_error >=
      SGC_LADRC_FF_FULL_ERROR_PCT)
    {
      gamma_candidate = 1.0f;
    } else if (  // NOLINT(readability/braces)
      abs_error >
      SGC_LADRC_FF_ZERO_ERROR_PCT)
    {
      gamma_candidate =
        (
        abs_error -
        SGC_LADRC_FF_ZERO_ERROR_PCT
        )
        /
        (
        SGC_LADRC_FF_FULL_ERROR_PCT -
        SGC_LADRC_FF_ZERO_ERROR_PCT
        );
    }

    gamma_candidate =
      sgc_clampf(
                gamma_candidate,
                0.0f,
                1.0f
      );

        /*
         * FF monotono por referencia:
         * solo puede decrecer hasta el siguiente SET_Q.
         */
    if (
      gamma_candidate <
      st->ff_gamma_state)
    {
      st->ff_gamma_state =
        gamma_candidate;
    }
  }

  r.feedforward_gamma =
    st->ff_gamma_state;

  r.feedforward_applied_kpa =
    st->ff_gamma_state *
    st->step_ff_kpa;

  r.pressure_ref_previous_kpa =
    st->pressure_ref_kpa;

  r.pressure_ref_raw_kpa =
    r.equivalent_pressure_kpa +
    r.feedback_correction_kpa +
    r.feedforward_applied_kpa;

  bool rate_limited = false;
  bool saturated = false;

  float final_pref =
    sgc_apply_common_pref_limits(
            r.pressure_ref_raw_kpa,
            st->pressure_ref_kpa,
            &rate_limited,
            &saturated
    );

  const float max_up_step =
    SGC_PREF_RATE_UP_KPA_S * SGC_OUTER_TS_S;

  const float max_down_step =
    SGC_PREF_RATE_DOWN_KPA_S * SGC_OUTER_TS_S;

  r.pressure_ref_rate_limited_kpa =
    r.pressure_ref_raw_kpa;

  if (
    r.pressure_ref_rate_limited_kpa >
    st->pressure_ref_kpa + max_up_step)
  {
    r.pressure_ref_rate_limited_kpa =
      st->pressure_ref_kpa + max_up_step;
  } else if (  // NOLINT(readability/braces)
    r.pressure_ref_rate_limited_kpa <
    st->pressure_ref_kpa - max_down_step)
  {
    r.pressure_ref_rate_limited_kpa =
      st->pressure_ref_kpa - max_down_step;
  }

  r.rate_limited =
    rate_limited;

  r.saturated =
    saturated;

  st->pressure_ref_kpa =
    final_pref;

  r.pressure_ref_kpa =
    st->pressure_ref_kpa;

  return r;
}

SgcPressurePIResult sgc_ctrl_pressure_pi_update(
  SgcCtrlState *st,
  float reference,
  float pressure
)
{
  SgcPressurePIResult r;

  r.error =
    reference - pressure;

  r.p_term =
    SGC_KP_PRESSURE *
    r.error;

  if (
    fabsf(r.error) <=
    SGC_PRESSURE_DEADBAND_KPA)
  {
    r.i_term =
      st->pressure_integral;

    r.output_unsat = 0.0f;
    r.output = 0.0f;

    return r;
  }

  float candidate_integral =
    st->pressure_integral +
    SGC_KI_PRESSURE *
    r.error *
    SGC_PRESSURE_TS_S;

  candidate_integral =
    sgc_clampf(
            candidate_integral,
            SGC_PRESSURE_I_MIN,
            SGC_PRESSURE_I_MAX
    );

  float candidate_magnitude_unsat = 0.0f;

  if (r.error > 0.0f) {
    candidate_magnitude_unsat =
      r.p_term +
      fmaxf(
                candidate_integral,
                0.0f
      );
  } else {
    candidate_magnitude_unsat =
      -r.p_term +
      fmaxf(
                -candidate_integral,
                0.0f
      );
  }

  bool saturating_effective_command =
    candidate_magnitude_unsat >
    1.0f;

  if (!saturating_effective_command) {
    st->pressure_integral =
      candidate_integral;
  }

  r.i_term =
    st->pressure_integral;

  if (r.error > 0.0f) {
    float magnitude_unsat =
      r.p_term +
      fmaxf(
                st->pressure_integral,
                0.0f
      );

    r.output_unsat =
      magnitude_unsat;

    r.output =
      sgc_clampf(
                magnitude_unsat,
                0.0f,
                1.0f
      );
  } else {
    float magnitude_unsat =
      -r.p_term +
      fmaxf(
                -st->pressure_integral,
                0.0f
      );

    r.output_unsat =
      -magnitude_unsat;

    r.output =
      -sgc_clampf(
                magnitude_unsat,
                0.0f,
                1.0f
      );
  }

  return r;
}

void sgc_ctrl_initialize_pi_bumpless(SgcCtrlState *st, float position_ref_pct, float feedback_pct)
{
  st->previous_position_error =
    position_ref_pct -
    feedback_pct;

  memset(
        &st->last_position_pi,
        0,
        sizeof(st->last_position_pi)
  );

  st->last_position_pi.error =
    st->previous_position_error;

  st->last_position_pi.previous_error =
    st->previous_position_error;

  st->last_position_pi.pressure_ref_previous_kpa =
    st->pressure_ref_kpa;

  st->last_position_pi.pressure_ref_raw_kpa =
    st->pressure_ref_kpa;

  st->last_position_pi.pressure_ref_rate_limited_kpa =
    st->pressure_ref_kpa;

  st->last_position_pi.pressure_ref_kpa =
    st->pressure_ref_kpa;
}

void sgc_ctrl_initialize_adrc_bumpless(SgcCtrlState *st, float position_ref_pct, float feedback_pct)
{
    /*
     * Inicializacion coherente con q_dot_hat = z2 + b0*u.
     * Se supone q_dot_hat=0 al entrar al modo ADRC:
     *
     *     z2 = -b0 * P_ref
     *
     * Por tanto equivalent_pressure=-z2/b0=P_ref y el primer
     * update no fuerza un salto artificial de referencia.
     */
  st->active_b0 = st->target_b0;

  if (!isfinite(st->active_b0) || st->active_b0 <= 1e-5f) {
    st->active_b0 = SGC_B0_FALLBACK;
  }

  st->eso_z1_pct =
    feedback_pct;

  st->eso_z2_pct_s =
    -st->active_b0 *
    st->pressure_ref_kpa;

  memset(
        &st->last_ladrc,
        0,
        sizeof(st->last_ladrc)
  );

  st->last_ladrc.position_error_pct =
    position_ref_pct -
    st->eso_z1_pct;

  st->last_ladrc.equivalent_pressure_kpa =
    st->pressure_ref_kpa;

  st->last_ladrc.pressure_ref_previous_kpa =
    st->pressure_ref_kpa;

  st->last_ladrc.pressure_ref_raw_kpa =
    st->pressure_ref_kpa;

  st->last_ladrc.pressure_ref_rate_limited_kpa =
    st->pressure_ref_kpa;

  st->last_ladrc.pressure_ref_kpa =
    st->pressure_ref_kpa;
}

void sgc_ctrl_reset_outer_diagnostics(SgcCtrlState *st)
{
  memset(&st->last_position_pi, 0, sizeof(st->last_position_pi));
  memset(&st->last_ladrc, 0, sizeof(st->last_ladrc));
}
void sgc_ctrl_reset_pressure_pi(SgcCtrlState *st)
{
  st->pressure_integral = 0.0f;
  memset(&st->last_pressure_pi, 0, sizeof(st->last_pressure_pi));
}
void sgc_ctrl_latch_step_feedforward(SgcCtrlState *st, float old_q_ref, float new_q_ref)
{
  if (fabsf(new_q_ref - old_q_ref) > 0.001f) {
    st->step_ff_kpa = sgc_ctrl_compute_adrc_feedforward(old_q_ref, new_q_ref,
            st->adrc_model_k, st->schedule_input_valid);
    st->ff_gamma_state = fabsf(st->step_ff_kpa) > 1e-6f ? 1.0f : 0.0f;
  }
}
