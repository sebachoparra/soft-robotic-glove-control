// Copyright 2026 Sebastian Parra

#ifndef SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_CTRL_H_
#define SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_CTRL_H_
#include "soft_glove_core/soft_glove_core.h"
#include "soft_glove_core/soft_glove_core_ukf.h"

#define SGC_OUTER_LOOP_MS 500
#define SGC_B0_NOMINAL 2.20f
#define SGC_B0_FALLBACK SGC_B0_NOMINAL
#define SGC_B0_SMOOTH_TAU_S 0.50f
#define SGC_ESO_OMEGA_O 2.50f
#define SGC_ESO_BETA1 (2.0f * SGC_ESO_OMEGA_O)
#define SGC_ESO_BETA2 (SGC_ESO_OMEGA_O * SGC_ESO_OMEGA_O)
#define SGC_ESO_LOOP_MS 50
#define SGC_ESO_TS_S 0.050f
#define SGC_LADRC_OMEGA_C 0.70f
#define SGC_LADRC_FF_MODEL_WEIGHT 0.30f
#define SGC_LADRC_FF_MAX_UP_KPA 8.0f
#define SGC_LADRC_FF_MAX_DOWN_KPA 5.0f
#define SGC_LADRC_FF_FULL_ERROR_PCT 10.0f
#define SGC_LADRC_FF_ZERO_ERROR_PCT 2.0f
#define SGC_KP_PRESSURE 0.060f
#define SGC_KI_PRESSURE 0.025f
#define SGC_PRESSURE_LOOP_MS 100
#define SGC_PRESSURE_TS_S 0.100f
#define SGC_PRESSURE_I_MIN -1.0f
#define SGC_PRESSURE_I_MAX 1.0f
#define SGC_PRESSURE_DEADBAND_KPA 1.0f

typedef struct
{
  float error;
  float p_term;
  float i_term;
  float output_unsat;
  float output;
} SgcPressurePIResult;

typedef struct
{
  float error;
  float previous_error;

  float p_increment_kpa;
  float i_increment_kpa;
  float delta_pref_kpa;

  float pressure_ref_previous_kpa;
  float pressure_ref_raw_kpa;
  float pressure_ref_rate_limited_kpa;
  float pressure_ref_kpa;

  bool rate_limited;
  bool saturated;
} SgcPositionPIResult;

typedef struct
{
  float position_error_pct;

  float equivalent_pressure_kpa;
  float feedback_correction_kpa;

  float feedforward_step_kpa;
  float feedforward_gamma;
  float feedforward_applied_kpa;

  float pressure_ref_previous_kpa;
  float pressure_ref_raw_kpa;
  float pressure_ref_rate_limited_kpa;
  float pressure_ref_kpa;

  bool rate_limited;
  bool saturated;
} SgcLADRCResult;

typedef struct
{
  uint8_t active_gain_region_id;
  SgcPlantSchedule scheduled_plant;
  bool schedule_input_valid;
  float active_kp_position;
  float active_ki_position;
  float previous_position_error;
  SgcPositionPIResult last_position_pi;
  float eso_z1_pct;
  float eso_z2_pct_s;
  float active_b0;
  float target_b0;
  float adrc_model_k;
  float adrc_model_tau_s;
  float adrc_b0_identified;
  bool adrc_model_valid;
  float step_ff_kpa;
  float ff_gamma_state;
  SgcLADRCResult last_ladrc;
  float pressure_integral;
  SgcPressurePIResult last_pressure_pi;
  float pressure_ref_kpa;
} SgcCtrlState;

void sgc_ctrl_state_init(SgcCtrlState *st);
bool sgc_ctrl_ukf_feedback_available(const SgcUkfTelemetry *t);
void sgc_ctrl_update_plant_schedule(
  SgcCtrlState *st, bool ukf_available, float s_hat,
  float eta_hat);
float sgc_ctrl_compute_adrc_feedforward(
  float q_from, float q_to, float model_k,
  bool valid_nominal);
void sgc_ctrl_update_active_b0(SgcCtrlState *st);
SgcPositionPIResult sgc_ctrl_position_pi_update(SgcCtrlState *st, float q_ref, float q);
void sgc_ctrl_leso_update(SgcCtrlState *st, float pressure_input_kpa, float position_measured_pct);
SgcLADRCResult sgc_ctrl_ladrc_update(SgcCtrlState *st, float q_ref);
SgcPressurePIResult sgc_ctrl_pressure_pi_update(SgcCtrlState *st, float reference, float pressure);
void sgc_ctrl_initialize_pi_bumpless(SgcCtrlState *st, float position_ref_pct, float feedback_pct);
void sgc_ctrl_initialize_adrc_bumpless(
  SgcCtrlState *st, float position_ref_pct,
  float feedback_pct);
void sgc_ctrl_reset_outer_diagnostics(SgcCtrlState *st);
void sgc_ctrl_reset_pressure_pi(SgcCtrlState *st);
void sgc_ctrl_latch_step_feedforward(SgcCtrlState *st, float old_q_ref, float new_q_ref);
#endif  // SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_CTRL_H_
