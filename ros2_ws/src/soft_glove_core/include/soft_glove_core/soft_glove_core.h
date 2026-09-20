// Copyright 2026 Sebastian Parra

#ifndef SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_H_
#define SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================
 * CONSTANTS
 * ================================================================= */

#define SGC_PRESSURE_EWMA_ALPHA  0.20f
#define SGC_FLEX_EWMA_ALPHA      0.20f
#define SGC_FLEX_SPAN_COUNTS     494.794f
#define SGC_ADC_REF_VOLTAGE      3.3f
#define SGC_ADC_MAX_VALUE        4095.0f
#define SGC_DIVIDER_GAIN         1.545f
#define SGC_KPA_PER_VOLT         50.0f
#define SGC_PRESSURE_REF_MIN_KPA 0.0f
#define SGC_PRESSURE_REF_MAX_KPA 170.0f
#define SGC_OUTER_TS_S           0.500f
#define SGC_PREF_RATE_UP_KPA_S   10.0f
#define SGC_PREF_RATE_DOWN_KPA_S 12.0f
#define SGC_PLANT_P1             0.40f
#define SGC_UKF_MODEL_N          101
#define SGC_UKF_MODEL_S_MIN      0.0f
#define SGC_UKF_MODEL_S_MAX      100.0f


/* =================================================================
 * TYPES
 * ================================================================= */

typedef struct
{
  float q;
  float k_up;
  float tau_up;
  float k_down;
  float tau_down;
} SgcPlantAnchor;

#define SGC_PLANT_ANCHOR_COUNT 3

typedef struct
{
  float q;
  float eta;
  float k;
  float tau;
  float kp;
  float ki;
  float b0;
  bool clamped;
} SgcPlantSchedule;

/* =================================================================
 * TABLES (copied byte-for-byte from golden reference)
 * ================================================================= */

extern const SgcPlantAnchor sgc_plant_anchors[SGC_PLANT_ANCHOR_COUNT];

extern const float sgc_lut_f_up[SGC_UKF_MODEL_N];
extern const float sgc_lut_f_down[SGC_UKF_MODEL_N];
extern const float sgc_lut_theta_up_deg[SGC_UKF_MODEL_N];
extern const float sgc_lut_theta_down_deg[SGC_UKF_MODEL_N];
extern const float sgc_lut_sigma_f_up[SGC_UKF_MODEL_N];
extern const float sgc_lut_sigma_f_down[SGC_UKF_MODEL_N];
extern const float sgc_lut_sigma_theta_up_deg[SGC_UKF_MODEL_N];
extern const float sgc_lut_sigma_theta_down_deg[SGC_UKF_MODEL_N];

#define SGC_LUT_F_UP                    sgc_lut_f_up
#define SGC_LUT_F_DOWN                  sgc_lut_f_down
#define SGC_LUT_THETA_UP_DEG            sgc_lut_theta_up_deg
#define SGC_LUT_THETA_DOWN_DEG          sgc_lut_theta_down_deg
#define SGC_LUT_SIGMA_F_UP              sgc_lut_sigma_f_up
#define SGC_LUT_SIGMA_F_DOWN            sgc_lut_sigma_f_down
#define SGC_LUT_SIGMA_THETA_UP_DEG      sgc_lut_sigma_theta_up_deg
#define SGC_LUT_SIGMA_THETA_DOWN_DEG    sgc_lut_sigma_theta_down_deg

/* =================================================================
 * FUNCTIONS
 * ================================================================= */

/* Common utility: clampf (main.c:419-428) */
float sgc_clampf(float x, float xmin, float xmax);

/* Block 1: LUT evaluation (ukf_shadow_rp2040.h:393-408) */
float sgc_lut(const float *table, float s);

/* Block 2: plant schedule evaluation (plant_schedule.h:15-42) */
bool sgc_plant_schedule_eval(float q, float eta, SgcPlantSchedule *out);

/* Block 3: conversions (main.c:563-585) */
float sgc_pressure_raw_to_vout(float raw);
float sgc_pressure_raw_to_kpa(float raw, float pressure_zero_vout);
float sgc_flex_raw_to_position_unclipped(float raw, float flex_zero_raw);

/* Block 4: EWMA (main.c:587-596) */
float sgc_ewma(float x, float previous, float alpha);

/* Block 5: magnitude to pulse (main.c:1524-1546) */
uint32_t sgc_magnitude_to_pulse(float magnitude, uint32_t min_ms, uint32_t max_ms);

/* Block 6: apply common pref limits (main.c:1047-1101) */
float sgc_apply_common_pref_limits(
  float raw_pref,
  float previous_pref,
  bool *rate_limited,
  bool *saturated
);

/* Scaffold probe preserved for test_scaffold_link compatibility */
int soft_glove_core_scaffold_probe(void);

#ifdef __cplusplus
}
#endif

#endif  // SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_H_
