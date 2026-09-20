// Copyright 2026 Sebastian Parra
#ifndef SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_UKF_H_
#define SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_UKF_H_
#include "soft_glove_core/soft_glove_core.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
#define SGC_UKFS_NX 4
#define SGC_UKFS_NZ 2
#define SGC_UKFS_NSIGMA (2 * SGC_UKFS_NX + 1)

/*
 * nx=4, alpha=0.2, beta=2, kappa=0
 *
 * lambda = alpha^2*(nx+kappa)-nx = -3.84
 * c      = nx+lambda             = 0.16
 *
 * Wm0 = -24
 * Wc0 = -21.04
 * Wi  =  3.125
 */
#define SGC_UKFS_C               0.16f
#define SGC_UKFS_WM0           -24.0f
#define SGC_UKFS_WC0           -21.04f
#define SGC_UKFS_WI              3.125f

#define SGC_UKFS_SIGMA_ACC       50.0f
#define SGC_UKFS_SIGMA_ETA_RW     0.06f
#define SGC_UKFS_SIGMA_BIAS_RW    0.005f
#define SGC_UKFS_R_SCALE          0.02f

/* Generalized eta dynamics identified offline. */
#define SGC_UKFS_ETA_A1          -0.03967525f
#define SGC_UKFS_ETA_A2          -0.01119239f
#define SGC_UKFS_ETA_A3           0.02141572f
#define SGC_UKFS_ETA_A4          -0.01612887f
#define SGC_UKFS_ETA_A5          -0.10246220f
#define SGC_UKFS_ETA_A6          -0.01530813f

#define SGC_UKFS_PI               3.14159265358979323846f

#define SGC_UKFS_S_MIN            0.0f
#define SGC_UKFS_S_MAX          100.0f
#define SGC_UKFS_V_MIN         -250.0f
#define SGC_UKFS_V_MAX          250.0f
#define SGC_UKFS_ETA_MIN          0.0f
#define SGC_UKFS_ETA_MAX          1.0f
#define SGC_UKFS_BIAS_MIN       -80.0f
#define SGC_UKFS_BIAS_MAX        80.0f

#define SGC_UKFS_SIGMA_F_MIN      5.0f
#define SGC_UKFS_SIGMA_T_MIN      1.0f

#define SGC_UKFS_P_FLOOR          1.0e-7f
#define SGC_UKFS_S_FLOOR          1.0e-8f

/* Initial covariance from offline validation. */
#define SGC_UKFS_P0_S             16.0f
#define SGC_UKFS_P0_V            225.0f
#define SGC_UKFS_P0_ETA            0.0625f
#define SGC_UKFS_P0_BIAS         225.0f

/* ============================ TYPES ========================= */

typedef struct { float w, x, y, z; } SgcQuat;
typedef struct
{
  bool valid;
  bool theta_valid;
  bool reference_valid;
  float theta_deg;
  float s_hat, v_hat, eta_hat, b_f_hat;
  float innovation_flex, innovation_theta;
  float sigma_flex_model, sigma_theta_model;
  float sigma_s, sigma_v, sigma_eta, sigma_b_f;
  float nis, rho_eta_b_f;
} SgcUkfTelemetry;
typedef struct
{
  float x[SGC_UKFS_NX];
  float P[SGC_UKFS_NX][SGC_UKFS_NX];
  SgcQuat qrel0;
  SgcQuat qdelta_prev;
  float half_prev;
  float half_accum;
  bool reference_valid;
  SgcUkfTelemetry telem;
} SgcUkfState;
extern const float sgc_ukf_n_axis[3];
float sgc_ukf_weight_m(int j);
float sgc_ukf_weight_c(int j);
void sgc_ukf_zero4(float A[SGC_UKFS_NX][SGC_UKFS_NX]);
void sgc_ukf_identity4(float A[SGC_UKFS_NX][SGC_UKFS_NX]);
bool sgc_ukf_cholesky4(
  const float A[SGC_UKFS_NX][SGC_UKFS_NX],
  float L[SGC_UKFS_NX][SGC_UKFS_NX]
);
void sgc_ukf_repair_cov4(float P[SGC_UKFS_NX][SGC_UKFS_NX]);
void sgc_ukf_measurement_model(
  const float x[SGC_UKFS_NX],
  float *flex_pred,
  float *theta_pred
);
void sgc_ukf_measurement_sigma(
  const float x[SGC_UKFS_NX],
  float *sigma_f,
  float *sigma_t
);
SgcQuat sgc_quat_normalize(SgcQuat q);
SgcQuat sgc_quat_conj(SgcQuat q);
SgcQuat sgc_quat_mul(SgcQuat a, SgcQuat b);
float sgc_quat_dot(SgcQuat a, SgcQuat b);
SgcQuat sgc_quat_neg(SgcQuat q);
SgcQuat sgc_quat_relative(SgcQuat q1, SgcQuat q2);
void sgc_ukf_constrain_state(float x[SGC_UKFS_NX]);
void sgc_ukf_process_model(
  const float xin[SGC_UKFS_NX],
  float dt,
  float xout[SGC_UKFS_NX]
);
void sgc_ukf_process_noise(float dt, float Q[SGC_UKFS_NX][SGC_UKFS_NX]);
bool sgc_ukf_update_theta(
  SgcUkfState *st,
  SgcQuat q1,
  SgcQuat q2,
  float *theta_deg
);
void sgc_ukf_generate_sigma_points(
  const float x[SGC_UKFS_NX],
  float P[SGC_UKFS_NX][SGC_UKFS_NX],
  float X[SGC_UKFS_NSIGMA][SGC_UKFS_NX]
);
void sgc_ukf_prediction_only(
  SgcUkfState *st,
  const float XP[SGC_UKFS_NSIGMA][SGC_UKFS_NX],
  const float xpred[SGC_UKFS_NX],
  float Ppred[SGC_UKFS_NX][SGC_UKFS_NX]
);
void sgc_ukf_update_filter(
  SgcUkfState *st,
  float flex_meas,
  bool theta_valid,
  float theta_meas,
  float dt
);
void sgc_ukf_reset(SgcUkfState *st);
void sgc_ukf_set_reference(SgcUkfState *st, SgcQuat qrel0);
#ifdef __cplusplus
}
#endif
#endif  // SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_UKF_H_
