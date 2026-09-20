// Copyright 2026 Sebastian Parra
#include "soft_glove_core/soft_glove_core_ukf.h"
#include <math.h>
const float sgc_ukf_n_axis[3] = {-0.14794015f, -0.98835782f, -0.03553205f};
float sgc_ukf_weight_m(int j)
{
  return (j == 0) ? SGC_UKFS_WM0 : SGC_UKFS_WI;
}

float sgc_ukf_weight_c(int j)
{
  return (j == 0) ? SGC_UKFS_WC0 : SGC_UKFS_WI;
}

void sgc_ukf_zero4(float A[SGC_UKFS_NX][SGC_UKFS_NX])
{
  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int j = 0; j < SGC_UKFS_NX; j++) {
      A[i][j] = 0.0f;
    }
  }
}

void sgc_ukf_identity4(float A[SGC_UKFS_NX][SGC_UKFS_NX])
{
  sgc_ukf_zero4(A);
  for (int i = 0; i < SGC_UKFS_NX; i++) {
    A[i][i] = 1.0f;
  }
}

bool sgc_ukf_cholesky4(
  const float A[SGC_UKFS_NX][SGC_UKFS_NX],
  float L[SGC_UKFS_NX][SGC_UKFS_NX]
)
{
  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int j = 0; j < SGC_UKFS_NX; j++) {
      L[i][j] = 0.0f;
    }
  }

  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int j = 0; j <= i; j++) {
      float sum = A[i][j];

      for (int k = 0; k < j; k++) {
        sum -= L[i][k] * L[j][k];
      }

      if (i == j) {
        if (!isfinite(sum) || sum <= SGC_UKFS_P_FLOOR) {
          return false;
        }

        L[i][j] = sqrtf(sum);
      } else {
        if (!isfinite(L[j][j]) || fabsf(L[j][j]) < SGC_UKFS_P_FLOOR) {
          return false;
        }

        L[i][j] = sum / L[j][j];
      }
    }
  }

  return true;
}

void sgc_ukf_repair_cov4(float P[SGC_UKFS_NX][SGC_UKFS_NX])
{
    /* Symmetrize and enforce diagonal floors. */
  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int j = i + 1; j < SGC_UKFS_NX; j++) {
      float a = 0.5f * (P[i][j] + P[j][i]);

      if (!isfinite(a)) {
        a = 0.0f;
      }

      P[i][j] = a;
      P[j][i] = a;
    }

    if (!isfinite(P[i][i]) || P[i][i] < SGC_UKFS_P_FLOOR) {
      P[i][i] = SGC_UKFS_P_FLOOR;
    }
  }

    /* Limit pairwise covariance magnitude. */
  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int j = i + 1; j < SGC_UKFS_NX; j++) {
      float lim =
        0.995f *
        sqrtf(fmaxf(P[i][i] * P[j][j], SGC_UKFS_P_FLOOR));

      P[i][j] = sgc_clampf(P[i][j], -lim, lim);
      P[j][i] = P[i][j];
    }
  }

    /*
     * Ensure Cholesky success by diagonal loading.
     * With only 4 states this is cheap and deterministic.
     */
  float L[SGC_UKFS_NX][SGC_UKFS_NX];

  if (sgc_ukf_cholesky4(P, L)) {
    return;
  }

  float jitter = 1.0e-6f;

  for (int attempt = 0; attempt < 8; attempt++) {
    for (int i = 0; i < SGC_UKFS_NX; i++) {
      P[i][i] += jitter;
    }

    if (sgc_ukf_cholesky4(P, L)) {
      return;
    }

    jitter *= 10.0f;
  }

    /* Last-resort diagonal covariance. */
  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int j = 0; j < SGC_UKFS_NX; j++) {
      if (i != j) {
        P[i][j] = 0.0f;
      }
    }

    if (P[i][i] < 1.0e-3f) {
      P[i][i] = 1.0e-3f;
    }
  }
}

void sgc_ukf_measurement_model(
  const float x[SGC_UKFS_NX],
  float *flex_pred,
  float *theta_pred
)
{
  float s = sgc_clampf(x[0], SGC_UKFS_S_MIN, SGC_UKFS_S_MAX);
  float eta = sgc_clampf(x[2], SGC_UKFS_ETA_MIN, SGC_UKFS_ETA_MAX);

  float fup = sgc_lut(sgc_lut_f_up, s);
  float fdn = sgc_lut(sgc_lut_f_down, s);

  float tup = sgc_lut(sgc_lut_theta_up_deg, s);
  float tdn = sgc_lut(sgc_lut_theta_down_deg, s);

  *flex_pred =
    (1.0f - eta) * fup +
    eta * fdn +
    x[3];

  *theta_pred =
    (1.0f - eta) * tup +
    eta * tdn;
}

void sgc_ukf_measurement_sigma(
  const float x[SGC_UKFS_NX],
  float *sigma_f,
  float *sigma_t
)
{
  float s = sgc_clampf(x[0], SGC_UKFS_S_MIN, SGC_UKFS_S_MAX);
  float eta = sgc_clampf(x[2], SGC_UKFS_ETA_MIN, SGC_UKFS_ETA_MAX);

  float sfup = fmaxf(sgc_lut(sgc_lut_sigma_f_up, s), SGC_UKFS_SIGMA_F_MIN);
  float sfdn = fmaxf(sgc_lut(sgc_lut_sigma_f_down, s), SGC_UKFS_SIGMA_F_MIN);

  float stup = fmaxf(sgc_lut(sgc_lut_sigma_theta_up_deg, s), SGC_UKFS_SIGMA_T_MIN);
  float stdn = fmaxf(sgc_lut(sgc_lut_sigma_theta_down_deg, s), SGC_UKFS_SIGMA_T_MIN);

  float var_f =
    (1.0f - eta) * sfup * sfup +
    eta * sfdn * sfdn;

  float var_t =
    (1.0f - eta) * stup * stup +
    eta * stdn * stdn;

  var_f = fmaxf(var_f, SGC_UKFS_SIGMA_F_MIN * SGC_UKFS_SIGMA_F_MIN);
  var_t = fmaxf(var_t, SGC_UKFS_SIGMA_T_MIN * SGC_UKFS_SIGMA_T_MIN);

    /* Rscale multiplies variance, exactly as in the MATLAB sweep. */
  var_f *= SGC_UKFS_R_SCALE;
  var_t *= SGC_UKFS_R_SCALE;

  *sigma_f = sqrtf(fmaxf(var_f, SGC_UKFS_S_FLOOR));
  *sigma_t = sqrtf(fmaxf(var_t, SGC_UKFS_S_FLOOR));
}

SgcQuat sgc_quat_normalize(SgcQuat q)
{
  float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

  if (!isfinite(n2) || n2 < 1.0e-12f) {
    return (SgcQuat) {1.0f, 0.0f, 0.0f, 0.0f};  // NOLINT(readability/braces)
  }

  float invn = 1.0f / sqrtf(n2);

  q.w *= invn;
  q.x *= invn;
  q.y *= invn;
  q.z *= invn;

  return q;
}

SgcQuat sgc_quat_conj(SgcQuat q)
{
  return (SgcQuat) {q.w, -q.x, -q.y, -q.z};  // NOLINT(readability/braces)
}

SgcQuat sgc_quat_mul(SgcQuat a, SgcQuat b)
{
  SgcQuat q;

  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return q;
}

float sgc_quat_dot(SgcQuat a, SgcQuat b)
{
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

SgcQuat sgc_quat_neg(SgcQuat q)
{
  q.w = -q.w;
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  return q;
}

SgcQuat sgc_quat_relative(SgcQuat q1, SgcQuat q2)
{
  q1 = sgc_quat_normalize(q1);
  q2 = sgc_quat_normalize(q2);

  return sgc_quat_normalize(
        sgc_quat_mul(
            sgc_quat_conj(q1),
            q2
        )
  );
}

void sgc_ukf_constrain_state(float x[SGC_UKFS_NX])
{
  x[0] = sgc_clampf(x[0], SGC_UKFS_S_MIN, SGC_UKFS_S_MAX);
  x[1] = sgc_clampf(x[1], SGC_UKFS_V_MIN, SGC_UKFS_V_MAX);
  x[2] = sgc_clampf(x[2], SGC_UKFS_ETA_MIN, SGC_UKFS_ETA_MAX);
  x[3] = sgc_clampf(x[3], SGC_UKFS_BIAS_MIN, SGC_UKFS_BIAS_MAX);

  if (x[0] <= SGC_UKFS_S_MIN && x[1] < 0.0f) {
    x[1] = 0.0f;
  }

  if (x[0] >= SGC_UKFS_S_MAX && x[1] > 0.0f) {
    x[1] = 0.0f;
  }
}

void sgc_ukf_process_model(
  const float xin[SGC_UKFS_NX],
  float dt,
  float xout[SGC_UKFS_NX]
)
{
  float s = xin[0];
  float v = xin[1];
  float eta = xin[2];
  float b = xin[3];

  xout[0] = s + dt * v;
  xout[1] = v;

  float zeta = 2.0f * eta - 1.0f;
  float sN = (s - 50.0f) / 50.0f;

  float dzdt =
    SGC_UKFS_ETA_A1 * v +
    SGC_UKFS_ETA_A2 * fabsf(v) * zeta +
    SGC_UKFS_ETA_A3 * v * fabsf(zeta) +
    SGC_UKFS_ETA_A4 * zeta +
    SGC_UKFS_ETA_A5 * sN +
    SGC_UKFS_ETA_A6;

  float zeta2 = zeta + dt * dzdt;
  zeta2 = sgc_clampf(zeta2, -1.0f, 1.0f);

  xout[2] = 0.5f * (zeta2 + 1.0f);

    /* Deterministic part: bias remains constant. */
  xout[3] = b;

  sgc_ukf_constrain_state(xout);
}

void sgc_ukf_process_noise(float dt, float Q[SGC_UKFS_NX][SGC_UKFS_NX])
{
  sgc_ukf_zero4(Q);

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;

  float qAcc = SGC_UKFS_SIGMA_ACC * SGC_UKFS_SIGMA_ACC;

  Q[0][0] = qAcc * dt4 / 4.0f;
  Q[0][1] = qAcc * dt3 / 2.0f;
  Q[1][0] = Q[0][1];
  Q[1][1] = qAcc * dt2;

  Q[2][2] =
    SGC_UKFS_SIGMA_ETA_RW *
    SGC_UKFS_SIGMA_ETA_RW *
    dt;

  Q[3][3] =
    SGC_UKFS_SIGMA_BIAS_RW *
    SGC_UKFS_SIGMA_BIAS_RW *
    dt;
}

bool sgc_ukf_update_theta(
  SgcUkfState *st,
  SgcQuat q1,
  SgcQuat q2,
  float *theta_deg
)
{
  if (!st->reference_valid) {
    return false;
  }

  SgcQuat qrel = sgc_quat_relative(q1, q2);

  SgcQuat qdelta =
    sgc_quat_mul(
            sgc_quat_conj(st->qrel0),
            qrel
    );

  qdelta = sgc_quat_normalize(qdelta);

  if (sgc_quat_dot(qdelta, st->qdelta_prev) < 0.0f) {
    qdelta = sgc_quat_neg(qdelta);
  }

  st->qdelta_prev = qdelta;

  float q_parallel =
    qdelta.x * sgc_ukf_n_axis[0] +
    qdelta.y * sgc_ukf_n_axis[1] +
    qdelta.z * sgc_ukf_n_axis[2];

  float half_now = atan2f(q_parallel, qdelta.w);

  float dhalf = half_now - st->half_prev;

  if (dhalf > SGC_UKFS_PI) {
    dhalf -= 2.0f * SGC_UKFS_PI;
  } else if (dhalf < -SGC_UKFS_PI) {
    dhalf += 2.0f * SGC_UKFS_PI;
  }

  st->half_accum += dhalf;
  st->half_prev = half_now;

  *theta_deg =
    2.0f *
    st->half_accum *
    180.0f /
    SGC_UKFS_PI;

  return isfinite(*theta_deg);
}

void sgc_ukf_generate_sigma_points(
  const float x[SGC_UKFS_NX],
  float P[SGC_UKFS_NX][SGC_UKFS_NX],
  float X[SGC_UKFS_NSIGMA][SGC_UKFS_NX]
)
{
  sgc_ukf_repair_cov4(P);

  float A[SGC_UKFS_NX][SGC_UKFS_NX];
  float L[SGC_UKFS_NX][SGC_UKFS_NX];

  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int j = 0; j < SGC_UKFS_NX; j++) {
      A[i][j] = SGC_UKFS_C * P[i][j];
    }
  }

  if (!sgc_ukf_cholesky4(A, L)) {
    for (int i = 0; i < SGC_UKFS_NX; i++) {
      A[i][i] += 1.0e-5f;
    }

    if (!sgc_ukf_cholesky4(A, L)) {
      for (int i = 0; i < SGC_UKFS_NX; i++) {
        for (int j = 0; j < SGC_UKFS_NX; j++) {
          L[i][j] = 0.0f;
        }

        L[i][i] = sqrtf(fmaxf(A[i][i], SGC_UKFS_P_FLOOR));
      }
    }
  }

  for (int k = 0; k < SGC_UKFS_NX; k++) {
    X[0][k] = x[k];
  }

  for (int c = 0; c < SGC_UKFS_NX; c++) {
    int jp = 1 + c;
    int jm = 1 + SGC_UKFS_NX + c;

    for (int r = 0; r < SGC_UKFS_NX; r++) {
      X[jp][r] = x[r] + L[r][c];
      X[jm][r] = x[r] - L[r][c];
    }

    sgc_ukf_constrain_state(X[jp]);
    sgc_ukf_constrain_state(X[jm]);
  }
}

void sgc_ukf_prediction_only(
  SgcUkfState *st,
  const float XP[SGC_UKFS_NSIGMA][SGC_UKFS_NX],
  const float xpred[SGC_UKFS_NX],
  float Ppred[SGC_UKFS_NX][SGC_UKFS_NX]
)
{
  for (int i = 0; i < SGC_UKFS_NX; i++) {
    st->x[i] = xpred[i];
  }

  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int j = 0; j < SGC_UKFS_NX; j++) {
      st->P[i][j] = Ppred[i][j];
    }
  }

  sgc_ukf_repair_cov4(st->P);

  st->telem.innovation_flex = NAN;
  st->telem.innovation_theta = NAN;
  st->telem.nis = NAN;
  st->telem.valid = false;

  (void)XP;
}

void sgc_ukf_update_filter(
  SgcUkfState *st,
  float flex_meas,
  bool theta_valid,
  float theta_meas,
  float dt
)
{
  float X[SGC_UKFS_NSIGMA][SGC_UKFS_NX];
  float XP[SGC_UKFS_NSIGMA][SGC_UKFS_NX];

  sgc_ukf_generate_sigma_points(st->x, st->P, X);

  for (int j = 0; j < SGC_UKFS_NSIGMA; j++) {
    sgc_ukf_process_model(X[j], dt, XP[j]);
  }

  float xpred[SGC_UKFS_NX] = {0};

  for (int j = 0; j < SGC_UKFS_NSIGMA; j++) {
    float w = sgc_ukf_weight_m(j);

    for (int i = 0; i < SGC_UKFS_NX; i++) {
      xpred[i] += w * XP[j][i];
    }
  }

  sgc_ukf_constrain_state(xpred);

  float Q[SGC_UKFS_NX][SGC_UKFS_NX];
  float Ppred[SGC_UKFS_NX][SGC_UKFS_NX];

  sgc_ukf_process_noise(dt, Q);

  for (int i = 0; i < SGC_UKFS_NX; i++) {
    for (int k = 0; k < SGC_UKFS_NX; k++) {
      Ppred[i][k] = Q[i][k];
    }
  }

  for (int j = 0; j < SGC_UKFS_NSIGMA; j++) {
    float w = sgc_ukf_weight_c(j);

    float dx[SGC_UKFS_NX];

    for (int i = 0; i < SGC_UKFS_NX; i++) {
      dx[i] = XP[j][i] - xpred[i];
    }

    for (int r = 0; r < SGC_UKFS_NX; r++) {
      for (int c = 0; c < SGC_UKFS_NX; c++) {
        Ppred[r][c] += w * dx[r] * dx[c];
      }
    }
  }

  sgc_ukf_repair_cov4(Ppred);

    /*
     * For the 4-state model a FLEX-only update is intentionally not used.
     * Without theta, s/eta/bF are too strongly confounded.
     */
  if (!theta_valid || !isfinite(theta_meas) || !isfinite(flex_meas)) {
    sgc_ukf_prediction_only(st, XP, xpred, Ppred);
    return;
  }

  float Z[SGC_UKFS_NSIGMA][SGC_UKFS_NZ];

  for (int j = 0; j < SGC_UKFS_NSIGMA; j++) {
    sgc_ukf_measurement_model(
            XP[j],
            &Z[j][0],
            &Z[j][1]
    );
  }

  float zpred[SGC_UKFS_NZ] = {0};

  for (int j = 0; j < SGC_UKFS_NSIGMA; j++) {
    float w = sgc_ukf_weight_m(j);

    zpred[0] += w * Z[j][0];
    zpred[1] += w * Z[j][1];
  }

  float sigma_f;
  float sigma_t;

  sgc_ukf_measurement_sigma(
        xpred,
        &sigma_f,
        &sigma_t
  );

  st->telem.sigma_flex_model = sigma_f;
  st->telem.sigma_theta_model = sigma_t;

  float S00 = sigma_f * sigma_f;
  float S01 = 0.0f;
  float S11 = sigma_t * sigma_t;

  float Pxz[SGC_UKFS_NX][SGC_UKFS_NZ] = {{0}};

  for (int j = 0; j < SGC_UKFS_NSIGMA; j++) {
    float w = sgc_ukf_weight_c(j);

    float dz0 = Z[j][0] - zpred[0];
    float dz1 = Z[j][1] - zpred[1];

    S00 += w * dz0 * dz0;
    S01 += w * dz0 * dz1;
    S11 += w * dz1 * dz1;

    for (int i = 0; i < SGC_UKFS_NX; i++) {
      float dx = XP[j][i] - xpred[i];

      Pxz[i][0] += w * dx * dz0;
      Pxz[i][1] += w * dx * dz1;
    }
  }

  float det = S00 * S11 - S01 * S01;

  if (!isfinite(det) || det < SGC_UKFS_S_FLOOR) {
    sgc_ukf_prediction_only(st, XP, xpred, Ppred);
    return;
  }

  float invS00 = S11 / det;
  float invS01 = -S01 / det;
  float invS11 = S00 / det;

  float K[SGC_UKFS_NX][SGC_UKFS_NZ];

  for (int i = 0; i < SGC_UKFS_NX; i++) {
    K[i][0] =
      Pxz[i][0] * invS00 +
      Pxz[i][1] * invS01;

    K[i][1] =
      Pxz[i][0] * invS01 +
      Pxz[i][1] * invS11;
  }

  float innov0 = flex_meas - zpred[0];
  float innov1 = theta_meas - zpred[1];

  for (int i = 0; i < SGC_UKFS_NX; i++) {
    st->x[i] =
      xpred[i] +
      K[i][0] * innov0 +
      K[i][1] * innov1;
  }

  sgc_ukf_constrain_state(st->x);

    /*
     * Joseph-style full form is unnecessary here because S already
     * contains the predicted innovation covariance; use P = P- KSK'.
     */
  for (int r = 0; r < SGC_UKFS_NX; r++) {
    for (int c = 0; c < SGC_UKFS_NX; c++) {
      float ksk =
        K[r][0] * (S00 * K[c][0] + S01 * K[c][1]) +
        K[r][1] * (S01 * K[c][0] + S11 * K[c][1]);

      st->P[r][c] = Ppred[r][c] - ksk;
    }
  }

  sgc_ukf_repair_cov4(st->P);

  float nis =
    innov0 * (invS00 * innov0 + invS01 * innov1) +
    innov1 * (invS01 * innov0 + invS11 * innov1);

  st->telem.innovation_flex = innov0;
  st->telem.innovation_theta = innov1;
  st->telem.nis = nis;
  st->telem.valid = true;
}

void sgc_ukf_reset(SgcUkfState *st)
{
  st->x[0] = 0.0f;
  st->x[1] = 0.0f;
  st->x[2] = 0.0f;
  st->x[3] = 0.0f;

  sgc_ukf_zero4(st->P);

  st->P[0][0] = SGC_UKFS_P0_S;
  st->P[1][1] = SGC_UKFS_P0_V;
  st->P[2][2] = SGC_UKFS_P0_ETA;
  st->P[3][3] = SGC_UKFS_P0_BIAS;

  st->qdelta_prev = (SgcQuat) {1.0f, 0.0f, 0.0f, 0.0f};

  st->half_prev = 0.0f;
  st->half_accum = 0.0f;


  st->telem.valid = false;
  st->telem.theta_valid = false;

  st->telem.s_hat = 0.0f;
  st->telem.v_hat = 0.0f;
  st->telem.eta_hat = 0.0f;
  st->telem.b_f_hat = 0.0f;

  st->telem.theta_deg = 0.0f;

  st->telem.innovation_flex = NAN;
  st->telem.innovation_theta = NAN;

  st->telem.sigma_s = sqrtf(SGC_UKFS_P0_S);
  st->telem.sigma_v = sqrtf(SGC_UKFS_P0_V);
  st->telem.sigma_eta = sqrtf(SGC_UKFS_P0_ETA);
  st->telem.sigma_b_f = sqrtf(SGC_UKFS_P0_BIAS);

  st->telem.nis = NAN;
  st->telem.rho_eta_b_f = 0.0f;
}
void sgc_ukf_set_reference(SgcUkfState *st, SgcQuat qrel0)
{
  st->qrel0 = sgc_quat_normalize(qrel0);
  st->reference_valid = true;
  sgc_ukf_reset(st);
  st->telem.reference_valid = true;
}
