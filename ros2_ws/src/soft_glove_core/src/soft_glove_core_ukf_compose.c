// Copyright 2026 Sebastian Parra
#include "soft_glove_core/soft_glove_core_ukf_compose.h"

#include <math.h>
#include <string.h>

void sgc_ukf_compose_init(SgcUkfCompose *c)
{
  memset(c, 0, sizeof(*c));
}

void sgc_ukf_compose_step(
  SgcUkfCompose *c, float flex_raw,
  bool ok1, SgcQuat q1, bool ok2, SgcQuat q2, uint32_t now_us)
{
  c->bno1_ok = ok1;
  c->bno2_ok = ok2;

  if (ok1) {
    c->bno1_q = q1;
  }

  if (ok2) {
    c->bno2_q = q2;
  }

  float theta = NAN;
  bool theta_ok = false;

  if (ok1 && ok2) {
    theta_ok = sgc_ukf_update_theta(&c->ukf, q1, q2, &theta);
  }

  c->ukf.telem.theta_valid = theta_ok;

  if (theta_ok) {
    c->ukf.telem.theta_deg = theta;
  }

  c->flex_aligned = flex_raw;

  float dt =
    (c->last_update_us == 0u) ?
    ((float)50 / 1000.0f) :
    ((float)(now_us - c->last_update_us) * 1.0e-6f);

  c->last_update_us = now_us;

  if (!isfinite(dt) || dt <= 0.0f || dt > 0.20f) {
    dt = (float)50 / 1000.0f;
  }

  c->dt_used = dt;

  if (c->ukf.reference_valid) {
    sgc_ukf_update_filter(&c->ukf, flex_raw, theta_ok, theta, dt);
  } else {
    c->ukf.telem.valid = false;
  }

  c->ukf.telem.reference_valid = c->ukf.reference_valid;

  c->ukf.telem.s_hat = c->ukf.x[0];
  c->ukf.telem.v_hat = c->ukf.x[1];
  c->ukf.telem.eta_hat = c->ukf.x[2];
  c->ukf.telem.b_f_hat = c->ukf.x[3];

  c->ukf.telem.sigma_s = sqrtf(fmaxf(c->ukf.P[0][0], 0.0f));
  c->ukf.telem.sigma_v = sqrtf(fmaxf(c->ukf.P[1][1], 0.0f));
  c->ukf.telem.sigma_eta = sqrtf(fmaxf(c->ukf.P[2][2], 0.0f));
  c->ukf.telem.sigma_b_f = sqrtf(fmaxf(c->ukf.P[3][3], 0.0f));

  float denom =
    sqrtf(
    fmaxf(
      c->ukf.P[2][2] * c->ukf.P[3][3],
      SGC_UKFS_P_FLOOR
    )
    );

  c->ukf.telem.rho_eta_b_f =
    sgc_clampf(
    c->ukf.P[2][3] / denom,
    -1.0f,
    1.0f
    );
}

bool sgc_ukf_accumulate_reference(
  SgcUkfCompose *c, const SgcRefSample *s, uint32_t n)
{
  SgcQuat sum = {0.0f, 0.0f, 0.0f, 0.0f};
  SgcQuat qref = {1.0f, 0.0f, 0.0f, 0.0f};

  bool have_ref = false;
  uint32_t good = 0u;

  for (uint32_t i = 0u; i < n; i++) {
    if (s[i].ok1 && s[i].ok2) {
      SgcQuat qr = sgc_quat_relative(s[i].q1, s[i].q2);

      if (!have_ref) {
        qref = qr;
        have_ref = true;
      }

      if (sgc_quat_dot(qr, qref) < 0.0f) {
        qr = sgc_quat_neg(qr);
      }

      sum.w += qr.w;
      sum.x += qr.x;
      sum.y += qr.y;
      sum.z += qr.z;

      good++;
    }
  }

  if (good < n / 2u) {
    return false;
  }

  sgc_ukf_set_reference(&c->ukf, sum);
  return true;
}
