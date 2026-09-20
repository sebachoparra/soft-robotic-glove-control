// Copyright 2026 Sebastian Parra
#ifndef SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_UKF_COMPOSE_H_
#define SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_UKF_COMPOSE_H_

#include "soft_glove_core/soft_glove_core_ukf.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  SgcUkfState ukf;
  uint32_t last_update_us;
  bool bno1_ok, bno2_ok;
  SgcQuat bno1_q, bno2_q;
  float flex_aligned;
  float dt_used;
} SgcUkfCompose;

typedef struct
{
  bool ok1;
  SgcQuat q1;
  bool ok2;
  SgcQuat q2;
} SgcRefSample;

void sgc_ukf_compose_init(SgcUkfCompose *c);
void sgc_ukf_compose_step(
  SgcUkfCompose *c, float flex_raw,
  bool ok1, SgcQuat q1, bool ok2, SgcQuat q2, uint32_t now_us);
bool sgc_ukf_accumulate_reference(
  SgcUkfCompose *c, const SgcRefSample *s, uint32_t n);

#ifdef __cplusplus
}
#endif
#endif  // SOFT_GLOVE_CORE__SOFT_GLOVE_CORE_UKF_COMPOSE_H_
