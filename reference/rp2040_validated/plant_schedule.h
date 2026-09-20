#ifndef PLANT_SCHEDULE_H
#define PLANT_SCHEDULE_H
#include <math.h>
#include <stdbool.h>
#define PLANT_P1 0.40f
/* Rounded identification values supplied with this project. */
typedef struct { float q, k_up, tau_up, k_down, tau_down; } PlantAnchor;
static const PlantAnchor PLANT_ANCHORS[] = {
    {30.0f, 0.818f, 1.906f, 1.342f, 5.262f},
    {50.0f, 1.170f, 0.177f, 0.831f, 2.970f},
    {70.0f, 0.945f, 0.119f, 0.520f, 4.194f}
};
#define PLANT_ANCHOR_COUNT (sizeof(PLANT_ANCHORS)/sizeof(PLANT_ANCHORS[0]))
typedef struct { float q, eta, k, tau, kp, ki, b0; bool clamped; } PlantSchedule;
static inline bool plant_schedule_eval(float q, float eta, PlantSchedule *out)
{
    if (!out || !isfinite(q) || !isfinite(eta)) return false;
    float qc = fminf(fmaxf(q, PLANT_ANCHORS[0].q),
                      PLANT_ANCHORS[PLANT_ANCHOR_COUNT-1].q);
    float h = fminf(fmaxf(eta, 0.0f), 1.0f);
    unsigned i = 0;
    while (i + 2 < PLANT_ANCHOR_COUNT && qc > PLANT_ANCHORS[i+1].q) ++i;
    const PlantAnchor *a = &PLANT_ANCHORS[i], *b = &PLANT_ANCHORS[i+1];
    if (!(b->q > a->q) || !(a->k_up > 0) || !(b->k_up > 0)
        || !(a->k_down > 0) || !(b->k_down > 0)
        || !(a->tau_up > 0) || !(b->tau_up > 0)
        || !(a->tau_down > 0) || !(b->tau_down > 0)) return false;
    float w = (qc-a->q)/(b->q-a->q);
    float ku = (1-w)*logf(a->k_up) + w*logf(b->k_up);
    float kd = (1-w)*logf(a->k_down) + w*logf(b->k_down);
    float tu = (1-w)*logf(a->tau_up) + w*logf(b->tau_up);
    float td = (1-w)*logf(a->tau_down) + w*logf(b->tau_down);
    PlantSchedule r = {0};
    r.q = qc; r.eta = h; r.clamped = qc != q || h != eta;
    r.k = expf((1-h)*ku + h*kd);
    r.tau = expf((1-h)*tu + h*td);
    r.ki = PLANT_P1/r.k; r.kp = r.ki*r.tau; r.b0 = r.k/r.tau;
    if (!isfinite(r.kp) || !isfinite(r.ki) || !isfinite(r.b0)
        || !(r.kp > 0) || !(r.ki > 0) || !(r.b0 > 0)) return false;
    *out = r;
    return true;
}
#endif
