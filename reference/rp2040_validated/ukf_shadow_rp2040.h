#ifndef UKF_SHADOW_RP2040_H
#define UKF_SHADOW_RP2040_H

/*
 * UKF4_SHADOW_RP2040
 * ================================================================
 * Final offline-tuned 4-state UKF candidate for RP2040 shadow-mode
 * deployment.
 *
 * State:
 *     x = [s, sdot, eta, bF]^T
 *
 * Measurements:
 *     z = [FLEX_raw, theta_signed]^T
 *
 * Measurement model:
 *     F = (1-eta) F_UP(s) + eta F_DOWN(s) + bF
 *     T = (1-eta) T_UP(s) + eta T_DOWN(s)
 *
 * Process model:
 *     s(k+1) = s + dt*sdot
 *     sdot(k+1) = sdot
 *
 *     zeta = 2*eta - 1
 *     dzeta/dt =
 *          a1*sdot
 *        + a2*abs(sdot)*zeta
 *        + a3*sdot*abs(zeta)
 *        + a4*zeta
 *        + a5*(s-50)/50
 *        + a6
 *
 *     bF(k+1) = bF(k) + w_b
 *
 * Offline-selected tuning:
 *     alpha = 0.2
 *     beta  = 2
 *     kappa = 0
 *     sigma_acc      = 50
 *     sigma_eta_RW   = 0.06
 *     sigma_bF_RW    = 0.005 counts/sqrt(s)
 *     R_scale        = 0.02  (multiplies measurement variances)
 *
 * IMPORTANT:
 *   - No pressure, q_ref, PI, ADRC, Direction, Step, or quasi-static
 *     metadata enters the estimator.
 *   - FLEX is NOT re-zeroed per run. The state bF estimates the slow
 *     sensor/session offset.
 *   - For this first embedded deployment the UKF remains SHADOW ONLY.
 *   - If theta becomes unavailable, the filter performs prediction only
 *     instead of a FLEX-only update, because FLEX alone cannot reliably
 *     separate s, eta and bF.
 * ================================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"

#include "ukf_model_lut.h"

/* ========================== TIMING ========================== */

#define UKF_SHADOW_LOOP_MS              50u
#define UKF_SHADOW_REF_SAMPLES          40u
#define UKF_SHADOW_REF_SAMPLE_DELAY_MS  20u

/* ============================ I2C =========================== */

#define UKFS_I2C_PORT   i2c0
#define UKFS_SDA_PIN    4
#define UKFS_SCL_PIN    5

#define UKFS_BNO1_ADDR  0x28
#define UKFS_BNO2_ADDR  0x29

/* ========================== BNO055 ========================== */

#define UKFS_BNO_CHIP_ID_REG         0x00
#define UKFS_BNO_CHIP_ID_VALUE       0xA0
#define UKFS_BNO_PAGE_ID_REG         0x07
#define UKFS_BNO_QUA_DATA_W_LSB_REG  0x20
#define UKFS_BNO_UNIT_SEL_REG        0x3B
#define UKFS_BNO_OPR_MODE_REG        0x3D
#define UKFS_BNO_PWR_MODE_REG        0x3E
#define UKFS_BNO_SYS_TRIGGER_REG     0x3F

#define UKFS_BNO_MODE_CONFIG         0x00
#define UKFS_BNO_MODE_NDOF           0x0C

/* ============================ UKF =========================== */

#define UKFS_NX 4
#define UKFS_NZ 2
#define UKFS_NSIGMA (2 * UKFS_NX + 1)

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
#define UKFS_C               0.16f
#define UKFS_WM0           -24.0f
#define UKFS_WC0           -21.04f
#define UKFS_WI              3.125f

#define UKFS_SIGMA_ACC       50.0f
#define UKFS_SIGMA_ETA_RW     0.06f
#define UKFS_SIGMA_BIAS_RW    0.005f
#define UKFS_R_SCALE          0.02f

/* Generalized eta dynamics identified offline. */
#define UKFS_ETA_A1          -0.03967525f
#define UKFS_ETA_A2          -0.01119239f
#define UKFS_ETA_A3           0.02141572f
#define UKFS_ETA_A4          -0.01612887f
#define UKFS_ETA_A5          -0.10246220f
#define UKFS_ETA_A6          -0.01530813f

#define UKFS_PI               3.14159265358979323846f

#define UKFS_S_MIN            0.0f
#define UKFS_S_MAX          100.0f
#define UKFS_V_MIN         -250.0f
#define UKFS_V_MAX          250.0f
#define UKFS_ETA_MIN          0.0f
#define UKFS_ETA_MAX          1.0f
#define UKFS_BIAS_MIN       -80.0f
#define UKFS_BIAS_MAX        80.0f

#define UKFS_SIGMA_F_MIN      5.0f
#define UKFS_SIGMA_T_MIN      1.0f

#define UKFS_P_FLOOR          1.0e-7f
#define UKFS_S_FLOOR          1.0e-8f

/* Initial covariance from offline validation. */
#define UKFS_P0_S             16.0f
#define UKFS_P0_V            225.0f
#define UKFS_P0_ETA            0.0625f
#define UKFS_P0_BIAS         225.0f

/* ============================ TYPES ========================= */

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} UKFSQuat;

typedef struct
{
    bool initialized;
    bool reference_valid;
    bool valid;

    bool bno1_present;
    bool bno2_present;
    bool bno1_ok;
    bool bno2_ok;
    bool theta_valid;

    float bno1_qw;
    float bno1_qx;
    float bno1_qy;
    float bno1_qz;

    float bno2_qw;
    float bno2_qx;
    float bno2_qy;
    float bno2_qz;

    /* Kept for compatibility with existing logger. For UKF4 this is raw FLEX. */
    float flex_aligned;
    float theta_deg;

    float s_hat;
    float v_hat;
    float eta_hat;
    float b_f_hat;

    float innovation_flex;
    float innovation_theta;

    /* Effective sigmas after Rscale. */
    float sigma_flex_model;
    float sigma_theta_model;

    float sigma_s;
    float sigma_v;
    float sigma_eta;
    float sigma_b_f;

    float nis;
    float rho_eta_b_f;

    uint32_t exec_us;
    uint32_t max_exec_us;
} UKFShadowTelemetry;

/* ========================= INTERNAL STATE =================== */

static bool ukfs_initialized = false;
static bool ukfs_bno1_present = false;
static bool ukfs_bno2_present = false;
static bool ukfs_reference_valid = false;

static UKFSQuat ukfs_qrel0 = {1.0f, 0.0f, 0.0f, 0.0f};
static UKFSQuat ukfs_qdelta_prev = {1.0f, 0.0f, 0.0f, 0.0f};

static float ukfs_half_prev = 0.0f;
static float ukfs_half_accum = 0.0f;

/*
 * x = [s, v, eta, bF]
 */
static float ukfs_x[UKFS_NX] = {0.0f, 0.0f, 0.0f, 0.0f};

/* Full symmetric covariance. */
static float ukfs_P[UKFS_NX][UKFS_NX] =
{
    {UKFS_P0_S,   0.0f,          0.0f,           0.0f},
    {0.0f,        UKFS_P0_V,     0.0f,           0.0f},
    {0.0f,        0.0f,          UKFS_P0_ETA,     0.0f},
    {0.0f,        0.0f,          0.0f,            UKFS_P0_BIAS}
};

static uint32_t ukfs_last_update_us = 0u;

static UKFShadowTelemetry ukfs_telem = {0};

/* ========================== UTILITIES ======================= */

static inline float ukfs_clampf(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static inline int16_t ukfs_le_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static inline float ukfs_weight_m(int j)
{
    return (j == 0) ? UKFS_WM0 : UKFS_WI;
}

static inline float ukfs_weight_c(int j)
{
    return (j == 0) ? UKFS_WC0 : UKFS_WI;
}

static inline void ukfs_zero4(float A[UKFS_NX][UKFS_NX])
{
    for (int i = 0; i < UKFS_NX; i++)
        for (int j = 0; j < UKFS_NX; j++)
            A[i][j] = 0.0f;
}

static inline void ukfs_identity4(float A[UKFS_NX][UKFS_NX])
{
    ukfs_zero4(A);
    for (int i = 0; i < UKFS_NX; i++)
        A[i][i] = 1.0f;
}

static inline bool ukfs_cholesky4(
    const float A[UKFS_NX][UKFS_NX],
    float L[UKFS_NX][UKFS_NX]
)
{
    for (int i = 0; i < UKFS_NX; i++)
        for (int j = 0; j < UKFS_NX; j++)
            L[i][j] = 0.0f;

    for (int i = 0; i < UKFS_NX; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            float sum = A[i][j];

            for (int k = 0; k < j; k++)
                sum -= L[i][k] * L[j][k];

            if (i == j)
            {
                if (!isfinite(sum) || sum <= UKFS_P_FLOOR)
                    return false;

                L[i][j] = sqrtf(sum);
            }
            else
            {
                if (!isfinite(L[j][j]) || fabsf(L[j][j]) < UKFS_P_FLOOR)
                    return false;

                L[i][j] = sum / L[j][j];
            }
        }
    }

    return true;
}

static void ukfs_repair_cov4(float P[UKFS_NX][UKFS_NX])
{
    /* Symmetrize and enforce diagonal floors. */
    for (int i = 0; i < UKFS_NX; i++)
    {
        for (int j = i + 1; j < UKFS_NX; j++)
        {
            float a = 0.5f * (P[i][j] + P[j][i]);

            if (!isfinite(a))
                a = 0.0f;

            P[i][j] = a;
            P[j][i] = a;
        }

        if (!isfinite(P[i][i]) || P[i][i] < UKFS_P_FLOOR)
            P[i][i] = UKFS_P_FLOOR;
    }

    /* Limit pairwise covariance magnitude. */
    for (int i = 0; i < UKFS_NX; i++)
    {
        for (int j = i + 1; j < UKFS_NX; j++)
        {
            float lim =
                0.995f
                * sqrtf(fmaxf(P[i][i] * P[j][j], UKFS_P_FLOOR));

            P[i][j] = ukfs_clampf(P[i][j], -lim, lim);
            P[j][i] = P[i][j];
        }
    }

    /*
     * Ensure Cholesky success by diagonal loading.
     * With only 4 states this is cheap and deterministic.
     */
    float L[UKFS_NX][UKFS_NX];

    if (ukfs_cholesky4(P, L))
        return;

    float jitter = 1.0e-6f;

    for (int attempt = 0; attempt < 8; attempt++)
    {
        for (int i = 0; i < UKFS_NX; i++)
            P[i][i] += jitter;

        if (ukfs_cholesky4(P, L))
            return;

        jitter *= 10.0f;
    }

    /* Last-resort diagonal covariance. */
    for (int i = 0; i < UKFS_NX; i++)
    {
        for (int j = 0; j < UKFS_NX; j++)
        {
            if (i != j)
                P[i][j] = 0.0f;
        }

        if (P[i][i] < 1.0e-3f)
            P[i][i] = 1.0e-3f;
    }
}

/* ============================ LUT =========================== */

static inline float ukfs_lut(const float *table, float s)
{
    s = ukfs_clampf(s, UKF_MODEL_S_MIN, UKF_MODEL_S_MAX);

    if (s >= UKF_MODEL_S_MAX)
        return table[UKF_MODEL_N - 1];

    int i = (int)s;

    if (i < 0) i = 0;
    if (i >= UKF_MODEL_N - 1) i = UKF_MODEL_N - 2;

    float a = s - (float)i;

    return table[i] + a * (table[i + 1] - table[i]);
}

static inline void ukfs_measurement_model(
    const float x[UKFS_NX],
    float *flex_pred,
    float *theta_pred
)
{
    float s = ukfs_clampf(x[0], UKFS_S_MIN, UKFS_S_MAX);
    float eta = ukfs_clampf(x[2], UKFS_ETA_MIN, UKFS_ETA_MAX);

    float fup = ukfs_lut(UKF_MODEL_F_UP, s);
    float fdn = ukfs_lut(UKF_MODEL_F_DOWN, s);

    float tup = ukfs_lut(UKF_MODEL_THETA_UP_DEG, s);
    float tdn = ukfs_lut(UKF_MODEL_THETA_DOWN_DEG, s);

    *flex_pred =
        (1.0f - eta) * fup
        + eta * fdn
        + x[3];

    *theta_pred =
        (1.0f - eta) * tup
        + eta * tdn;
}

static inline void ukfs_measurement_sigma(
    const float x[UKFS_NX],
    float *sigma_f,
    float *sigma_t
)
{
    float s = ukfs_clampf(x[0], UKFS_S_MIN, UKFS_S_MAX);
    float eta = ukfs_clampf(x[2], UKFS_ETA_MIN, UKFS_ETA_MAX);

    float sfup = fmaxf(ukfs_lut(UKF_MODEL_SIGMA_F_UP, s), UKFS_SIGMA_F_MIN);
    float sfdn = fmaxf(ukfs_lut(UKF_MODEL_SIGMA_F_DOWN, s), UKFS_SIGMA_F_MIN);

    float stup = fmaxf(ukfs_lut(UKF_MODEL_SIGMA_THETA_UP_DEG, s), UKFS_SIGMA_T_MIN);
    float stdn = fmaxf(ukfs_lut(UKF_MODEL_SIGMA_THETA_DOWN_DEG, s), UKFS_SIGMA_T_MIN);

    float var_f =
        (1.0f - eta) * sfup * sfup
        + eta * sfdn * sfdn;

    float var_t =
        (1.0f - eta) * stup * stup
        + eta * stdn * stdn;

    var_f = fmaxf(var_f, UKFS_SIGMA_F_MIN * UKFS_SIGMA_F_MIN);
    var_t = fmaxf(var_t, UKFS_SIGMA_T_MIN * UKFS_SIGMA_T_MIN);

    /* Rscale multiplies variance, exactly as in the MATLAB sweep. */
    var_f *= UKFS_R_SCALE;
    var_t *= UKFS_R_SCALE;

    *sigma_f = sqrtf(fmaxf(var_f, UKFS_S_FLOOR));
    *sigma_t = sqrtf(fmaxf(var_t, UKFS_S_FLOOR));
}

/* ========================= QUATERNIONS ====================== */

static inline UKFSQuat ukfs_quat_normalize(UKFSQuat q)
{
    float n2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;

    if (!isfinite(n2) || n2 < 1.0e-12f)
        return (UKFSQuat){1.0f, 0.0f, 0.0f, 0.0f};

    float invn = 1.0f / sqrtf(n2);

    q.w *= invn;
    q.x *= invn;
    q.y *= invn;
    q.z *= invn;

    return q;
}

static inline UKFSQuat ukfs_quat_conj(UKFSQuat q)
{
    return (UKFSQuat){q.w, -q.x, -q.y, -q.z};
}

static inline UKFSQuat ukfs_quat_mul(UKFSQuat a, UKFSQuat b)
{
    UKFSQuat q;

    q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;

    return q;
}

static inline float ukfs_quat_dot(UKFSQuat a, UKFSQuat b)
{
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline UKFSQuat ukfs_quat_neg(UKFSQuat q)
{
    q.w = -q.w;
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    return q;
}

/*
 * Matches the offline convention:
 *     q_rel = inv(q_BNO1) (*) q_BNO2
 */
static inline UKFSQuat ukfs_relative_quat(UKFSQuat q1, UKFSQuat q2)
{
    q1 = ukfs_quat_normalize(q1);
    q2 = ukfs_quat_normalize(q2);

    return ukfs_quat_normalize(
        ukfs_quat_mul(
            ukfs_quat_conj(q1),
            q2
        )
    );
}

/* ============================ BNO =========================== */

static bool ukfs_bno_read_regs(
    uint8_t addr,
    uint8_t reg,
    uint8_t *data,
    size_t len
)
{
    int w = i2c_write_blocking(
        UKFS_I2C_PORT,
        addr,
        &reg,
        1,
        true
    );

    if (w != 1)
        return false;

    int r = i2c_read_blocking(
        UKFS_I2C_PORT,
        addr,
        data,
        len,
        false
    );

    return r == (int)len;
}

static bool ukfs_bno_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};

    return i2c_write_blocking(
        UKFS_I2C_PORT,
        addr,
        tx,
        2,
        false
    ) == 2;
}

static bool ukfs_bno_detect(uint8_t addr)
{
    uint8_t id = 0;

    return
        ukfs_bno_read_regs(
            addr,
            UKFS_BNO_CHIP_ID_REG,
            &id,
            1
        )
        && id == UKFS_BNO_CHIP_ID_VALUE;
}

static bool ukfs_bno_init(uint8_t addr)
{
    if (!ukfs_bno_detect(addr))
        return false;

    if (!ukfs_bno_write_reg(addr, UKFS_BNO_OPR_MODE_REG, UKFS_BNO_MODE_CONFIG))
        return false;

    sleep_ms(25);

    if (!ukfs_bno_write_reg(addr, UKFS_BNO_PAGE_ID_REG, 0x00))
        return false;

    if (!ukfs_bno_write_reg(addr, UKFS_BNO_UNIT_SEL_REG, 0x00))
        return false;

    if (!ukfs_bno_write_reg(addr, UKFS_BNO_PWR_MODE_REG, 0x00))
        return false;

    sleep_ms(10);

    if (!ukfs_bno_write_reg(addr, UKFS_BNO_SYS_TRIGGER_REG, 0x00))
        return false;

    sleep_ms(10);

    if (!ukfs_bno_write_reg(addr, UKFS_BNO_OPR_MODE_REG, UKFS_BNO_MODE_NDOF))
        return false;

    sleep_ms(30);

    return true;
}

static bool ukfs_bno_read_quat(
    uint8_t addr,
    bool present,
    UKFSQuat *q
)
{
    if (!present)
        return false;

    uint8_t data[8];

    if (!ukfs_bno_read_regs(
            addr,
            UKFS_BNO_QUA_DATA_W_LSB_REG,
            data,
            sizeof(data)))
        return false;

    q->w = (float)ukfs_le_i16(&data[0]) / 16384.0f;
    q->x = (float)ukfs_le_i16(&data[2]) / 16384.0f;
    q->y = (float)ukfs_le_i16(&data[4]) / 16384.0f;
    q->z = (float)ukfs_le_i16(&data[6]) / 16384.0f;

    *q = ukfs_quat_normalize(*q);

    return
        isfinite(q->w)
        && isfinite(q->x)
        && isfinite(q->y)
        && isfinite(q->z);
}

/* ======================== STATE MODEL ======================= */

static inline void ukfs_constrain_state(float x[UKFS_NX])
{
    x[0] = ukfs_clampf(x[0], UKFS_S_MIN, UKFS_S_MAX);
    x[1] = ukfs_clampf(x[1], UKFS_V_MIN, UKFS_V_MAX);
    x[2] = ukfs_clampf(x[2], UKFS_ETA_MIN, UKFS_ETA_MAX);
    x[3] = ukfs_clampf(x[3], UKFS_BIAS_MIN, UKFS_BIAS_MAX);

    if (x[0] <= UKFS_S_MIN && x[1] < 0.0f)
        x[1] = 0.0f;

    if (x[0] >= UKFS_S_MAX && x[1] > 0.0f)
        x[1] = 0.0f;
}

static inline void ukfs_process_model(
    const float xin[UKFS_NX],
    float dt,
    float xout[UKFS_NX]
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
        UKFS_ETA_A1 * v
        + UKFS_ETA_A2 * fabsf(v) * zeta
        + UKFS_ETA_A3 * v * fabsf(zeta)
        + UKFS_ETA_A4 * zeta
        + UKFS_ETA_A5 * sN
        + UKFS_ETA_A6;

    float zeta2 = zeta + dt * dzdt;
    zeta2 = ukfs_clampf(zeta2, -1.0f, 1.0f);

    xout[2] = 0.5f * (zeta2 + 1.0f);

    /* Deterministic part: bias remains constant. */
    xout[3] = b;

    ukfs_constrain_state(xout);
}

static inline void ukfs_process_noise(float dt, float Q[UKFS_NX][UKFS_NX])
{
    ukfs_zero4(Q);

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    float qAcc = UKFS_SIGMA_ACC * UKFS_SIGMA_ACC;

    Q[0][0] = qAcc * dt4 / 4.0f;
    Q[0][1] = qAcc * dt3 / 2.0f;
    Q[1][0] = Q[0][1];
    Q[1][1] = qAcc * dt2;

    Q[2][2] =
        UKFS_SIGMA_ETA_RW
        * UKFS_SIGMA_ETA_RW
        * dt;

    Q[3][3] =
        UKFS_SIGMA_BIAS_RW
        * UKFS_SIGMA_BIAS_RW
        * dt;
}

/* =========================== RESET ========================== */

static void ukf_shadow_reset_filter(void)
{
    ukfs_x[0] = 0.0f;
    ukfs_x[1] = 0.0f;
    ukfs_x[2] = 0.0f;
    ukfs_x[3] = 0.0f;

    ukfs_zero4(ukfs_P);

    ukfs_P[0][0] = UKFS_P0_S;
    ukfs_P[1][1] = UKFS_P0_V;
    ukfs_P[2][2] = UKFS_P0_ETA;
    ukfs_P[3][3] = UKFS_P0_BIAS;

    ukfs_qdelta_prev = (UKFSQuat){1.0f, 0.0f, 0.0f, 0.0f};

    ukfs_half_prev = 0.0f;
    ukfs_half_accum = 0.0f;

    ukfs_last_update_us = 0u;

    ukfs_telem.valid = false;
    ukfs_telem.theta_valid = false;

    ukfs_telem.s_hat = 0.0f;
    ukfs_telem.v_hat = 0.0f;
    ukfs_telem.eta_hat = 0.0f;
    ukfs_telem.b_f_hat = 0.0f;

    ukfs_telem.theta_deg = 0.0f;

    ukfs_telem.innovation_flex = NAN;
    ukfs_telem.innovation_theta = NAN;

    ukfs_telem.sigma_s = sqrtf(UKFS_P0_S);
    ukfs_telem.sigma_v = sqrtf(UKFS_P0_V);
    ukfs_telem.sigma_eta = sqrtf(UKFS_P0_ETA);
    ukfs_telem.sigma_b_f = sqrtf(UKFS_P0_BIAS);

    ukfs_telem.nis = NAN;
    ukfs_telem.rho_eta_b_f = 0.0f;
}

/* ======================= PUBLIC HW INIT ===================== */

static void ukf_shadow_hw_init(void)
{
    i2c_init(UKFS_I2C_PORT, 400u * 1000u);

    gpio_set_function(UKFS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(UKFS_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(UKFS_SDA_PIN);
    gpio_pull_up(UKFS_SCL_PIN);

    ukfs_bno1_present = ukfs_bno_init(UKFS_BNO1_ADDR);
    ukfs_bno2_present = ukfs_bno_init(UKFS_BNO2_ADDR);

    ukfs_telem.bno1_present = ukfs_bno1_present;
    ukfs_telem.bno2_present = ukfs_bno2_present;

    ukfs_initialized = true;
    ukfs_telem.initialized = true;

    ukf_shadow_reset_filter();

    printf(
        "#UKF4_SHADOW_HW,"
        "BNO1=%d,"
        "BNO2=%d,"
        "RATE_HZ=%.1f,"
        "SIGMA_ACC=%.3f,"
        "SIGMA_ETA_RW=%.5f,"
        "SIGMA_BF_RW=%.5f,"
        "R_SCALE=%.5f,"
        "FEEDBACK_TO_CONTROL=0\n",
        ukfs_bno1_present ? 1 : 0,
        ukfs_bno2_present ? 1 : 0,
        1000.0f / (float)UKF_SHADOW_LOOP_MS,
        UKFS_SIGMA_ACC,
        UKFS_SIGMA_ETA_RW,
        UKFS_SIGMA_BIAS_RW,
        UKFS_R_SCALE
    );
}

/* ==================== REFERENCE CALIBRATION ================= */

static bool ukf_shadow_calibrate_reference(float flex_at_zero_raw)
{
    /*
     * flex_at_zero_raw is intentionally NOT used to offset the FLEX
     * measurement. UKF4 estimates session/sensor offset through bF.
     */
    (void)flex_at_zero_raw;

    ukfs_reference_valid = false;

    if (!ukfs_initialized)
        return false;

    if (!ukfs_bno1_present || !ukfs_bno2_present)
    {
        ukf_shadow_reset_filter();
        ukfs_telem.reference_valid = false;

        printf(
            "#UKF4_REF,"
            "STATUS=FAILED_BNO,"
            "FLEX_REZERO=0,"
            "BIAS_STATE_ENABLED=1\n"
        );

        return false;
    }

    UKFSQuat sum = {0.0f, 0.0f, 0.0f, 0.0f};
    UKFSQuat qref = {1.0f, 0.0f, 0.0f, 0.0f};

    bool have_ref = false;
    uint32_t good = 0u;

    for (uint32_t i = 0u; i < UKF_SHADOW_REF_SAMPLES; i++)
    {
        UKFSQuat q1;
        UKFSQuat q2;

        bool ok1 = ukfs_bno_read_quat(
            UKFS_BNO1_ADDR,
            ukfs_bno1_present,
            &q1
        );

        bool ok2 = ukfs_bno_read_quat(
            UKFS_BNO2_ADDR,
            ukfs_bno2_present,
            &q2
        );

        if (ok1 && ok2)
        {
            UKFSQuat qr = ukfs_relative_quat(q1, q2);

            if (!have_ref)
            {
                qref = qr;
                have_ref = true;
            }

            if (ukfs_quat_dot(qr, qref) < 0.0f)
                qr = ukfs_quat_neg(qr);

            sum.w += qr.w;
            sum.x += qr.x;
            sum.y += qr.y;
            sum.z += qr.z;

            good++;
        }

        sleep_ms(UKF_SHADOW_REF_SAMPLE_DELAY_MS);
    }

    if (good < UKF_SHADOW_REF_SAMPLES / 2u)
    {
        ukf_shadow_reset_filter();
        ukfs_telem.reference_valid = false;

        printf(
            "#UKF4_REF,"
            "STATUS=FAILED,"
            "GOOD=%lu,"
            "FLEX_REZERO=0\n",
            (unsigned long)good
        );

        return false;
    }

    ukfs_qrel0 = ukfs_quat_normalize(sum);

    ukfs_reference_valid = true;

    ukf_shadow_reset_filter();
    ukfs_telem.reference_valid = true;

    printf(
        "#UKF4_REF,"
        "STATUS=PASS,"
        "GOOD=%lu,"
        "QREL0=%.6f,%.6f,%.6f,%.6f,"
        "FLEX_REZERO=0,"
        "BIAS_STATE_ENABLED=1\n",
        (unsigned long)good,
        ukfs_qrel0.w,
        ukfs_qrel0.x,
        ukfs_qrel0.y,
        ukfs_qrel0.z
    );

    return true;
}

/* ======================= ANGLE UPDATE ======================= */

static bool ukfs_update_theta(
    UKFSQuat q1,
    UKFSQuat q2,
    float *theta_deg
)
{
    if (!ukfs_reference_valid)
        return false;

    UKFSQuat qrel = ukfs_relative_quat(q1, q2);

    UKFSQuat qdelta =
        ukfs_quat_mul(
            ukfs_quat_conj(ukfs_qrel0),
            qrel
        );

    qdelta = ukfs_quat_normalize(qdelta);

    if (ukfs_quat_dot(qdelta, ukfs_qdelta_prev) < 0.0f)
        qdelta = ukfs_quat_neg(qdelta);

    ukfs_qdelta_prev = qdelta;

    float q_parallel =
        qdelta.x * UKF_MODEL_N_AXIS[0]
        + qdelta.y * UKF_MODEL_N_AXIS[1]
        + qdelta.z * UKF_MODEL_N_AXIS[2];

    float half_now = atan2f(q_parallel, qdelta.w);

    float dhalf = half_now - ukfs_half_prev;

    if (dhalf > UKFS_PI)
        dhalf -= 2.0f * UKFS_PI;
    else if (dhalf < -UKFS_PI)
        dhalf += 2.0f * UKFS_PI;

    ukfs_half_accum += dhalf;
    ukfs_half_prev = half_now;

    *theta_deg =
        2.0f
        * ukfs_half_accum
        * 180.0f
        / UKFS_PI;

    return isfinite(*theta_deg);
}

/* =========================== UKF CORE ======================= */

static void ukfs_generate_sigma_points(
    const float x[UKFS_NX],
    float P[UKFS_NX][UKFS_NX],
    float X[UKFS_NSIGMA][UKFS_NX]
)
{
    ukfs_repair_cov4(P);

    float A[UKFS_NX][UKFS_NX];
    float L[UKFS_NX][UKFS_NX];

    for (int i = 0; i < UKFS_NX; i++)
    {
        for (int j = 0; j < UKFS_NX; j++)
            A[i][j] = UKFS_C * P[i][j];
    }

    if (!ukfs_cholesky4(A, L))
    {
        for (int i = 0; i < UKFS_NX; i++)
            A[i][i] += 1.0e-5f;

        if (!ukfs_cholesky4(A, L))
        {
            for (int i = 0; i < UKFS_NX; i++)
            {
                for (int j = 0; j < UKFS_NX; j++)
                    L[i][j] = 0.0f;

                L[i][i] = sqrtf(fmaxf(A[i][i], UKFS_P_FLOOR));
            }
        }
    }

    for (int k = 0; k < UKFS_NX; k++)
        X[0][k] = x[k];

    for (int c = 0; c < UKFS_NX; c++)
    {
        int jp = 1 + c;
        int jm = 1 + UKFS_NX + c;

        for (int r = 0; r < UKFS_NX; r++)
        {
            X[jp][r] = x[r] + L[r][c];
            X[jm][r] = x[r] - L[r][c];
        }

        ukfs_constrain_state(X[jp]);
        ukfs_constrain_state(X[jm]);
    }
}

static void ukfs_prediction_only(
    const float XP[UKFS_NSIGMA][UKFS_NX],
    const float xpred[UKFS_NX],
    float Ppred[UKFS_NX][UKFS_NX]
)
{
    for (int i = 0; i < UKFS_NX; i++)
        ukfs_x[i] = xpred[i];

    for (int i = 0; i < UKFS_NX; i++)
        for (int j = 0; j < UKFS_NX; j++)
            ukfs_P[i][j] = Ppred[i][j];

    ukfs_repair_cov4(ukfs_P);

    ukfs_telem.innovation_flex = NAN;
    ukfs_telem.innovation_theta = NAN;
    ukfs_telem.nis = NAN;
    ukfs_telem.valid = false;

    (void)XP;
}

static void ukfs_update_filter(
    float flex_meas,
    bool theta_valid,
    float theta_meas,
    float dt
)
{
    float X[UKFS_NSIGMA][UKFS_NX];
    float XP[UKFS_NSIGMA][UKFS_NX];

    ukfs_generate_sigma_points(ukfs_x, ukfs_P, X);

    for (int j = 0; j < UKFS_NSIGMA; j++)
        ukfs_process_model(X[j], dt, XP[j]);

    float xpred[UKFS_NX] = {0};

    for (int j = 0; j < UKFS_NSIGMA; j++)
    {
        float w = ukfs_weight_m(j);

        for (int i = 0; i < UKFS_NX; i++)
            xpred[i] += w * XP[j][i];
    }

    ukfs_constrain_state(xpred);

    float Q[UKFS_NX][UKFS_NX];
    float Ppred[UKFS_NX][UKFS_NX];

    ukfs_process_noise(dt, Q);

    for (int i = 0; i < UKFS_NX; i++)
    {
        for (int k = 0; k < UKFS_NX; k++)
            Ppred[i][k] = Q[i][k];
    }

    for (int j = 0; j < UKFS_NSIGMA; j++)
    {
        float w = ukfs_weight_c(j);

        float dx[UKFS_NX];

        for (int i = 0; i < UKFS_NX; i++)
            dx[i] = XP[j][i] - xpred[i];

        for (int r = 0; r < UKFS_NX; r++)
        {
            for (int c = 0; c < UKFS_NX; c++)
                Ppred[r][c] += w * dx[r] * dx[c];
        }
    }

    ukfs_repair_cov4(Ppred);

    /*
     * For the 4-state model a FLEX-only update is intentionally not used.
     * Without theta, s/eta/bF are too strongly confounded.
     */
    if (!theta_valid || !isfinite(theta_meas) || !isfinite(flex_meas))
    {
        ukfs_prediction_only(XP, xpred, Ppred);
        return;
    }

    float Z[UKFS_NSIGMA][UKFS_NZ];

    for (int j = 0; j < UKFS_NSIGMA; j++)
    {
        ukfs_measurement_model(
            XP[j],
            &Z[j][0],
            &Z[j][1]
        );
    }

    float zpred[UKFS_NZ] = {0};

    for (int j = 0; j < UKFS_NSIGMA; j++)
    {
        float w = ukfs_weight_m(j);

        zpred[0] += w * Z[j][0];
        zpred[1] += w * Z[j][1];
    }

    float sigma_f;
    float sigma_t;

    ukfs_measurement_sigma(
        xpred,
        &sigma_f,
        &sigma_t
    );

    ukfs_telem.sigma_flex_model = sigma_f;
    ukfs_telem.sigma_theta_model = sigma_t;

    float S00 = sigma_f * sigma_f;
    float S01 = 0.0f;
    float S11 = sigma_t * sigma_t;

    float Pxz[UKFS_NX][UKFS_NZ] = {{0}};

    for (int j = 0; j < UKFS_NSIGMA; j++)
    {
        float w = ukfs_weight_c(j);

        float dz0 = Z[j][0] - zpred[0];
        float dz1 = Z[j][1] - zpred[1];

        S00 += w * dz0 * dz0;
        S01 += w * dz0 * dz1;
        S11 += w * dz1 * dz1;

        for (int i = 0; i < UKFS_NX; i++)
        {
            float dx = XP[j][i] - xpred[i];

            Pxz[i][0] += w * dx * dz0;
            Pxz[i][1] += w * dx * dz1;
        }
    }

    float det = S00 * S11 - S01 * S01;

    if (!isfinite(det) || det < UKFS_S_FLOOR)
    {
        ukfs_prediction_only(XP, xpred, Ppred);
        return;
    }

    float invS00 = S11 / det;
    float invS01 = -S01 / det;
    float invS11 = S00 / det;

    float K[UKFS_NX][UKFS_NZ];

    for (int i = 0; i < UKFS_NX; i++)
    {
        K[i][0] =
            Pxz[i][0] * invS00
            + Pxz[i][1] * invS01;

        K[i][1] =
            Pxz[i][0] * invS01
            + Pxz[i][1] * invS11;
    }

    float innov0 = flex_meas - zpred[0];
    float innov1 = theta_meas - zpred[1];

    for (int i = 0; i < UKFS_NX; i++)
    {
        ukfs_x[i] =
            xpred[i]
            + K[i][0] * innov0
            + K[i][1] * innov1;
    }

    ukfs_constrain_state(ukfs_x);

    /*
     * Joseph-style full form is unnecessary here because S already
     * contains the predicted innovation covariance; use P = P- KSK'.
     */
    for (int r = 0; r < UKFS_NX; r++)
    {
        for (int c = 0; c < UKFS_NX; c++)
        {
            float ksk =
                K[r][0] * (S00 * K[c][0] + S01 * K[c][1])
                + K[r][1] * (S01 * K[c][0] + S11 * K[c][1]);

            ukfs_P[r][c] = Ppred[r][c] - ksk;
        }
    }

    ukfs_repair_cov4(ukfs_P);

    float nis =
        innov0 * (invS00 * innov0 + invS01 * innov1)
        + innov1 * (invS01 * innov0 + invS11 * innov1);

    ukfs_telem.innovation_flex = innov0;
    ukfs_telem.innovation_theta = innov1;
    ukfs_telem.nis = nis;
    ukfs_telem.valid = true;
}

/* ======================= PUBLIC UPDATE ====================== */

static void ukf_shadow_update(float flex_raw)
{
    uint64_t t0 = time_us_64();

    UKFSQuat q1;
    UKFSQuat q2;

    bool ok1 = ukfs_bno_read_quat(
        UKFS_BNO1_ADDR,
        ukfs_bno1_present,
        &q1
    );

    bool ok2 = ukfs_bno_read_quat(
        UKFS_BNO2_ADDR,
        ukfs_bno2_present,
        &q2
    );

    ukfs_telem.bno1_ok = ok1;
    ukfs_telem.bno2_ok = ok2;

    if (ok1)
    {
        ukfs_telem.bno1_qw = q1.w;
        ukfs_telem.bno1_qx = q1.x;
        ukfs_telem.bno1_qy = q1.y;
        ukfs_telem.bno1_qz = q1.z;
    }

    if (ok2)
    {
        ukfs_telem.bno2_qw = q2.w;
        ukfs_telem.bno2_qx = q2.x;
        ukfs_telem.bno2_qy = q2.y;
        ukfs_telem.bno2_qz = q2.z;
    }

    float theta = NAN;
    bool theta_ok = false;

    if (ok1 && ok2)
        theta_ok = ukfs_update_theta(q1, q2, &theta);

    ukfs_telem.theta_valid = theta_ok;

    if (theta_ok)
        ukfs_telem.theta_deg = theta;

    /*
     * UKF4 uses the raw filtered FLEX counts directly.
     * No per-run FLEX re-zero is applied.
     */
    ukfs_telem.flex_aligned = flex_raw;

    uint32_t now32 = time_us_32();

    float dt =
        (ukfs_last_update_us == 0u)
        ? ((float)UKF_SHADOW_LOOP_MS / 1000.0f)
        : ((float)(now32 - ukfs_last_update_us) * 1.0e-6f);

    ukfs_last_update_us = now32;

    if (!isfinite(dt) || dt <= 0.0f || dt > 0.20f)
        dt = (float)UKF_SHADOW_LOOP_MS / 1000.0f;

    if (ukfs_reference_valid)
    {
        ukfs_update_filter(
            flex_raw,
            theta_ok,
            theta,
            dt
        );
    }
    else
    {
        ukfs_telem.valid = false;
    }

    ukfs_telem.reference_valid = ukfs_reference_valid;

    ukfs_telem.s_hat = ukfs_x[0];
    ukfs_telem.v_hat = ukfs_x[1];
    ukfs_telem.eta_hat = ukfs_x[2];
    ukfs_telem.b_f_hat = ukfs_x[3];

    ukfs_telem.sigma_s = sqrtf(fmaxf(ukfs_P[0][0], 0.0f));
    ukfs_telem.sigma_v = sqrtf(fmaxf(ukfs_P[1][1], 0.0f));
    ukfs_telem.sigma_eta = sqrtf(fmaxf(ukfs_P[2][2], 0.0f));
    ukfs_telem.sigma_b_f = sqrtf(fmaxf(ukfs_P[3][3], 0.0f));

    float denom =
        sqrtf(
            fmaxf(
                ukfs_P[2][2] * ukfs_P[3][3],
                UKFS_P_FLOOR
            )
        );

    ukfs_telem.rho_eta_b_f =
        ukfs_clampf(
            ukfs_P[2][3] / denom,
            -1.0f,
            1.0f
        );

    uint64_t t1 = time_us_64();

    uint32_t exec =
        (uint32_t)(t1 - t0);

    ukfs_telem.exec_us = exec;

    if (exec > ukfs_telem.max_exec_us)
        ukfs_telem.max_exec_us = exec;
}

/* ======================== PUBLIC GETTER ===================== */

static const UKFShadowTelemetry *ukf_shadow_get(void)
{
    return &ukfs_telem;
}

#endif /* UKF_SHADOW_RP2040_H */
