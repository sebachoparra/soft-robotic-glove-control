// Copyright 2026 Sebastian Parra
//
// Stage 1 Differential Equivalence Test
// Compares extracted pure functions in soft_glove_core directly against
// the frozen RP2040 golden reference in the same binary.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* The extracted side */
#include "soft_glove_core/soft_glove_core.h"

/* The golden side: pulls in the real static functions, unmodified */
#include "main.c"
#undef main

typedef struct {
    const char *name;
    uint64_t compared;
    uint64_t mismatched;
} BlockStats;

static inline bool compare_float_bits(float a, float b)
{
    return memcmp(&a, &b, sizeof(float)) == 0;
}

/* =================================================================
 * TEST BLOCK 1: LUT
 * ================================================================= */

static void test_b1_lut(BlockStats *stats)
{
    printf("--- Running Block 1: LUT (ukfs_lut vs sgc_lut) ---\n");

    const struct {
        const char *name;
        const float *golden_table;
        const float *ext_table;
    } tables[8] = {
        {"UKF_MODEL_F_UP", UKF_MODEL_F_UP, sgc_lut_f_up},
        {"UKF_MODEL_F_DOWN", UKF_MODEL_F_DOWN, sgc_lut_f_down},
        {"UKF_MODEL_THETA_UP_DEG", UKF_MODEL_THETA_UP_DEG, sgc_lut_theta_up_deg},
        {"UKF_MODEL_THETA_DOWN_DEG", UKF_MODEL_THETA_DOWN_DEG, sgc_lut_theta_down_deg},
        {"UKF_MODEL_SIGMA_F_UP", UKF_MODEL_SIGMA_F_UP, sgc_lut_sigma_f_up},
        {"UKF_MODEL_SIGMA_F_DOWN", UKF_MODEL_SIGMA_F_DOWN, sgc_lut_sigma_f_down},
        {"UKF_MODEL_SIGMA_THETA_UP_DEG", UKF_MODEL_SIGMA_THETA_UP_DEG, sgc_lut_sigma_theta_up_deg},
        {"UKF_MODEL_SIGMA_THETA_DOWN_DEG", UKF_MODEL_SIGMA_THETA_DOWN_DEG, sgc_lut_sigma_theta_down_deg}
    };

    // 1. Table buffer byte-for-byte comparison
    for (int t = 0; t < 8; ++t) {
        stats->compared++;
        if (memcmp(tables[t].golden_table, tables[t].ext_table, UKF_MODEL_N * sizeof(float)) != 0) {
            stats->mismatched++;
            printf("  FAIL: Table byte divergence in %s\n", tables[t].name);
        }
    }

    // 2. Point evaluations across all 8 tables
    // Inputs: all 101 grid points, midpoints, s < 0, s > 100, s = 100, s = 99.5, fractional sweep
    const float specific_probes[] = {
        -1000.0f, -100.0f, -50.0f, -10.0f, -1.0f, -0.5f, -0.01f, -0.001f, -1e-6f, -0.0f,
        0.0f, 99.5f, 100.0f,
        100.0001f, 100.001f, 100.01f, 100.1f, 100.5f, 101.0f, 150.0f, 200.0f, 1000.0f
    };
    const size_t num_probes = sizeof(specific_probes) / sizeof(specific_probes[0]);

    for (int t = 0; t < 8; ++t) {
        // Evaluate with extracted table and golden table
        // (a) All 101 grid points
        for (int i = 0; i < UKF_MODEL_N; ++i) {
            float s = (float)i;
            float g = ukfs_lut(tables[t].golden_table, s);
            float e = sgc_lut(tables[t].ext_table, s);
            stats->compared++;
            if (!compare_float_bits(g, e)) {
                stats->mismatched++;
                printf("  FAIL: B1 %s at grid s=%f: g=%f, e=%f\n", tables[t].name, s, g, e);
            }
        }

        // (b) Midpoints
        for (int i = 0; i < UKF_MODEL_N - 1; ++i) {
            float s = (float)i + 0.5f;
            float g = ukfs_lut(tables[t].golden_table, s);
            float e = sgc_lut(tables[t].ext_table, s);
            stats->compared++;
            if (!compare_float_bits(g, e)) {
                stats->mismatched++;
                printf("  FAIL: B1 %s at midpoint s=%f: g=%f, e=%f\n", tables[t].name, s, g, e);
            }
        }

        // (c) Quarter points
        for (int i = 0; i < UKF_MODEL_N - 1; ++i) {
            float s1 = (float)i + 0.25f;
            float s2 = (float)i + 0.75f;
            float g1 = ukfs_lut(tables[t].golden_table, s1);
            float e1 = sgc_lut(tables[t].ext_table, s1);
            stats->compared++;
            if (!compare_float_bits(g1, e1)) {
                stats->mismatched++;
            }
            float g2 = ukfs_lut(tables[t].golden_table, s2);
            float e2 = sgc_lut(tables[t].ext_table, s2);
            stats->compared++;
            if (!compare_float_bits(g2, e2)) {
                stats->mismatched++;
            }
        }

        // (d) Specific edge & out-of-range probes
        for (size_t p = 0; p < num_probes; ++p) {
            float s = specific_probes[p];
            float g = ukfs_lut(tables[t].golden_table, s);
            float e = sgc_lut(tables[t].ext_table, s);
            stats->compared++;
            if (!compare_float_bits(g, e)) {
                stats->mismatched++;
                printf("  FAIL: B1 %s at probe s=%f: g=%f, e=%f\n", tables[t].name, s, g, e);
            }
        }

        // (e) Fine step sweep: 0.0 to 100.0 with step 0.1
        for (int k = 0; k <= 1000; ++k) {
            float s = (float)k * 0.1f;
            float g = ukfs_lut(tables[t].golden_table, s);
            float e = sgc_lut(tables[t].ext_table, s);
            stats->compared++;
            if (!compare_float_bits(g, e)) {
                stats->mismatched++;
            }
        }
    }
}

/* =================================================================
 * TEST BLOCK 2: PLANT_SCHEDULE
 * ================================================================= */

static void eval_and_compare_plant_schedule(float q, float eta, BlockStats *stats)
{
    PlantSchedule g_out;
    SgcPlantSchedule e_out;
    memset(&g_out, 0xAA, sizeof(g_out));
    memset(&e_out, 0x55, sizeof(e_out));

    bool g_ret = plant_schedule_eval(q, eta, &g_out);
    bool e_ret = sgc_plant_schedule_eval(q, eta, &e_out);

    stats->compared++;
    if (g_ret != e_ret) {
        stats->mismatched++;
        printf("  FAIL: B2 return mismatch at q=%f, eta=%f: g=%d, e=%d\n", q, eta, g_ret, e_ret);
        return;
    }

    if (g_ret) {
        // Compare every field
        stats->compared++;
        if (!compare_float_bits(g_out.q, e_out.q)) stats->mismatched++;

        stats->compared++;
        if (!compare_float_bits(g_out.eta, e_out.eta)) stats->mismatched++;

        stats->compared++;
        if (!compare_float_bits(g_out.k, e_out.k)) stats->mismatched++;

        stats->compared++;
        if (!compare_float_bits(g_out.tau, e_out.tau)) stats->mismatched++;

        stats->compared++;
        if (!compare_float_bits(g_out.kp, e_out.kp)) stats->mismatched++;

        stats->compared++;
        if (!compare_float_bits(g_out.ki, e_out.ki)) stats->mismatched++;

        stats->compared++;
        if (!compare_float_bits(g_out.b0, e_out.b0)) stats->mismatched++;

        stats->compared++;
        if (g_out.clamped != e_out.clamped) {
            stats->mismatched++;
            printf("  FAIL: B2 clamped flag mismatch at q=%f, eta=%f: g=%d, e=%d\n",
                   q, eta, g_out.clamped, e_out.clamped);
        }
    }
}

static void test_b2_plant_schedule(BlockStats *stats)
{
    printf("--- Running Block 2: Plant Schedule (plant_schedule_eval vs sgc_plant_schedule_eval) ---\n");

    // Check table memory identity
    stats->compared++;
    if (memcmp(PLANT_ANCHORS, sgc_plant_anchors, sizeof(PLANT_ANCHORS)) != 0) {
        stats->mismatched++;
        printf("  FAIL: PLANT_ANCHORS table divergence\n");
    }

    // 1. Six anchor evaluation points: {30, 50, 70} x {eta=0, eta=1}
    const float anchors[3] = {30.0f, 50.0f, 70.0f};
    const float etas[2] = {0.0f, 1.0f};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 2; ++j) {
            eval_and_compare_plant_schedule(anchors[i], etas[j], stats);
        }
    }

    // 2. Segment boundary q=50 with various eta
    const float boundary_etas[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
    for (size_t i = 0; i < sizeof(boundary_etas)/sizeof(boundary_etas[0]); ++i) {
        eval_and_compare_plant_schedule(50.0f, boundary_etas[i], stats);
    }

    // 3. Dense sweep over valid region [30, 70] x [0, 1]
    for (int qi = 300; qi <= 700; qi += 5) {
        float q = (float)qi * 0.1f;
        for (int ei = 0; ei <= 100; ei += 10) {
            float eta = (float)ei * 0.01f;
            eval_and_compare_plant_schedule(q, eta, stats);
        }
    }

    // 4. Clamping: q < 30 and q > 70
    const float q_clamps[] = {-100.0f, -10.0f, 0.0f, 15.0f, 29.0f, 29.99f, 70.01f, 71.0f, 85.0f, 150.0f};
    for (size_t i = 0; i < sizeof(q_clamps)/sizeof(q_clamps[0]); ++i) {
        for (int ei = 0; ei <= 100; ei += 25) {
            eval_and_compare_plant_schedule(q_clamps[i], (float)ei * 0.01f, stats);
        }
    }

    // 5. Clamping: eta < 0 and eta > 1
    const float eta_clamps[] = {-10.0f, -2.0f, -0.5f, -0.001f, 1.001f, 1.5f, 2.0f, 10.0f};
    for (size_t i = 0; i < sizeof(eta_clamps)/sizeof(eta_clamps[0]); ++i) {
        for (int qi = 30; qi <= 70; qi += 10) {
            eval_and_compare_plant_schedule((float)qi, eta_clamps[i], stats);
        }
    }

    // 6. Joint clamping: both q and eta out of bounds
    for (size_t i = 0; i < sizeof(q_clamps)/sizeof(q_clamps[0]); ++i) {
        for (size_t j = 0; j < sizeof(eta_clamps)/sizeof(eta_clamps[0]); ++j) {
            eval_and_compare_plant_schedule(q_clamps[i], eta_clamps[j], stats);
        }
    }

    // 7. NaN and +/-Inf inputs
    const float special_vals[] = {NAN, (float)INFINITY, (float)-INFINITY};
    for (int i = 0; i < 3; ++i) {
        eval_and_compare_plant_schedule(special_vals[i], 0.5f, stats);
        eval_and_compare_plant_schedule(45.0f, special_vals[i], stats);
        for (int j = 0; j < 3; ++j) {
            eval_and_compare_plant_schedule(special_vals[i], special_vals[j], stats);
        }
    }

    // 8. out == NULL
    bool g_null = plant_schedule_eval(50.0f, 0.5f, NULL);
    bool e_null = sgc_plant_schedule_eval(50.0f, 0.5f, NULL);
    stats->compared++;
    if (g_null != e_null || g_null != false) {
        stats->mismatched++;
        printf("  FAIL: B2 out==NULL mismatch: g=%d, e=%d\n", g_null, e_null);
    }

    bool g_null_nan = plant_schedule_eval(NAN, NAN, NULL);
    bool e_null_nan = sgc_plant_schedule_eval(NAN, NAN, NULL);
    stats->compared++;
    if (g_null_nan != e_null_nan || g_null_nan != false) {
        stats->mismatched++;
        printf("  FAIL: B2 out==NULL with NaN mismatch: g=%d, e=%d\n", g_null_nan, e_null_nan);
    }
}

/* =================================================================
 * TEST BLOCK 3: CONVERSIONS
 * ================================================================= */

static void test_b3_conversions(BlockStats *stats)
{
    printf("--- Running Block 3: Conversions (pressure/flex raw to engineering units) ---\n");

    const float zero_vouts[] = {-0.5f, 0.0f, 0.1f, 0.5f, 0.825f, 1.545f, 2.0f, 3.3f};
    const size_t num_vouts = sizeof(zero_vouts) / sizeof(zero_vouts[0]);

    const float zero_raws[] = {-100.0f, 0.0f, 200.0f, 500.0f, 1000.0f, 2048.0f, 3000.0f, 4095.0f, 5000.0f};
    const size_t num_raws = sizeof(zero_raws) / sizeof(zero_raws[0]);

    // 1. raw sweep [0, 4095]
    for (int r = 0; r <= 4095; ++r) {
        float raw = (float)r;

        // pressure_raw_to_vout
        float g_vout = pressure_raw_to_vout(raw);
        float e_vout = sgc_pressure_raw_to_vout(raw);
        stats->compared++;
        if (!compare_float_bits(g_vout, e_vout)) {
            stats->mismatched++;
            printf("  FAIL: pressure_raw_to_vout at raw=%f\n", raw);
        }

        // pressure_raw_to_kpa with various pressure_zero_vout
        for (size_t v = 0; v < num_vouts; ++v) {
            pressure_zero_vout = zero_vouts[v];
            float g_kpa = pressure_raw_to_kpa(raw);
            float e_kpa = sgc_pressure_raw_to_kpa(raw, zero_vouts[v]);
            stats->compared++;
            if (!compare_float_bits(g_kpa, e_kpa)) {
                stats->mismatched++;
                printf("  FAIL: pressure_raw_to_kpa at raw=%f, zero_vout=%f\n", raw, zero_vouts[v]);
            }
        }

        // flex_raw_to_position_unclipped with various flex_zero_raw
        for (size_t f = 0; f < num_raws; ++f) {
            flex_zero_raw = zero_raws[f];
            float g_flex = flex_raw_to_position_unclipped(raw);
            float e_flex = sgc_flex_raw_to_position_unclipped(raw, zero_raws[f]);
            stats->compared++;
            if (!compare_float_bits(g_flex, e_flex)) {
                stats->mismatched++;
                printf("  FAIL: flex_raw_to_position_unclipped at raw=%f, zero_raw=%f\n", raw, zero_raws[f]);
            }
        }
    }

    // 2. Out of range raw values
    const float out_raws[] = {-1000.0f, -100.0f, -10.0f, -1.0f, -0.5f, 4095.5f, 4096.0f, 5000.0f, 10000.0f};
    for (size_t i = 0; i < sizeof(out_raws)/sizeof(out_raws[0]); ++i) {
        float raw = out_raws[i];

        float g_vout = pressure_raw_to_vout(raw);
        float e_vout = sgc_pressure_raw_to_vout(raw);
        stats->compared++;
        if (!compare_float_bits(g_vout, e_vout)) {
            stats->mismatched++;
        }

        for (size_t v = 0; v < num_vouts; ++v) {
            pressure_zero_vout = zero_vouts[v];
            float g_kpa = pressure_raw_to_kpa(raw);
            float e_kpa = sgc_pressure_raw_to_kpa(raw, zero_vouts[v]);
            stats->compared++;
            if (!compare_float_bits(g_kpa, e_kpa)) {
                stats->mismatched++;
            }
        }

        for (size_t f = 0; f < num_raws; ++f) {
            flex_zero_raw = zero_raws[f];
            float g_flex = flex_raw_to_position_unclipped(raw);
            float e_flex = sgc_flex_raw_to_position_unclipped(raw, zero_raws[f]);
            stats->compared++;
            if (!compare_float_bits(g_flex, e_flex)) {
                stats->mismatched++;
            }
        }
    }
}

/* =================================================================
 * TEST BLOCK 4: EWMA
 * ================================================================= */

static void test_b4_ewma(BlockStats *stats)
{
    printf("--- Running Block 4: EWMA (ewma vs sgc_ewma) ---\n");

    const float alphas[] = {0.0f, 0.05f, 0.20f, 0.50f, 0.80f, 1.0f};
    const size_t num_alphas = sizeof(alphas) / sizeof(alphas[0]);

    const float test_values[] = {
        -500.0f, -100.0f, -10.0f, -1.0f, -0.5f, 0.0f, 0.5f, 1.0f,
        10.0f, 50.0f, 100.0f, 1000.0f, 4095.0f, 10000.0f
    };
    const size_t num_vals = sizeof(test_values) / sizeof(test_values[0]);

    // 1. Grid of alpha x x x previous
    for (size_t a = 0; a < num_alphas; ++a) {
        float alpha = alphas[a];
        for (size_t x_idx = 0; x_idx < num_vals; ++x_idx) {
            float x = test_values[x_idx];
            for (size_t p_idx = 0; p_idx < num_vals; ++p_idx) {
                float prev = test_values[p_idx];
                float g = ewma(x, prev, alpha);
                float e = sgc_ewma(x, prev, alpha);
                stats->compared++;
                if (!compare_float_bits(g, e)) {
                    stats->mismatched++;
                    printf("  FAIL: EWMA divergence: x=%f, prev=%f, alpha=%f (g=%f, e=%f)\n",
                           x, prev, alpha, g, e);
                }
            }
        }
    }

    // 2. Multi-step recursive sequence
    for (size_t a = 0; a < num_alphas; ++a) {
        float alpha = alphas[a];
        float g_state = 0.0f;
        float e_state = 0.0f;

        // Run 200 dynamic steps (step, ramp, oscillation)
        for (int step = 0; step < 200; ++step) {
            float input = sinf((float)step * 0.1f) * 100.0f + (float)(step % 20);
            g_state = ewma(input, g_state, alpha);
            e_state = sgc_ewma(input, e_state, alpha);

            stats->compared++;
            if (!compare_float_bits(g_state, e_state)) {
                stats->mismatched++;
                printf("  FAIL: EWMA recursive sequence step=%d, alpha=%f\n", step, alpha);
            }
        }
    }
}

/* =================================================================
 * TEST BLOCK 5: MAGNITUDE_TO_PULSE
 * ================================================================= */

static void test_b5_magnitude_to_pulse(BlockStats *stats)
{
    printf("--- Running Block 5: Magnitude to Pulse (magnitude_to_pulse vs sgc_magnitude_to_pulse) ---\n");

    const struct {
        uint32_t min_ms;
        uint32_t max_ms;
    } ranges[] = {
        {FILL_MIN_MS, FILL_MAX_MS},   // 5, 80 (main.c:1713)
        {VENT_MIN_MS, VENT_MAX_MS},   // 10, 100 (main.c:1735)
        {0, 0},
        {50, 50},
        {20, 100}
    };
    const size_t num_ranges = sizeof(ranges) / sizeof(ranges[0]);

    // Test specific boundary values and values exposing truncation vs rounding
    const float specific_magnitudes[] = {
        // Boundaries and out-of-range
        -100.0f, -10.0f, -1.0f, -0.1f, -0.0001f, 0.0f, 1.0f, 1.0001f, 1.1f, 2.0f, 10.0f,
        // Truncation-exposing points: values where (float)min + mag*(max-min) has fractional part > 0.5
        // With min=5, max=80, span=75:
        // 0.0123 * 75 = 0.9225 -> 5.9225 -> truncates to 5 (round would be 6)
        0.0123f,
        // 0.533 * 75 = 39.975 -> 44.975 -> truncates to 44 (round would be 45)
        0.533f,
        // 0.999 * 75 = 74.925 -> 79.925 -> truncates to 79 (round would be 80)
        0.999f,
        // With min=10, max=100, span=90:
        // 0.0099 * 90 = 0.891 -> 10.891 -> truncates to 10 (round would be 11)
        0.0099f,
        // 0.9999 * 90 = 89.991 -> 99.991 -> truncates to 99 (round would be 100)
        0.9999f,
        0.25f, 0.5f, 0.75f
    };
    const size_t num_mags = sizeof(specific_magnitudes) / sizeof(specific_magnitudes[0]);

    for (size_t r = 0; r < num_ranges; ++r) {
        uint32_t min_ms = ranges[r].min_ms;
        uint32_t max_ms = ranges[r].max_ms;

        // Specific probes
        for (size_t m = 0; m < num_mags; ++m) {
            float mag = specific_magnitudes[m];
            uint32_t g = magnitude_to_pulse(mag, min_ms, max_ms);
            uint32_t e = sgc_magnitude_to_pulse(mag, min_ms, max_ms);

            stats->compared++;
            if (g != e) {
                stats->mismatched++;
                printf("  FAIL: B5 mag=%f, min=%u, max=%u: g=%u, e=%u\n", mag, min_ms, max_ms, g, e);
            }
        }

        // Dense sweep over [-0.2, 1.2] in steps of 0.001
        for (int k = -200; k <= 1200; ++k) {
            float mag = (float)k * 0.001f;
            uint32_t g = magnitude_to_pulse(mag, min_ms, max_ms);
            uint32_t e = sgc_magnitude_to_pulse(mag, min_ms, max_ms);

            stats->compared++;
            if (g != e) {
                stats->mismatched++;
                printf("  FAIL: B5 sweep mag=%f, min=%u, max=%u: g=%u, e=%u\n", mag, min_ms, max_ms, g, e);
            }
        }
    }
}

/* =================================================================
 * TEST BLOCK 6: RATE_LIMITER
 * ================================================================= */

static void eval_and_compare_rate_limiter(float raw_pref, float prev_pref, BlockStats *stats)
{
    bool g_rl = false, g_sat = false;
    bool e_rl = false, e_sat = false;

    float g_val = apply_common_pref_limits(raw_pref, prev_pref, &g_rl, &g_sat);
    float e_val = sgc_apply_common_pref_limits(raw_pref, prev_pref, &e_rl, &e_sat);

    stats->compared++;
    if (!compare_float_bits(g_val, e_val)) {
        stats->mismatched++;
        printf("  FAIL: B6 value mismatch: raw=%f, prev=%f (g=%f, e=%f)\n", raw_pref, prev_pref, g_val, e_val);
    }

    stats->compared++;
    if (g_rl != e_rl) {
        stats->mismatched++;
        printf("  FAIL: B6 rate_limited mismatch: raw=%f, prev=%f (g=%d, e=%d)\n",
               raw_pref, prev_pref, g_rl, e_rl);
    }

    stats->compared++;
    if (g_sat != e_sat) {
        stats->mismatched++;
        printf("  FAIL: B6 saturated mismatch: raw=%f, prev=%f (g=%d, e=%d)\n",
               raw_pref, prev_pref, g_sat, e_sat);
    }
}

static void test_b6_rate_limiter(BlockStats *stats)
{
    printf("--- Running Block 6: Rate Limiter (apply_common_pref_limits vs sgc_apply_common_pref_limits) ---\n");

    // 1. Specific rate limiting step tests
    // max_up_step = 10.0f * 0.500f = 5.0f
    // max_down_step = 12.0f * 0.500f = 6.0f
    const float prev_test = 50.0f;
    const float raw_steps[] = {
        prev_test - 10.0f, prev_test - 6.001f, prev_test - 6.0f, prev_test - 5.999f, prev_test - 3.0f,
        prev_test,
        prev_test + 3.0f, prev_test + 4.999f, prev_test + 5.0f, prev_test + 5.001f, prev_test + 10.0f
    };
    for (size_t i = 0; i < sizeof(raw_steps)/sizeof(raw_steps[0]); ++i) {
        eval_and_compare_rate_limiter(raw_steps[i], prev_test, stats);
    }

    // 2. Rate-limit before clamp tests
    // Exceeds rate limit, AND resulting limited value exceeds [0, 170]
    eval_and_compare_rate_limiter(200.0f, 168.0f, stats);  // limited=173, clamp=170 -> rl=true, sat=true
    eval_and_compare_rate_limiter(-50.0f, 2.0f, stats);    // limited=-4, clamp=0 -> rl=true, sat=true

    // 3. Clamp [0, 170] without rate limit
    eval_and_compare_rate_limiter(172.0f, 169.0f, stats);  // 172 <= 174, not rate limited, clamped to 170 -> rl=false, sat=true
    eval_and_compare_rate_limiter(-1.0f, 1.0f, stats);     // -1 >= -5, not rate limited, clamped to 0 -> rl=false, sat=true

    // 4. All four flag combinations
    // (false, false)
    eval_and_compare_rate_limiter(52.0f, 50.0f, stats);
    // (true, false)
    eval_and_compare_rate_limiter(60.0f, 50.0f, stats);
    // (false, true)
    eval_and_compare_rate_limiter(172.0f, 169.0f, stats);
    // (true, true)
    eval_and_compare_rate_limiter(200.0f, 168.0f, stats);

    // 5. Threshold 1e-6f saturation tests around upper limit (170.0f) and lower limit (0.0f)
    const float deltas[] = {
        0.0f, 1e-8f, 1e-7f, 0.5e-6f, 0.9e-6f, 1.0e-6f, 1.01e-6f, 1.1e-6f, 1.5e-6f, 2.0e-6f, 1e-5f, 1e-4f, 1e-2f
    };
    for (size_t d = 0; d < sizeof(deltas)/sizeof(deltas[0]); ++d) {
        // Near upper clamp: prev=170, raw=170 + delta
        eval_and_compare_rate_limiter(170.0f + deltas[d], 170.0f, stats);
        // Near upper clamp: prev=169, raw=170 + delta
        eval_and_compare_rate_limiter(170.0f + deltas[d], 169.0f, stats);
        // Near lower clamp: prev=0, raw=0 - delta
        eval_and_compare_rate_limiter(0.0f - deltas[d], 0.0f, stats);
        // Near lower clamp: prev=1, raw=0 - delta
        eval_and_compare_rate_limiter(0.0f - deltas[d], 1.0f, stats);
    }

    // 6. Dense 2D sweep: previous_pref x raw_pref
    const float prev_grid[] = {-20.0f, 0.0f, 10.0f, 50.0f, 100.0f, 160.0f, 168.0f, 170.0f, 180.0f};
    const float raw_grid[] = {
        -30.0f, -10.0f, 0.0f, 5.0f, 25.0f, 50.0f, 75.0f, 100.0f, 150.0f, 165.0f, 170.0f, 175.0f, 200.0f
    };
    for (size_t p = 0; p < sizeof(prev_grid)/sizeof(prev_grid[0]); ++p) {
        for (size_t r = 0; r < sizeof(raw_grid)/sizeof(raw_grid[0]); ++r) {
            eval_and_compare_rate_limiter(raw_grid[r], prev_grid[p], stats);
        }
    }
}

/* =================================================================
 * UTILITY: CLAMPF
 * ================================================================= */

static void test_clampf(BlockStats *stats)
{
    printf("--- Running Utility: clampf (clampf_local vs sgc_clampf) ---\n");

    const float bounds[][2] = {
        {0.0f, 100.0f},
        {0.0f, 1.0f},
        {-10.0f, 10.0f},
        {50.0f, 50.0f},
        {-100.0f, -50.0f}
    };
    for (size_t b = 0; b < sizeof(bounds)/sizeof(bounds[0]); ++b) {
        float xmin = bounds[b][0];
        float xmax = bounds[b][1];

        // Specific probes
        float probes[] = {
            xmin - 100.0f, xmin - 1.0f, xmin - 0.001f, xmin,
            (xmin + xmax) * 0.5f,
            xmax, xmax + 0.001f, xmax + 1.0f, xmax + 100.0f
        };
        for (size_t p = 0; p < sizeof(probes)/sizeof(probes[0]); ++p) {
            float x = probes[p];
            float g = clampf_local(x, xmin, xmax);
            float e = sgc_clampf(x, xmin, xmax);
            stats->compared++;
            if (!compare_float_bits(g, e)) {
                stats->mismatched++;
            }
        }
    }
}

/* =================================================================
 * MAIN ENTRY POINT
 * ================================================================= */

int main(void)
{
    printf("=================================================================\n");
    printf("STAGE 1 DIFFERENTIAL EQUIVALENCE SUITE\n");
    printf("=================================================================\n\n");

    BlockStats b1 = {"B1_LUT", 0, 0};
    BlockStats b2 = {"B2_PLANT_SCHEDULE", 0, 0};
    BlockStats b3 = {"B3_CONVERSIONS", 0, 0};
    BlockStats b4 = {"B4_EWMA", 0, 0};
    BlockStats b5 = {"B5_MAGNITUDE_TO_PULSE", 0, 0};
    BlockStats b6 = {"B6_RATE_LIMITER", 0, 0};
    BlockStats b_clamp = {"CLAMPF_UTILITY", 0, 0};

    test_b1_lut(&b1);
    test_b2_plant_schedule(&b2);
    test_b3_conversions(&b3);
    test_b4_ewma(&b4);
    test_b5_magnitude_to_pulse(&b5);
    test_b6_rate_limiter(&b6);
    test_clampf(&b_clamp);

    printf("\n=================================================================\n");
    printf("RESULTS SUMMARY\n");
    printf("=================================================================\n");
    printf("%-25s %12s %12s %10s\n", "Block", "Compared", "Mismatched", "Status");
    printf("-----------------------------------------------------------------\n");

    BlockStats all[7] = {b1, b2, b3, b4, b5, b6, b_clamp};
    uint64_t total_compared = 0;
    uint64_t total_mismatched = 0;
    bool any_empty = false;

    for (int i = 0; i < 7; ++i) {
        if (all[i].compared == 0) {
            any_empty = true;
        }
        total_compared += all[i].compared;
        total_mismatched += all[i].mismatched;
        const char *status = (all[i].compared > 0 && all[i].mismatched == 0) ? "PASS" : "FAIL";
        printf("%-25s %12lu %12lu %10s\n",
               all[i].name, (unsigned long)all[i].compared, (unsigned long)all[i].mismatched, status);
    }
    printf("-----------------------------------------------------------------\n");
    printf("%-25s %12lu %12lu %10s\n",
           "TOTAL", (unsigned long)total_compared, (unsigned long)total_mismatched,
           (total_compared > 0 && total_mismatched == 0 && !any_empty) ? "PASS" : "FAIL");
    printf("=================================================================\n");

    if (any_empty || total_compared == 0 || total_mismatched > 0) {
        return 1;
    }
    return 0;
}
