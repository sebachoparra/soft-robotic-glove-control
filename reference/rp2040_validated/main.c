#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"

#include "ukf_shadow_rp2040.h"

/*
 * ================================================================
 * CONTROL_UNIFIED_PI_ADRC_V2_UKF4_PI_ADRC_FEEDBACK_TEST_RP2040
 * ================================================================
 *
 * Firmware de laboratorio SIN secuencia experimental embebida.
 *
 * El PC decide:
 *   - controlador externo: PI o ADRC;
 *   - referencia q_ref;
 *   - inicio/fin de registro;
 *   - marcas experimentales.
 *
 * El microcontrolador SOLO:
 *   - adquiere y filtra sensores;
 *   - ejecuta el controlador externo seleccionado;
 *   - ejecuta un UNICO lazo interno de presion compartido;
 *   - ejecuta FILL/HOLD/VENT;
 *   - aplica seguridad local;
 *   - transmite telemetria comun.
 *
 * Arquitectura:
 *
 *                       +-- PI_q gain scheduled --+
 *   q_ref --------------|                        |--> P_ref_raw
 *                       +-- LADRC + LESO + FF ---+
 *                                                   |
 *                                         MISMO rate limiter
 *                                                   |
 *                                                  P_ref
 *                                                   |
 *                                         MISMO PI presion
 *                                                   |
 *                                      MISMO FILL/HOLD/VENT
 *                                                   |
 *                                                  planta
 *
 * Comandos seriales:
 *
 *   START
 *   CTRL,PI          (feedback FLEX)
 *   CTRL,PI_UKF      (feedback UKF4 s_hat)
 *   CTRL,ADRC        (feedback FLEX + feedforward original)
 *   CTRL,ADRC_UKF    (feedback UKF4 s_hat + MISMO feedforward ADRC)
 *   CTRL,ADRC
 *   CTRL,PRESSURE
 *   SET_Q,<pct>
 *   SET_P,<kPa>
 *   MARK,<texto>
 *   START_LOG
 *   STOP_LOG
 *   STATUS
 *   CALIBRATE
 *   VENT
 *   HOLD
 *   END_RUN
 *   ABORT
 *
 * Reglas:
 *   - Tras calibrar, bomba y solenoides quedan APAGADOS.
 *   - CTRL,PI / CTRL,PI_UKF / CTRL,ADRC ENCIENDEN la bomba e inicializan el controlador.
 *   - Scheduling continuo K(s,eta), tau(s,eta), leyendo el UKF sin modificarlo.
 *   - END_RUN deshabilita control, ventea y APAGA TODO.
 *   - El protocolo experimental NO existe en este firmware.
 *   - CALIBRATE deshabilita control, ventea, recalibra y deja CTRL=NONE/OFF.
 *
 * Comparacion PI vs ADRC:
 *   El lazo desde P_ref hasta la planta es literalmente unico y comun.
 * ================================================================
 */


/* =================================================================
 * HARDWARE
 * ================================================================= */

#define PUMP_PIN       16
#define V1_PIN         17
#define V2_PIN         14

#define PRESSURE_PIN   26
#define PRESSURE_ADC   0

#define FLEX_PIN       28
#define FLEX_ADC       2


/* =================================================================
 * VALVULAS
 * ================================================================= */

#define VALVE_ON   1
#define VALVE_OFF  0

#define V1_FILL_LEVEL  VALVE_OFF
#define V2_FILL_LEVEL  VALVE_OFF

#define V1_HOLD_LEVEL  VALVE_OFF
#define V2_HOLD_LEVEL  VALVE_ON

#define V1_VENT_LEVEL  VALVE_ON
#define V2_VENT_LEVEL  VALVE_ON


/* =================================================================
 * ADQUISICION / LOG
 * ================================================================= */

#define SAMPLE_PERIOD_MS        10
#define LOG_PERIOD_MS           50
#define ADC_AVERAGE_SAMPLES      4

#define PRESSURE_EWMA_ALPHA     0.20f
#define FLEX_EWMA_ALPHA         0.20f


/* =================================================================
 * CALIBRACION ACTUAL
 * ================================================================= */

#define FLEX_SPAN_COUNTS        494.794f
#define FLEX_ZERO_SAMPLES       500

#define ADC_REF_VOLTAGE         3.3f
#define ADC_MAX_VALUE        4095.0f
#define DIVIDER_GAIN            1.545f
#define KPA_PER_VOLT           50.0f
#define PRESSURE_ZERO_SAMPLES   500


/* =================================================================
 * LIMITES P_ref COMUNES
 * ================================================================= */

#define PRESSURE_REF_MIN_KPA     0.0f
#define PRESSURE_REF_MAX_KPA   170.0f

/*
 * Gobernador comun PI/ADRC:
 *   Ts externo = 0.5 s
 *   UP   = +5 kPa por update
 *   DOWN = -6 kPa por update
 */
#define PREF_RATE_UP_KPA_S      10.0f
#define PREF_RATE_DOWN_KPA_S    12.0f


/* =================================================================
 * PI EXTERNO DE POSICION
 * ================================================================= */

#define OUTER_LOOP_MS            500
#define OUTER_TS_S             0.500f

#include "plant_schedule.h"


/* =================================================================
 * ADRC / LESO
 * ================================================================= */

#define B0_NOMINAL                 2.20f
#define B0_FALLBACK                B0_NOMINAL

#define B0_SMOOTH_TAU_S            0.50f

#define ESO_OMEGA_O                2.50f
#define ESO_BETA1                 (2.0f * ESO_OMEGA_O)
#define ESO_BETA2                 (ESO_OMEGA_O * ESO_OMEGA_O)
#define ESO_LOOP_MS                50
#define ESO_TS_S                   0.050f

#define LADRC_OMEGA_C              0.70f

#define LADRC_FF_MODEL_WEIGHT       0.30f
#define LADRC_FF_MAX_UP_KPA         8.0f
#define LADRC_FF_MAX_DOWN_KPA       5.0f
#define LADRC_FF_FULL_ERROR_PCT    10.0f
#define LADRC_FF_ZERO_ERROR_PCT     2.0f

/* =================================================================
 * PI INTERNO DE PRESION - UNICO PARA AMBOS
 * ================================================================= */

#define KP_PRESSURE               0.060f
#define KI_PRESSURE               0.025f

#define PRESSURE_LOOP_MS           100
#define PRESSURE_TS_S             0.100f

#define PRESSURE_I_MIN           -1.0f
#define PRESSURE_I_MAX            1.0f

#define PRESSURE_DEADBAND_KPA      1.0f


/* =================================================================
 * PULSOS / REVERSAL - UNICO PARA AMBOS
 * ================================================================= */

#define FILL_MIN_MS                 5
#define FILL_MAX_MS                80

#define VENT_MIN_MS                10
#define VENT_MAX_MS               100

#define REVERSAL_LOCKOUT_MS        200


/* =================================================================
 * SEGURIDAD
 * ================================================================= */

#define HARD_PRESSURE_KPA         200.0f

#define INITIAL_VENT_MS         10000
#define RECAL_VENT_MS           10000
#define FINAL_VENT_MS            8000


/* =================================================================
 * TIPOS
 * ================================================================= */

typedef enum
{
    STATE_UNKNOWN = 99,
    STATE_VENT = -1,
    STATE_HOLD = 0,
    STATE_FILL = 1,
    STATE_OFF = 2
} PneumaticState;

typedef enum
{
    CTRL_NONE = 0,
    CTRL_PI = 1,          /* feedback FLEX normalizado */
    CTRL_ADRC = 2,
    CTRL_PRESSURE = 3,
    CTRL_PI_UKF = 4,      /* feedback UKF4: s_hat */
    CTRL_ADRC_UKF = 5     /* LESO/LADRC feedback UKF4: s_hat */
} ControllerMode;

typedef struct
{
    float error;
    float p_term;
    float i_term;
    float output_unsat;
    float output;
} PressurePIResult;

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
} PositionPIResult;

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
} LADRCResult;


/* =================================================================
 * GLOBALES - PLANTA / SENSORES
 * ================================================================= */

static volatile PneumaticState pneumatic_state = STATE_UNKNOWN;
static ControllerMode controller_mode = CTRL_NONE;

static bool control_enabled = false;
static bool logging_enabled = false;
static bool command_abort = false;

static uint32_t global_start_ms = 0;

static float pressure_zero_raw = 0.0f;
static float pressure_zero_vout = 0.0f;
static float flex_zero_raw = 0.0f;

static float pressure_filtered = 0.0f;
static float flex_filtered_raw = 0.0f;

static float latest_pressure_raw = 0.0f;
static float latest_pressure_kpa = 0.0f;
static float latest_pressure_vout = 0.0f;

static float latest_flex_raw = 0.0f;
static float latest_position_unclipped = 0.0f;
static float latest_position_pct = 0.0f;

static float position_ref_pct = 0.0f;
static float previous_q_ref_pct = 0.0f;

static float pressure_ref_kpa = 0.0f;


/* =================================================================
 * GLOBALES - PI EXTERNO
 * ================================================================= */

static uint8_t active_gain_region_id = 0; /* 0 = continuous scheduling */
static PlantSchedule scheduled_plant = {0};
static bool schedule_input_valid = false;
static float active_kp_position = PLANT_P1 * 1.906f / 0.818f;
static float active_ki_position = PLANT_P1 / 0.818f;
static float previous_position_error = 0.0f;

static PositionPIResult last_position_pi = {0};


/* =================================================================
 * GLOBALES - ADRC
 * ================================================================= */

static float eso_z1_pct = 0.0f;
static float eso_z2_pct_s = 0.0f;

static float active_b0 = B0_FALLBACK;
static float target_b0 = B0_FALLBACK;

static float adrc_model_k = 0.0f;
static float adrc_model_tau_s = 0.0f;
static float adrc_b0_identified = B0_FALLBACK;
static bool adrc_model_valid = false;

static float step_ff_kpa = 0.0f;
static float ff_gamma_state = 0.0f;

static LADRCResult last_ladrc = {0};


/* =================================================================
 * GLOBALES - PI PRESION / PULSOS
 * ================================================================= */

static float pressure_integral = 0.0f;
static PressurePIResult last_pressure_pi = {0};

static volatile bool pulse_active = false;
static volatile uint32_t pulse_end_ms = 0;
static uint32_t last_pulse_ms = 0;

static volatile alarm_id_t pulse_alarm_id = 0;
static volatile bool pulse_alarm_armed = false;
static volatile uint32_t pulse_alarm_fail_count = 0;

static PneumaticState last_pulse_direction = STATE_HOLD;
static PneumaticState pending_reversal_direction = STATE_HOLD;
static uint32_t reversal_block_until_ms = 0;


/* =================================================================
 * DIAGNOSTICO
 * ================================================================= */

static bool outer_updated_since_log = false;
static bool eso_updated_since_log = false;
static bool pressure_updated_since_log = false;


/* =================================================================
 * UTILIDADES
 * ================================================================= */

static uint32_t millis_now(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static uint32_t experiment_time_ms(uint32_t now_ms)
{
    if (global_start_ms == 0u)
        return 0u;

    return now_ms - global_start_ms;
}

static float clampf_local(
    float x,
    float xmin,
    float xmax
)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static const char *state_name(PneumaticState state)
{
    switch (state)
    {
        case STATE_FILL: return "FILL";
        case STATE_HOLD: return "HOLD";
        case STATE_VENT: return "VENT";
        case STATE_OFF: return "OFF";
        default: return "UNKNOWN";
    }
}

static const char *controller_name(ControllerMode mode)
{
    switch (mode)
    {
        case CTRL_PI: return "PI";
        case CTRL_PI_UKF: return "PI_UKF";
        case CTRL_ADRC: return "ADRC";
        case CTRL_ADRC_UKF: return "ADRC_UKF";
        case CTRL_PRESSURE: return "PRESSURE";
        default: return "NONE";
    }
}

static bool ukf4_feedback_available(void)
{
    const UKFShadowTelemetry *u = ukf_shadow_get();

    return
        u->valid
        && u->reference_valid
        && u->theta_valid
        && isfinite(u->s_hat);
}

static float active_position_feedback_pct(void)
{
    if (controller_mode == CTRL_PI_UKF || controller_mode == CTRL_ADRC_UKF)
    {
        const UKFShadowTelemetry *u = ukf_shadow_get();

        if (ukf4_feedback_available())
        {
            return clampf_local(
                u->s_hat,
                0.0f,
                100.0f
            );
        }
    }

    return latest_position_pct;
}

static const char *active_position_feedback_name(void)
{
    return
        (controller_mode == CTRL_PI_UKF || controller_mode == CTRL_ADRC_UKF)
        ? "UKF4_S"
        : "FLEX_Q";
}


/* =================================================================
 * NEUMATICA
 * ================================================================= */

static void set_state(PneumaticState state)
{
    pneumatic_state = state;

    switch (state)
    {
        case STATE_FILL:
            gpio_put(V1_PIN, V1_FILL_LEVEL);
            gpio_put(V2_PIN, V2_FILL_LEVEL);
            break;

        case STATE_HOLD:
            gpio_put(V1_PIN, V1_HOLD_LEVEL);
            gpio_put(V2_PIN, V2_HOLD_LEVEL);
            break;

        case STATE_VENT:
            gpio_put(V1_PIN, V1_VENT_LEVEL);
            gpio_put(V2_PIN, V2_VENT_LEVEL);
            break;

        default:
            break;
    }
}

static void pump_on(void)
{
    gpio_put(PUMP_PIN, 1);
}

static void pump_off(void)
{
    gpio_put(PUMP_PIN, 0);
}

static void all_outputs_off(void)
{
    pump_off();
    gpio_put(V1_PIN, 0);
    gpio_put(V2_PIN, 0);
    pneumatic_state = STATE_OFF;
}


/* =================================================================
 * ADC / CONVERSIONES
 * ================================================================= */

static float read_adc_average(uint channel, int samples)
{
    adc_select_input(channel);
    sleep_us(10);

    uint32_t sum = 0;

    for (int i = 0; i < samples; i++)
    {
        sum += adc_read();
        sleep_us(200);
    }

    return (float)sum / (float)samples;
}

static float pressure_raw_to_vout(float raw)
{
    float vadc =
        raw * ADC_REF_VOLTAGE
        / ADC_MAX_VALUE;

    return vadc * DIVIDER_GAIN;
}

static float pressure_raw_to_kpa(float raw)
{
    return
        (pressure_raw_to_vout(raw) - pressure_zero_vout)
        * KPA_PER_VOLT;
}

static float flex_raw_to_position_unclipped(float raw)
{
    return
        100.0f
        * (flex_zero_raw - raw)
        / FLEX_SPAN_COUNTS;
}

static float ewma(
    float x,
    float previous,
    float alpha
)
{
    return
        alpha * x
        + (1.0f - alpha) * previous;
}


/* =================================================================
 * ALARMA DE PULSO
 * ================================================================= */

static int64_t pulse_alarm_callback(
    alarm_id_t id,
    void *user_data
)
{
    (void)id;
    (void)user_data;

    pulse_active = false;
    pulse_alarm_armed = false;
    pulse_alarm_id = 0;

    set_state(STATE_HOLD);

    return 0;
}

static void cancel_active_pulse_alarm(void)
{
    if (pulse_alarm_armed)
    {
        cancel_alarm(pulse_alarm_id);

        pulse_alarm_armed = false;
        pulse_alarm_id = 0;
    }

    pulse_active = false;
}


/* =================================================================
 * REINICIALIZACION / CALIBRACION
 * ================================================================= */

static void reset_pressure_actuation_state(void)
{
    cancel_active_pulse_alarm();

    pressure_integral = 0.0f;
    memset(&last_pressure_pi, 0, sizeof(last_pressure_pi));

    last_pulse_ms = 0;
    last_pulse_direction = STATE_HOLD;
    pending_reversal_direction = STATE_HOLD;
    reversal_block_until_ms = 0u;

    set_state(STATE_HOLD);
}

static void reset_outer_diagnostics(void)
{
    memset(&last_position_pi, 0, sizeof(last_position_pi));
    memset(&last_ladrc, 0, sizeof(last_ladrc));

    outer_updated_since_log = false;
    eso_updated_since_log = false;
}

static void calibrate_pressure_zero(void)
{
    printf("#CALIBRATING_PRESSURE_ZERO\n");

    pressure_zero_raw =
        read_adc_average(
            PRESSURE_ADC,
            PRESSURE_ZERO_SAMPLES
        );

    pressure_zero_vout =
        pressure_raw_to_vout(
            pressure_zero_raw
        );

    printf(
        "#pressure_zero_raw=%.3f\n",
        pressure_zero_raw
    );

    printf(
        "#pressure_zero_vout=%.5f\n",
        pressure_zero_vout
    );
}

static void calibrate_flex_zero(void)
{
    printf("#CALIBRATING_FLEX_ZERO\n");

    flex_zero_raw =
        read_adc_average(
            FLEX_ADC,
            FLEX_ZERO_SAMPLES
        );

    printf(
        "#flex_zero_raw=%.3f\n",
        flex_zero_raw
    );

    printf(
        "#flex_span_counts=%.3f\n",
        FLEX_SPAN_COUNTS
    );

    printf(
        "#flex_estimated_100_raw=%.3f\n",
        flex_zero_raw - FLEX_SPAN_COUNTS
    );
}

static void initialize_filters_from_current_sensors(void)
{
    latest_pressure_raw =
        read_adc_average(
            PRESSURE_ADC,
            ADC_AVERAGE_SAMPLES
        );

    latest_pressure_vout =
        pressure_raw_to_vout(
            latest_pressure_raw
        );

    latest_pressure_kpa =
        pressure_raw_to_kpa(
            latest_pressure_raw
        );

    pressure_filtered =
        latest_pressure_kpa;

    latest_flex_raw =
        read_adc_average(
            FLEX_ADC,
            ADC_AVERAGE_SAMPLES
        );

    flex_filtered_raw =
        latest_flex_raw;

    latest_position_unclipped =
        flex_raw_to_position_unclipped(
            latest_flex_raw
        );

    latest_position_pct =
        clampf_local(
            latest_position_unclipped,
            0.0f,
            100.0f
        );
}

static void initialize_pi_bumpless(void)
{
    previous_position_error =
        position_ref_pct
        - active_position_feedback_pct();

    memset(
        &last_position_pi,
        0,
        sizeof(last_position_pi)
    );

    last_position_pi.error =
        previous_position_error;

    last_position_pi.previous_error =
        previous_position_error;

    last_position_pi.pressure_ref_previous_kpa =
        pressure_ref_kpa;

    last_position_pi.pressure_ref_raw_kpa =
        pressure_ref_kpa;

    last_position_pi.pressure_ref_rate_limited_kpa =
        pressure_ref_kpa;

    last_position_pi.pressure_ref_kpa =
        pressure_ref_kpa;
}

static void initialize_adrc_bumpless(void)
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
    active_b0 = target_b0;

    if (!isfinite(active_b0) || active_b0 <= 1e-5f)
        active_b0 = B0_FALLBACK;

    eso_z1_pct =
        active_position_feedback_pct();

    eso_z2_pct_s =
        -active_b0
        * pressure_ref_kpa;

    memset(
        &last_ladrc,
        0,
        sizeof(last_ladrc)
    );

    last_ladrc.position_error_pct =
        position_ref_pct
        - eso_z1_pct;

    last_ladrc.equivalent_pressure_kpa =
        pressure_ref_kpa;

    last_ladrc.pressure_ref_previous_kpa =
        pressure_ref_kpa;

    last_ladrc.pressure_ref_raw_kpa =
        pressure_ref_kpa;

    last_ladrc.pressure_ref_rate_limited_kpa =
        pressure_ref_kpa;

    last_ladrc.pressure_ref_kpa =
        pressure_ref_kpa;
}

static void reset_controller_state_to_safe_none(void)
{
    controller_mode = CTRL_NONE;
    control_enabled = false;

    position_ref_pct =
        latest_position_pct;

    previous_q_ref_pct =
        position_ref_pct;

    pressure_ref_kpa =
        clampf_local(
            pressure_filtered,
            PRESSURE_REF_MIN_KPA,
            PRESSURE_REF_MAX_KPA
        );

    active_gain_region_id = 0;
    active_kp_position = PLANT_P1 * 1.906f / 0.818f;
    active_ki_position = PLANT_P1 / 0.818f;
    schedule_input_valid = false;

    target_b0 = B0_FALLBACK;
    active_b0 = B0_FALLBACK;

    adrc_model_k = 0.0f;
    adrc_model_tau_s = 0.0f;
    adrc_b0_identified = B0_FALLBACK;
    adrc_model_valid = false;

    step_ff_kpa = 0.0f;
    ff_gamma_state = 0.0f;

    initialize_pi_bumpless();
    initialize_adrc_bumpless();
    reset_pressure_actuation_state();
    reset_outer_diagnostics();
}

static void perform_full_calibration(uint32_t vent_ms)
{
    control_enabled = false;
    controller_mode = CTRL_NONE;
    logging_enabled = false;

    cancel_active_pulse_alarm();

    pump_off();
    set_state(STATE_VENT);

    printf(
        "#CALIBRATION_START,T_MS=%lu,VENT_MS=%lu\n",
        (unsigned long)experiment_time_ms(millis_now()),
        (unsigned long)vent_ms
    );

    sleep_ms(vent_ms);

    calibrate_pressure_zero();
    calibrate_flex_zero();

    initialize_filters_from_current_sensors();

    /*
     * Recalibrate the UKF IMU reference every time the system is fully
     * vented/calibrated. The UKF4 normally runs in shadow mode, but
     * CTRL,PI_UKF can use s_hat as the outer PI feedback for testing.
     */
    ukf_shadow_calibrate_reference(
        flex_zero_raw
    );

    pressure_ref_kpa =
        clampf_local(
            pressure_filtered,
            PRESSURE_REF_MIN_KPA,
            PRESSURE_REF_MAX_KPA
        );

    position_ref_pct =
        latest_position_pct;

    previous_q_ref_pct =
        position_ref_pct;

    reset_pressure_actuation_state();
    reset_controller_state_to_safe_none();

    all_outputs_off();

    printf(
        "#CALIBRATION_DONE,T_MS=%lu,Q=%.3f,P=%.3f,PREF=%.3f,OUTPUTS=OFF\n",
        (unsigned long)experiment_time_ms(millis_now()),
        latest_position_pct,
        pressure_filtered,
        pressure_ref_kpa
    );
}


/* =================================================================
 * SCHEDULING PI
 * ================================================================= */

/* Read-only consumer of UKF outputs in every position controller mode.
 * Invalid UKF: retain last schedule; existing feedback fault handling is unchanged.
 */
static void update_plant_schedule(void)
{
    const UKFShadowTelemetry *u = ukf_shadow_get();
    PlantSchedule next;
    schedule_input_valid = ukf4_feedback_available()
        && plant_schedule_eval(u->s_hat, u->eta_hat, &next);
    if (!schedule_input_valid) return;
    scheduled_plant = next;
    active_gain_region_id = 0;
    active_kp_position = next.kp;
    active_ki_position = next.ki;
    adrc_model_k = next.k;
    adrc_model_tau_s = next.tau;
    adrc_b0_identified = next.b0;
    adrc_model_valid = true;
    //target_b0 = next.b0;
    
    /* Scheduling continuo con la regularizacion original. */
    target_b0 =
        0.80f * B0_NOMINAL
        + 0.20f * next.b0;
    
    /* Do not reset PI output/error history, LESO, or feedforward here. */
}

static float compute_adrc_feedforward(
    float q_from,
    float q_to,
    float model_k,
    bool valid_nominal
)
{
    if (
        !valid_nominal
        || !isfinite(model_k)
        || fabsf(model_k) < 1e-5f
    )
    {
        return 0.0f;
    }

    float ff =
        LADRC_FF_MODEL_WEIGHT
        * (q_to - q_from)
        / model_k;

    if (ff > LADRC_FF_MAX_UP_KPA)
        ff = LADRC_FF_MAX_UP_KPA;

    if (ff < -LADRC_FF_MAX_DOWN_KPA)
        ff = -LADRC_FF_MAX_DOWN_KPA;

    return ff;
}

static void update_active_b0(void)
{
    const float alpha =
        ESO_TS_S
        / (B0_SMOOTH_TAU_S + ESO_TS_S);

    const float old_b0 =
        active_b0;

    float new_b0 =
        old_b0
        + alpha
        * (target_b0 - old_b0);

    if (
        !isfinite(new_b0)
        || new_b0 <= 1e-5f
    )
    {
        new_b0 = B0_FALLBACK;
    }

    if (
        fabsf(target_b0 - new_b0)
        < 1e-5f
    )
    {
        new_b0 = target_b0;
    }

    /*
     * Conserva:
     *     q_dot_hat = z2 + b0 * P_ref
     */
    eso_z2_pct_s +=
        (old_b0 - new_b0)
        * pressure_ref_kpa;

    active_b0 =
        new_b0;
}


/* =================================================================
 * RATE LIMITER COMUN
 * ================================================================= */

static float apply_common_pref_limits(
    float raw_pref,
    float previous_pref,
    bool *rate_limited,
    bool *saturated
)
{
    const float max_up_step =
        PREF_RATE_UP_KPA_S
        * OUTER_TS_S;

    const float max_down_step =
        PREF_RATE_DOWN_KPA_S
        * OUTER_TS_S;

    float limited =
        raw_pref;

    *rate_limited = false;
    *saturated = false;

    if (
        limited
        > previous_pref + max_up_step
    )
    {
        limited =
            previous_pref + max_up_step;

        *rate_limited = true;
    }
    else if (
        limited
        < previous_pref - max_down_step
    )
    {
        limited =
            previous_pref - max_down_step;

        *rate_limited = true;
    }

    float final_pref =
        clampf_local(
            limited,
            PRESSURE_REF_MIN_KPA,
            PRESSURE_REF_MAX_KPA
        );

    *saturated =
        fabsf(final_pref - limited)
        > 1e-6f;

    return final_pref;
}


/* =================================================================
 * PI EXTERNO
 * ================================================================= */

static PositionPIResult position_pi_update(
    float q_ref,
    float q
)
{
    PositionPIResult r;
    memset(&r, 0, sizeof(r));

    r.error =
        q_ref - q;

    r.previous_error =
        previous_position_error;

    r.p_increment_kpa =
        active_kp_position
        * (r.error - previous_position_error);

    r.i_increment_kpa =
        active_ki_position
        * OUTER_TS_S
        * r.error;

    r.delta_pref_kpa =
        r.p_increment_kpa
        + r.i_increment_kpa;

    r.pressure_ref_previous_kpa =
        pressure_ref_kpa;

    r.pressure_ref_raw_kpa =
        pressure_ref_kpa
        + r.delta_pref_kpa;

    bool rate_limited = false;
    bool saturated = false;

    float final_pref =
        apply_common_pref_limits(
            r.pressure_ref_raw_kpa,
            pressure_ref_kpa,
            &rate_limited,
            &saturated
        );

    /*
     * Reconstruye el valor intermedio solo para diagnostico.
     */
    const float max_up_step =
        PREF_RATE_UP_KPA_S * OUTER_TS_S;

    const float max_down_step =
        PREF_RATE_DOWN_KPA_S * OUTER_TS_S;

    r.pressure_ref_rate_limited_kpa =
        r.pressure_ref_raw_kpa;

    if (
        r.pressure_ref_rate_limited_kpa
        > pressure_ref_kpa + max_up_step
    )
    {
        r.pressure_ref_rate_limited_kpa =
            pressure_ref_kpa + max_up_step;
    }
    else if (
        r.pressure_ref_rate_limited_kpa
        < pressure_ref_kpa - max_down_step
    )
    {
        r.pressure_ref_rate_limited_kpa =
            pressure_ref_kpa - max_down_step;
    }

    r.rate_limited =
        rate_limited;

    r.saturated =
        saturated;

    pressure_ref_kpa =
        final_pref;

    r.pressure_ref_kpa =
        pressure_ref_kpa;

    previous_position_error =
        r.error;

    return r;
}


/* =================================================================
 * LESO / ADRC
 * ================================================================= */

static void leso_update(
    float pressure_input_kpa,
    float position_measured_pct
)
{
    float error =
        position_measured_pct
        - eso_z1_pct;

    float z1_dot =
        eso_z2_pct_s
        + active_b0
        * pressure_input_kpa
        + ESO_BETA1
        * error;

    float z2_dot =
        ESO_BETA2
        * error;

    eso_z1_pct +=
        ESO_TS_S
        * z1_dot;

    eso_z2_pct_s +=
        ESO_TS_S
        * z2_dot;
}

static LADRCResult ladrc_update(
    float q_ref
)
{
    LADRCResult r;
    memset(&r, 0, sizeof(r));

    r.position_error_pct =
        q_ref - eso_z1_pct;

    r.equivalent_pressure_kpa =
        -eso_z2_pct_s
        / active_b0;

    r.feedback_correction_kpa =
        (LADRC_OMEGA_C / active_b0)
        * r.position_error_pct;

    r.feedforward_step_kpa =
        step_ff_kpa;

    const float abs_error =
        fabsf(
            r.position_error_pct
        );

    const bool ff_direction_ok =
        (
            step_ff_kpa > 0.0f
            && r.position_error_pct > 0.0f
        )
        || (
            step_ff_kpa < 0.0f
            && r.position_error_pct < 0.0f
        );

    if (
        fabsf(step_ff_kpa)
        <= 1e-6f
    )
    {
        ff_gamma_state = 0.0f;
    }
    else if (!ff_direction_ok)
    {
        ff_gamma_state = 0.0f;
    }
    else
    {
        float gamma_candidate = 0.0f;

        if (
            abs_error
            >= LADRC_FF_FULL_ERROR_PCT
        )
        {
            gamma_candidate = 1.0f;
        }
        else if (
            abs_error
            > LADRC_FF_ZERO_ERROR_PCT
        )
        {
            gamma_candidate =
                (
                    abs_error
                    - LADRC_FF_ZERO_ERROR_PCT
                )
                /
                (
                    LADRC_FF_FULL_ERROR_PCT
                    - LADRC_FF_ZERO_ERROR_PCT
                );
        }

        gamma_candidate =
            clampf_local(
                gamma_candidate,
                0.0f,
                1.0f
            );

        /*
         * FF monotono por referencia:
         * solo puede decrecer hasta el siguiente SET_Q.
         */
        if (
            gamma_candidate
            < ff_gamma_state
        )
        {
            ff_gamma_state =
                gamma_candidate;
        }
    }

    r.feedforward_gamma =
        ff_gamma_state;

    r.feedforward_applied_kpa =
        ff_gamma_state
        * step_ff_kpa;

    r.pressure_ref_previous_kpa =
        pressure_ref_kpa;

    r.pressure_ref_raw_kpa =
        r.equivalent_pressure_kpa
        + r.feedback_correction_kpa
        + r.feedforward_applied_kpa;

    bool rate_limited = false;
    bool saturated = false;

    float final_pref =
        apply_common_pref_limits(
            r.pressure_ref_raw_kpa,
            pressure_ref_kpa,
            &rate_limited,
            &saturated
        );

    const float max_up_step =
        PREF_RATE_UP_KPA_S * OUTER_TS_S;

    const float max_down_step =
        PREF_RATE_DOWN_KPA_S * OUTER_TS_S;

    r.pressure_ref_rate_limited_kpa =
        r.pressure_ref_raw_kpa;

    if (
        r.pressure_ref_rate_limited_kpa
        > pressure_ref_kpa + max_up_step
    )
    {
        r.pressure_ref_rate_limited_kpa =
            pressure_ref_kpa + max_up_step;
    }
    else if (
        r.pressure_ref_rate_limited_kpa
        < pressure_ref_kpa - max_down_step
    )
    {
        r.pressure_ref_rate_limited_kpa =
            pressure_ref_kpa - max_down_step;
    }

    r.rate_limited =
        rate_limited;

    r.saturated =
        saturated;

    pressure_ref_kpa =
        final_pref;

    r.pressure_ref_kpa =
        pressure_ref_kpa;

    return r;
}


/* =================================================================
 * PI INTERNO DE PRESION - BLOQUE COMUN
 * ================================================================= */

static PressurePIResult pressure_pi_update(
    float reference,
    float pressure
)
{
    PressurePIResult r;

    r.error =
        reference - pressure;

    r.p_term =
        KP_PRESSURE
        * r.error;

    if (
        fabsf(r.error)
        <= PRESSURE_DEADBAND_KPA
    )
    {
        r.i_term =
            pressure_integral;

        r.output_unsat = 0.0f;
        r.output = 0.0f;

        return r;
    }

    float candidate_integral =
        pressure_integral
        + KI_PRESSURE
        * r.error
        * PRESSURE_TS_S;

    candidate_integral =
        clampf_local(
            candidate_integral,
            PRESSURE_I_MIN,
            PRESSURE_I_MAX
        );

    float candidate_magnitude_unsat = 0.0f;

    if (r.error > 0.0f)
    {
        candidate_magnitude_unsat =
            r.p_term
            + fmaxf(
                candidate_integral,
                0.0f
            );
    }
    else
    {
        candidate_magnitude_unsat =
            -r.p_term
            + fmaxf(
                -candidate_integral,
                0.0f
            );
    }

    bool saturating_effective_command =
        candidate_magnitude_unsat
        > 1.0f;

    if (!saturating_effective_command)
    {
        pressure_integral =
            candidate_integral;
    }

    r.i_term =
        pressure_integral;

    if (r.error > 0.0f)
    {
        float magnitude_unsat =
            r.p_term
            + fmaxf(
                pressure_integral,
                0.0f
            );

        r.output_unsat =
            magnitude_unsat;

        r.output =
            clampf_local(
                magnitude_unsat,
                0.0f,
                1.0f
            );
    }
    else
    {
        float magnitude_unsat =
            -r.p_term
            + fmaxf(
                -pressure_integral,
                0.0f
            );

        r.output_unsat =
            -magnitude_unsat;

        r.output =
            -clampf_local(
                magnitude_unsat,
                0.0f,
                1.0f
            );
    }

    return r;
}


/* =================================================================
 * PULSOS / ACTUACION - BLOQUE COMUN
 * ================================================================= */

static uint32_t magnitude_to_pulse(
    float magnitude,
    uint32_t min_ms,
    uint32_t max_ms
)
{
    magnitude =
        clampf_local(
            magnitude,
            0.0f,
            1.0f
        );

    return
        (uint32_t)(
            (float)min_ms
            + magnitude
            * (
                (float)max_ms
                - (float)min_ms
            )
        );
}

static void start_pulse(
    PneumaticState state,
    uint32_t pulse_ms,
    uint32_t now_ms
)
{
    if (pulse_alarm_armed)
        cancel_active_pulse_alarm();

    set_state(state);

    pulse_active = true;
    pulse_end_ms =
        now_ms + pulse_ms;

    alarm_id_t id =
        add_alarm_in_ms(
            pulse_ms,
            pulse_alarm_callback,
            NULL,
            true
        );

    if (id < 0)
    {
        pulse_alarm_fail_count++;

        pulse_active = false;
        pulse_alarm_armed = false;
        pulse_alarm_id = 0;

        set_state(STATE_HOLD);
        return;
    }

    pulse_alarm_id = id;
    pulse_alarm_armed = true;
}

static void update_active_pulse(
    uint32_t now_ms
)
{
    if (!pulse_active)
        return;

    if (
        (int32_t)(
            now_ms - pulse_end_ms
        ) >= 0
    )
    {
        cancel_active_pulse_alarm();
        set_state(STATE_HOLD);
    }
}

static void execute_pressure_control(
    float reference,
    float pressure,
    uint32_t now_ms
)
{
    last_pressure_pi =
        pressure_pi_update(
            reference,
            pressure
        );

    last_pulse_ms = 0;

    if (pulse_active)
        return;

    if (
        fabsf(last_pressure_pi.error)
        <= PRESSURE_DEADBAND_KPA
    )
    {
        pending_reversal_direction =
            STATE_HOLD;

        set_state(STATE_HOLD);
        return;
    }

    PneumaticState requested_state =
        (
            last_pressure_pi.error
            > 0.0f
        )
        ? STATE_FILL
        : STATE_VENT;

    if (
        pending_reversal_direction
        != STATE_HOLD
    )
    {
        if (
            requested_state
            != pending_reversal_direction
        )
        {
            pending_reversal_direction =
                requested_state;

            reversal_block_until_ms =
                now_ms
                + REVERSAL_LOCKOUT_MS;

            set_state(STATE_HOLD);
            return;
        }

        if (
            (int32_t)(
                now_ms
                - reversal_block_until_ms
            ) < 0
        )
        {
            set_state(STATE_HOLD);
            return;
        }

        pending_reversal_direction =
            STATE_HOLD;
    }
    else if (
        last_pulse_direction
        != STATE_HOLD
        && requested_state
        != last_pulse_direction
    )
    {
        pending_reversal_direction =
            requested_state;

        reversal_block_until_ms =
            now_ms
            + REVERSAL_LOCKOUT_MS;

        set_state(STATE_HOLD);

        printf(
            "#PRESSURE_REVERSAL_LOCKOUT,"
            "T_MS=%lu,FROM=%s,TO=%s,MS=%d\n",
            (unsigned long)experiment_time_ms(now_ms),
            state_name(last_pulse_direction),
            state_name(requested_state),
            REVERSAL_LOCKOUT_MS
        );

        return;
    }

    if (
        requested_state
        == STATE_FILL
    )
    {
        uint32_t pulse =
            magnitude_to_pulse(
                last_pressure_pi.output,
                FILL_MIN_MS,
                FILL_MAX_MS
            );

        last_pulse_ms =
            pulse;

        last_pulse_direction =
            STATE_FILL;

        start_pulse(
            STATE_FILL,
            pulse,
            now_ms
        );

        return;
    }

    uint32_t pulse =
        magnitude_to_pulse(
            -last_pressure_pi.output,
            VENT_MIN_MS,
            VENT_MAX_MS
        );

    last_pulse_ms =
        pulse;

    last_pulse_direction =
        STATE_VENT;

    start_pulse(
        STATE_VENT,
        pulse,
        now_ms
    );
}


/* =================================================================
 * CAMBIO DE REFERENCIA / CONTROLADOR
 * ================================================================= */

static void apply_reference_change(
    float new_q_ref
)
{
    new_q_ref =
        clampf_local(
            new_q_ref,
            0.0f,
            100.0f
        );

    float old_q_ref =
        position_ref_pct;

    previous_q_ref_pct =
        old_q_ref;

    position_ref_pct =
        new_q_ref;

    update_plant_schedule();
    if (fabsf(new_q_ref - old_q_ref) > 0.001f)
    {
        /* Keep existing step feedforward envelope and limits. Only K's
         * source changes; the event remains latched until the next SET_Q. */
        step_ff_kpa = compute_adrc_feedforward(old_q_ref, new_q_ref,
            adrc_model_k, schedule_input_valid);
        ff_gamma_state = fabsf(step_ff_kpa) > 1e-6f ? 1.0f : 0.0f;
    }

    printf(
        "#QREF_SET,T_MS=%lu,CTRL=%s,OLD=%.3f,NEW=%.3f,"
        "PI_GAIN_ID=%u,B0_TARGET=%.6f,FF_KPA=%.6f\n",
        (unsigned long)experiment_time_ms(millis_now()),
        controller_name(controller_mode),
        old_q_ref,
        position_ref_pct,
        (unsigned int)active_gain_region_id,
        target_b0,
        step_ff_kpa
    );
}

static void select_controller_mode(
    ControllerMode new_mode
)
{
    controller_mode =
        new_mode;

    control_enabled =
        new_mode != CTRL_NONE;

    /*
     * P_ref se conserva para no introducir una discontinuidad
     * artificial en el lazo interno.
     */
    pressure_ref_kpa =
        clampf_local(
            pressure_ref_kpa,
            PRESSURE_REF_MIN_KPA,
            PRESSURE_REF_MAX_KPA
        );

    if (
        new_mode == CTRL_PI
        || new_mode == CTRL_PI_UKF
    )
    {
        update_plant_schedule();

        initialize_pi_bumpless();
    }
    else if (
        (new_mode == CTRL_ADRC || new_mode == CTRL_ADRC_UKF)
    )
    {
        /*
         * Cambiar de controlador NO debe disparar un feedforward
         * perteneciente a una referencia antigua. El FF solo vuelve
         * a habilitarse con un nuevo SET_Q.
         */
        ff_gamma_state = 0.0f;

        /*
         * Use the latest shared plant before initializing the observer.
         */
        update_plant_schedule();
        initialize_adrc_bumpless();
    }
    else if (
        new_mode
        == CTRL_PRESSURE
    )
    {
        /*
         * MODO DE IDENTIFICACION:
         *   - lazo EXTERNO de posicion abierto;
         *   - lazo INTERNO de presion permanece cerrado;
         *   - FLEX y UKF solo se registran, no gobiernan P_ref.
         *
         * Al entrar al modo, inicializamos P_ref en la presion actual
         * para evitar un salto artificial antes del primer SET_P.
         */
        position_ref_pct =
            latest_position_pct;

        previous_q_ref_pct =
            position_ref_pct;

        pressure_ref_kpa =
            clampf_local(
                pressure_filtered,
                PRESSURE_REF_MIN_KPA,
                PRESSURE_REF_MAX_KPA
            );

        ff_gamma_state = 0.0f;

        reset_pressure_actuation_state();
        reset_outer_diagnostics();
    }

    pump_on();
    set_state(STATE_HOLD);

    printf(
        "#CTRL_SET,T_MS=%lu,CTRL=%s,QREF=%.3f,Q=%.3f,"
        "PREF=%.3f,P=%.3f\n",
        (unsigned long)experiment_time_ms(millis_now()),
        controller_name(controller_mode),
        position_ref_pct,
        latest_position_pct,
        pressure_ref_kpa,
        pressure_filtered
    );
}


/* =================================================================
 * SERIAL
 * ================================================================= */

/*
 * START_LOG vuelve a emitir el header. Esto hace robusta la captura
 * aunque el PC se conecte despues del arranque/calibracion.
 */
static void print_csv_header(void);

static void print_status(void)
{
    printf(
        "#STATUS,T_MS=%lu,CTRL=%s,ENABLED=%d,LOG=%d,"
        "QREF=%.3f,Q=%.3f,QFB=%.3f,QFB_SRC=%s,PREF=%.3f,P=%.3f,"
        "STATE=%s,PI_GAIN_ID=%u,B0_ACTIVE=%.6f,B0_TARGET=%.6f,"
        "FF_KPA=%.6f\n",
        (unsigned long)experiment_time_ms(millis_now()),
        controller_name(controller_mode),
        control_enabled ? 1 : 0,
        logging_enabled ? 1 : 0,
        position_ref_pct,
        latest_position_pct,
        active_position_feedback_pct(),
        active_position_feedback_name(),
        pressure_ref_kpa,
        pressure_filtered,
        state_name(pneumatic_state),
        (unsigned int)active_gain_region_id,
        active_b0,
        target_b0,
        step_ff_kpa
    );

    const UKFShadowTelemetry *u =
        ukf_shadow_get();

    printf(
        "#UKF4_STATUS,VALID=%d,REF=%d,S=%.3f,V=%.3f,"
        "ETA=%.5f,BF=%.3f,THETA=%.3f,FLEX=%.3f,"
        "NIS=%.5f,RHO_ETABF=%.5f,BNO1=%d,BNO2=%d,"
        "EXEC_US=%lu,MAX_US=%lu\n",
        u->valid ? 1 : 0,
        u->reference_valid ? 1 : 0,
        u->s_hat,
        u->v_hat,
        u->eta_hat,
        u->b_f_hat,
        u->theta_deg,
        u->flex_aligned,
        u->nis,
        u->rho_eta_b_f,
        u->bno1_ok ? 1 : 0,
        u->bno2_ok ? 1 : 0,
        (unsigned long)u->exec_us,
        (unsigned long)u->max_exec_us
    );
}

static void process_command_line(
    const char *line
)
{
    if (
        strcmp(
            line,
            "START"
        ) == 0
    )
    {
        printf("#START_RECEIVED\n");
        return;
    }

    if (
        strcmp(
            line,
            "CTRL,PI"
        ) == 0
    )
    {
        select_controller_mode(
            CTRL_PI
        );
        return;
    }

    if (
        strcmp(
            line,
            "CTRL,PI_UKF"
        ) == 0
    )
    {
        if (!ukf4_feedback_available())
        {
            printf(
                "#COMMAND_ERROR,CMD=CTRL_PI_UKF,REASON=UKF4_NOT_VALID\n"
            );
            return;
        }

        select_controller_mode(
            CTRL_PI_UKF
        );
        return;
    }

    if (
        strcmp(
            line,
            "CTRL,ADRC"
        ) == 0
    )
    {
        select_controller_mode(
            CTRL_ADRC
        );
        return;
    }

    if (
        strcmp(
            line,
            "CTRL,ADRC_UKF"
        ) == 0
    )
    {
        if (!ukf4_feedback_available())
        {
            printf(
                "#COMMAND_ERROR,CMD=CTRL_ADRC_UKF,REASON=UKF4_NOT_VALID\n"
            );
            return;
        }

        select_controller_mode(
            CTRL_ADRC_UKF
        );
        return;
    }

    if (
        strcmp(
            line,
            "CTRL,PRESSURE"
        ) == 0
    )
    {
        select_controller_mode(
            CTRL_PRESSURE
        );
        return;
    }

    if (
        strcmp(
            line,
            "CTRL,NONE"
        ) == 0
    )
    {
        controller_mode =
            CTRL_NONE;

        control_enabled =
            false;

        cancel_active_pulse_alarm();
        all_outputs_off();

        printf(
            "#CTRL_SET,T_MS=%lu,CTRL=NONE,OUTPUTS=OFF\n",
            (unsigned long)experiment_time_ms(millis_now())
        );

        return;
    }

    float p_rx = 0.0f;

    if (
        sscanf(
            line,
            "SET_P,%f",
            &p_rx
        ) == 1
    )
    {
        if (
            controller_mode
            != CTRL_PRESSURE
        )
        {
            printf(
                "#COMMAND_ERROR,CMD=SET_P,REASON=CTRL_PRESSURE_REQUIRED\n"
            );
            return;
        }

        if (
            !isfinite(p_rx)
            || p_rx < PRESSURE_REF_MIN_KPA
            || p_rx > PRESSURE_REF_MAX_KPA
        )
        {
            printf(
                "#COMMAND_ERROR,CMD=SET_P,REASON=RANGE_0_120_KPA\n"
            );
            return;
        }

        pressure_ref_kpa =
            clampf_local(
                p_rx,
                PRESSURE_REF_MIN_KPA,
                PRESSURE_REF_MAX_KPA
            );

        printf(
            "#PREF_SET,T_MS=%lu,CTRL=%s,NEW=%.3f,P=%.3f\n",
            (unsigned long)experiment_time_ms(millis_now()),
            controller_name(controller_mode),
            pressure_ref_kpa,
            pressure_filtered
        );

        return;
    }

    float q_rx = 0.0f;

    if (
        sscanf(
            line,
            "SET_Q,%f",
            &q_rx
        ) == 1
    )
    {
        if (
            !isfinite(q_rx)
            || q_rx < 0.0f
            || q_rx > 100.0f
        )
        {
            printf(
                "#COMMAND_ERROR,CMD=SET_Q,REASON=RANGE_0_100\n"
            );
            return;
        }

        apply_reference_change(
            q_rx
        );

        return;
    }

    if (
        strncmp(
            line,
            "MARK,",
            5
        ) == 0
    )
    {
        const char *name =
            line + 5;

        if (
            strlen(name)
            == 0u
        )
        {
            printf(
                "#COMMAND_ERROR,CMD=MARK,REASON=EMPTY\n"
            );
            return;
        }

        printf(
            "#MARK,T_MS=%lu,CTRL=%s,QREF=%.3f,NAME=%s\n",
            (unsigned long)experiment_time_ms(millis_now()),
            controller_name(controller_mode),
            position_ref_pct,
            name
        );

        return;
    }

    if (
        strcmp(
            line,
            "START_LOG"
        ) == 0
    )
    {
        /*
         * El header se emite SIEMPRE al iniciar una captura para que
         * el PC no dependa de haber escuchado el boot del micro.
         */
        print_csv_header();

        logging_enabled =
            true;

        printf(
            "#LOG_START,T_MS=%lu\n",
            (unsigned long)experiment_time_ms(millis_now())
        );

        return;
    }

    if (
        strcmp(
            line,
            "STOP_LOG"
        ) == 0
    )
    {
        logging_enabled =
            false;

        printf(
            "#LOG_STOP,T_MS=%lu\n",
            (unsigned long)experiment_time_ms(millis_now())
        );

        return;
    }

    if (
        strcmp(
            line,
            "STATUS"
        ) == 0
    )
    {
        print_status();
        return;
    }

    if (
        strcmp(
            line,
            "VENT"
        ) == 0
    )
    {
        control_enabled =
            false;

        controller_mode =
            CTRL_NONE;

        cancel_active_pulse_alarm();

        pump_off();
        set_state(STATE_VENT);

        printf(
            "#VENT,T_MS=%lu\n",
            (unsigned long)experiment_time_ms(millis_now())
        );

        return;
    }

    if (
        strcmp(
            line,
            "HOLD"
        ) == 0
    )
    {
        control_enabled =
            false;

        controller_mode =
            CTRL_NONE;

        cancel_active_pulse_alarm();

        pump_off();
        set_state(STATE_HOLD);

        printf(
            "#HOLD,T_MS=%lu\n",
            (unsigned long)experiment_time_ms(millis_now())
        );

        return;
    }

    if (
        strcmp(
            line,
            "CALIBRATE"
        ) == 0
    )
    {
        perform_full_calibration(
            RECAL_VENT_MS
        );

        return;
    }

    if (
        strcmp(
            line,
            "END_RUN"
        ) == 0
    )
    {
        control_enabled = false;
        logging_enabled = false;
        controller_mode = CTRL_NONE;

        cancel_active_pulse_alarm();

        pump_off();
        set_state(STATE_VENT);

        printf(
            "#RUN_END_START,T_MS=%lu,VENT_MS=%d\n",
            (unsigned long)experiment_time_ms(millis_now()),
            FINAL_VENT_MS
        );

        sleep_ms(FINAL_VENT_MS);

        all_outputs_off();

        printf(
            "#RUN_FINISHED,T_MS=%lu,OUTPUTS=OFF\n",
            (unsigned long)experiment_time_ms(millis_now())
        );

        return;
    }

    if (
        strcmp(
            line,
            "ABORT"
        ) == 0
    )
    {
        command_abort =
            true;

        return;
    }

    if (
        strlen(line)
        > 0u
    )
    {
        printf(
            "#UNKNOWN_COMMAND,%s\n",
            line
        );
    }
}

static void poll_serial_commands(void)
{
    static char buffer[128];
    static int index = 0;

    while (true)
    {
        int ch =
            getchar_timeout_us(0);

        if (
            ch
            == PICO_ERROR_TIMEOUT
        )
        {
            break;
        }

        char c =
            (char)ch;

        if (
            c == '\n'
            || c == '\r'
        )
        {
            if (index == 0)
                continue;

            buffer[index] =
                '\0';

            process_command_line(
                buffer
            );

            index = 0;

            memset(
                buffer,
                0,
                sizeof(buffer)
            );

            continue;
        }

        if (
            index
            < (int)sizeof(buffer) - 1
        )
        {
            buffer[index++] =
                c;
        }
        else
        {
            index = 0;

            memset(
                buffer,
                0,
                sizeof(buffer)
            );

            printf(
                "#COMMAND_ERROR,REASON=LINE_TOO_LONG\n"
            );
        }
    }
}


/* =================================================================
 * SEGURIDAD
 * ================================================================= */

static void emergency_shutdown(
    const char *reason,
    float pressure
)
{
    control_enabled = false;
    logging_enabled = false;
    controller_mode = CTRL_NONE;

    pump_off();

    cancel_active_pulse_alarm();
    set_state(STATE_VENT);

    printf(
        "#ABORT,T_MS=%lu,REASON=%s,P=%.3f\n",
        (unsigned long)experiment_time_ms(millis_now()),
        reason,
        pressure
    );

    sleep_ms(
        FINAL_VENT_MS
    );

    all_outputs_off();

    printf(
        "#pulse_alarm_fail_count=%lu\n",
        (unsigned long)pulse_alarm_fail_count
    );

    printf(
        "#TEST_FINISHED\n"
    );
}


/* =================================================================
 * CSV
 * ================================================================= */

static void print_csv_header(void)
{
    printf(
        "t_ms,"
        "controller,"
        "control_enabled,"
        "q_ref_pct,"
        "q_unclipped_pct,"
        "q_pct,"
        "q_feedback_pct,"
        "q_feedback_source,"
        "q_error_pct,"

        "pressure_ref_raw_kpa,"
        "pressure_ref_rate_limited_kpa,"
        "pressure_ref_kpa,"
        "pressure_kpa,"
        "pressure_filtered_kpa,"
        "pressure_error_kpa,"
        "outer_rate_limited,"
        "outer_saturated,"
        "outer_updated,"

        "pi_gain_id,"
        "pi_kp,"
        "pi_ki,"
        "pi_previous_error_pct,"
        "pi_p_increment_kpa,"
        "pi_i_increment_kpa,"
        "pi_delta_pref_kpa,"

        "adrc_model_valid,"
        "adrc_model_k,"
        "adrc_model_tau_s,"
        "adrc_b0_identified,"
        "adrc_b0_target,"
        "adrc_b0_active,"
        "adrc_ff_step_kpa,"
        "adrc_ff_gamma,"
        "adrc_ff_applied_kpa,"
        "eso_z1_pct,"
        "eso_z2_pct_s,"
        "eso_equivalent_disturbance_kpa,"
        "eso_updated,"

        "pressure_p_term,"
        "pressure_i_term,"
        "pressure_u_unsat,"
        "pressure_u_cmd,"
        "pressure_pi_updated,"

        "state,"
        "pulse_ms,"
        "pump_on_cmd,"
        "pressure_raw,"
        "pressure_vout,"
        "flex_raw,"
        "flex_filtered_raw,"
        "flex_zero_raw,"
        "flex_span_counts,"

        "ukf_valid,"
        "ukf_ref_valid,"
        "ukf_theta_valid,"
        "ukf_s_pct,"
        "ukf_v_pct_s,"
        "ukf_theta_deg,"
        "ukf_flex_aligned,"
        "ukf_innovation_flex,"
        "ukf_innovation_theta,"
        "ukf_sigma_flex_model,"
        "ukf_sigma_theta_model,"
        "ukf_sigma_s,"
        "ukf_sigma_v,"
        "ukf_eta,"
        "ukf_bf_counts,"
        "ukf_sigma_eta,"
        "ukf_sigma_bf,"
        "ukf_nis,"
        "ukf_rho_eta_bf,"
        "bno1_ok,"
        "bno1_qw,"
        "bno1_qx,"
        "bno1_qy,"
        "bno1_qz,"
        "bno2_ok,"
        "bno2_qw,"
        "bno2_qx,"
        "bno2_qy,"
        "bno2_qz,"
        "ukf_exec_us,"
        "ukf_max_exec_us,"
        "gs_valid,gs_q,gs_eta,gs_clamped\n"
    );
}

static void print_csv_row(
    uint32_t now_ms
)
{
    float raw_pref =
        pressure_ref_kpa;

    float rate_pref =
        pressure_ref_kpa;

    bool rate_limited =
        false;

    bool saturated =
        false;

    float q_feedback =
        active_position_feedback_pct();

    float q_error =
        position_ref_pct
        - q_feedback;

    if (
        controller_mode == CTRL_PI
        || controller_mode == CTRL_PI_UKF
    )
    {
        raw_pref =
            last_position_pi.pressure_ref_raw_kpa;

        rate_pref =
            last_position_pi.pressure_ref_rate_limited_kpa;

        rate_limited =
            last_position_pi.rate_limited;

        saturated =
            last_position_pi.saturated;
    }
    else if (
        (controller_mode == CTRL_ADRC || controller_mode == CTRL_ADRC_UKF)
    )
    {
        raw_pref =
            last_ladrc.pressure_ref_raw_kpa;

        rate_pref =
            last_ladrc.pressure_ref_rate_limited_kpa;

        rate_limited =
            last_ladrc.rate_limited;

        saturated =
            last_ladrc.saturated;
    }

    float equivalent_disturbance =
        (
            active_b0 > 1e-5f
        )
        ? (
            -eso_z2_pct_s
            / active_b0
        )
        : 0.0f;

    const UKFShadowTelemetry *u =
        ukf_shadow_get();

    printf(
        "%lu,"
        "%s,"
        "%d,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%s,"
        "%.3f,"

        "%.5f,"
        "%.5f,"
        "%.5f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%d,"
        "%d,"
        "%d,"

        "%u,"
        "%.6f,"
        "%.6f,"
        "%.3f,"
        "%.5f,"
        "%.5f,"
        "%.5f,"

        "%d,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.3f,"
        "%.6f,"
        "%.6f,"
        "%d,"

        "%.5f,"
        "%.5f,"
        "%.5f,"
        "%.5f,"
        "%d,"

        "%s,"
        "%lu,"
        "%d,"
        "%.2f,"
        "%.5f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.3f,"

        "%d,"
        "%d,"
        "%d,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.5f,"
        "%.3f,"
        "%.5f,"
        "%.3f,"
        "%.6f,"
        "%.6f,"
        "%d,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%d,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%lu,"
        "%lu,%d,%.6f,%.6f,%d\n",

        (unsigned long)experiment_time_ms(now_ms),
        controller_name(controller_mode),
        control_enabled ? 1 : 0,
        position_ref_pct,
        latest_position_unclipped,
        latest_position_pct,
        q_feedback,
        active_position_feedback_name(),
        q_error,

        raw_pref,
        rate_pref,
        pressure_ref_kpa,
        latest_pressure_kpa,
        pressure_filtered,
        last_pressure_pi.error,
        rate_limited ? 1 : 0,
        saturated ? 1 : 0,
        outer_updated_since_log ? 1 : 0,

        (unsigned int)active_gain_region_id,
        active_kp_position,
        active_ki_position,
        last_position_pi.previous_error,
        last_position_pi.p_increment_kpa,
        last_position_pi.i_increment_kpa,
        last_position_pi.delta_pref_kpa,

        adrc_model_valid ? 1 : 0,
        adrc_model_k,
        adrc_model_tau_s,
        adrc_b0_identified,
        target_b0,
        active_b0,
        step_ff_kpa,
        last_ladrc.feedforward_gamma,
        last_ladrc.feedforward_applied_kpa,
        eso_z1_pct,
        eso_z2_pct_s,
        equivalent_disturbance,
        eso_updated_since_log ? 1 : 0,

        last_pressure_pi.p_term,
        last_pressure_pi.i_term,
        last_pressure_pi.output_unsat,
        last_pressure_pi.output,
        pressure_updated_since_log ? 1 : 0,

        state_name(pneumatic_state),
        (unsigned long)last_pulse_ms,
        control_enabled ? 1 : 0,
        latest_pressure_raw,
        latest_pressure_vout,
        latest_flex_raw,
        flex_filtered_raw,
        flex_zero_raw,
        FLEX_SPAN_COUNTS,

        u->valid ? 1 : 0,
        u->reference_valid ? 1 : 0,
        u->theta_valid ? 1 : 0,
        u->s_hat,
        u->v_hat,
        u->theta_deg,
        u->flex_aligned,
        u->innovation_flex,
        u->innovation_theta,
        u->sigma_flex_model,
        u->sigma_theta_model,
        u->sigma_s,
        u->sigma_v,
        u->eta_hat,
        u->b_f_hat,
        u->sigma_eta,
        u->sigma_b_f,
        u->nis,
        u->rho_eta_b_f,
        u->bno1_ok ? 1 : 0,
        u->bno1_qw,
        u->bno1_qx,
        u->bno1_qy,
        u->bno1_qz,
        u->bno2_ok ? 1 : 0,
        u->bno2_qw,
        u->bno2_qx,
        u->bno2_qy,
        u->bno2_qz,
        (unsigned long)u->exec_us,
        (unsigned long)u->max_exec_us,
        schedule_input_valid ? 1 : 0,
        scheduled_plant.q, scheduled_plant.eta,
        scheduled_plant.clamped ? 1 : 0
    );

    outer_updated_since_log =
        false;

    eso_updated_since_log =
        false;

    pressure_updated_since_log =
        false;
}


/* =================================================================
 * STARTUP METADATA
 * ================================================================= */

static void print_startup_metadata(void)
{
    printf(
        "\n#CONTROL_UNIFIED_PI_ADRC_V2_UKF4_PI_ADRC_FEEDBACK_TEST_RP2040\n"
    );

    printf(
        "#architecture=PC_COMMANDS_OUTER_CONTROLLER_SHARED_INNER_LOOP\n"
    );

    printf(
        "#run_power_policy=CTRL_STARTS_PUMP_END_RUN_VENTS_AND_OUTPUTS_OFF\n"
    );

    printf(
        "#controllers=PI,PI_UKF,ADRC,ADRC_UKF,PRESSURE\n"
    );

    printf(
        "#ukf_mode=SHADOW_OR_PI_ADRC_FEEDBACK_TEST\n"
    );

    printf(
        "#ukf_feedback_to_controller=SELECTABLE_PI_UKF_AND_ADRC_UKF\n"
    );

    printf(
        "#ukf_loop_ms=%u\n",
        (unsigned int)UKF_SHADOW_LOOP_MS
    );

    printf(
        "#ukf_state=s_sdot_eta_bF\n"
    );

    printf(
        "#ukf_realtime_direction_input=NONE\n"
    );

    printf(
        "#ukf_sigma_eta_rw=%.6f\n",
        UKFS_SIGMA_ETA_RW
    );

    printf(
        "#ukf_sigma_bf_rw=%.6f\n",
        UKFS_SIGMA_BIAS_RW
    );

    printf(
        "#ukf_R_scale=%.6f\n",
        UKFS_R_SCALE
    );

    printf(
        "#ukf_flex_rezero=0\n"
    );

    printf(
        "#ukf_bias_state_enabled=1\n"
    );

    printf(
        "#sample_period_ms=%d\n",
        SAMPLE_PERIOD_MS
    );

    printf(
        "#log_period_ms=%d\n",
        LOG_PERIOD_MS
    );

    printf(
        "#outer_loop_ms=%d\n",
        OUTER_LOOP_MS
    );

    printf(
        "#pressure_loop_ms=%d\n",
        PRESSURE_LOOP_MS
    );

    printf(
        "#eso_loop_ms=%d\n",
        ESO_LOOP_MS
    );

    printf(
        "#pref_rate_up_kpa_s=%.3f\n",
        PREF_RATE_UP_KPA_S
    );

    printf(
        "#pref_rate_down_kpa_s=%.3f\n",
        PREF_RATE_DOWN_KPA_S
    );

    printf(
        "#Kp_pressure=%.5f\n",
        KP_PRESSURE
    );

    printf(
        "#Ki_pressure=%.5f\n",
        KI_PRESSURE
    );

    printf(
        "#pressure_deadband_kpa=%.3f\n",
        PRESSURE_DEADBAND_KPA
    );

    printf(
        "#reversal_lockout_ms=%d\n",
        REVERSAL_LOCKOUT_MS
    );

    printf(
        "#flex_span_counts=%.3f\n",
        FLEX_SPAN_COUNTS
    );

    printf(
        "#ladrc_omega_c=%.5f\n",
        LADRC_OMEGA_C
    );

    printf(
        "#eso_omega_o=%.5f\n",
        ESO_OMEGA_O
    );

    printf(
        "#ladrc_ff_model_weight=%.3f\n",
        LADRC_FF_MODEL_WEIGHT
    );

    printf(
        "#ladrc_ff_max_up_kpa=%.3f\n",
        LADRC_FF_MAX_UP_KPA
    );

    printf(
        "#ladrc_ff_max_down_kpa=%.3f\n",
        LADRC_FF_MAX_DOWN_KPA
    );

    printf(
        "#ladrc_ff_full_error_pct=%.3f\n",
        LADRC_FF_FULL_ERROR_PCT
    );

    printf(
        "#ladrc_ff_zero_error_pct=%.3f\n",
        LADRC_FF_ZERO_ERROR_PCT
    );

    printf(
        "#hard_pressure_kpa=%.1f\n",
        HARD_PRESSURE_KPA
    );

    printf("#gain_schedule=log_plant_q_eta,PI_P1=%.6f,B0=K/TAU\n", PLANT_P1);
    printf("#schedule_q=UKF_S,eta=UKF_ETA,eta0=UP,eta1=DOWN,edge=HOLD_30_70\n");
    printf("#schedule_invalid=HOLD_LAST,pi_gain_id_0=CONTINUOUS\n");
    for (unsigned i = 0; i < PLANT_ANCHOR_COUNT; ++i)
    {
        const PlantAnchor *a = &PLANT_ANCHORS[i];
        printf("#PLANT_ANCHOR,Q=%.3f,K_UP=%.6f,TAU_UP=%.6f,K_DOWN=%.6f,TAU_DOWN=%.6f\n",
            a->q, a->k_up, a->tau_up, a->k_down, a->tau_down);
    }

    printf(
        "#READY,CONTROL_UNIFIED_PI_ADRC_V2_UKF4_PI_ADRC_FEEDBACK_TEST_RP2040\n"
    );
}


/* =================================================================
 * MAIN
 * ================================================================= */

int main(void)
{
    stdio_init_all();

    gpio_init(
        PUMP_PIN
    );

    gpio_set_dir(
        PUMP_PIN,
        GPIO_OUT
    );

    gpio_init(
        V1_PIN
    );

    gpio_set_dir(
        V1_PIN,
        GPIO_OUT
    );

    gpio_init(
        V2_PIN
    );

    gpio_set_dir(
        V2_PIN,
        GPIO_OUT
    );

    pump_off();
    set_state(
        STATE_VENT
    );

    adc_init();

    adc_gpio_init(
        PRESSURE_PIN
    );

    adc_gpio_init(
        FLEX_PIN
    );

    sleep_ms(
        1500
    );

    /*
     * I2C + two BNO055 + UKF4 module.
     * CTRL,PI keeps FLEX feedback; CTRL,PI_UKF uses UKF4 s_hat.
     * CTRL,ADRC keeps FLEX feedback; CTRL,ADRC_UKF uses UKF4 s_hat
     * in the LESO/LADRC while preserving the original feedforward.
     */
    ukf_shadow_hw_init();

    global_start_ms =
        millis_now();

    print_startup_metadata();
    print_csv_header();

    /*
     * Calibracion inicial automatica, una sola vez.
     * Despues el PC puede repetirla con CALIBRATE.
     */
    perform_full_calibration(
        INITIAL_VENT_MS
    );

    uint32_t now =
        millis_now();

    uint32_t next_sample =
        now;

    uint32_t next_ukf =
        now;

    uint32_t next_eso =
        now;

    uint32_t next_outer =
        now;

    uint32_t next_pressure =
        now;

    uint32_t next_log =
        now;

    while (true)
    {
        now =
            millis_now();

        poll_serial_commands();

        if (command_abort)
        {
            emergency_shutdown(
                "OPERATOR_ABORT",
                pressure_filtered
            );

            return 2;
        }

        update_active_pulse(
            now
        );

        if (
            (int32_t)(
                now - next_sample
            ) < 0
        )
        {
            sleep_ms(1);
            continue;
        }

        next_sample +=
            SAMPLE_PERIOD_MS;

        if (
            (int32_t)(
                now - next_sample
            ) >= SAMPLE_PERIOD_MS
        )
        {
            next_sample =
                now
                + SAMPLE_PERIOD_MS;
        }

        /* ---------------------------------------------------------
         * SENSORES
         * --------------------------------------------------------- */

        latest_pressure_raw =
            read_adc_average(
                PRESSURE_ADC,
                ADC_AVERAGE_SAMPLES
            );

        latest_pressure_vout =
            pressure_raw_to_vout(
                latest_pressure_raw
            );

        latest_pressure_kpa =
            pressure_raw_to_kpa(
                latest_pressure_raw
            );

        if (
            latest_pressure_kpa
            >= HARD_PRESSURE_KPA
        )
        {
            emergency_shutdown(
                "OVERPRESSURE",
                latest_pressure_kpa
            );

            return 1;
        }

        pressure_filtered =
            ewma(
                latest_pressure_kpa,
                pressure_filtered,
                PRESSURE_EWMA_ALPHA
            );

        latest_flex_raw =
            read_adc_average(
                FLEX_ADC,
                ADC_AVERAGE_SAMPLES
            );

        flex_filtered_raw =
            ewma(
                latest_flex_raw,
                flex_filtered_raw,
                FLEX_EWMA_ALPHA
            );

        latest_position_unclipped =
            flex_raw_to_position_unclipped(
                latest_flex_raw
            );

        latest_position_pct =
            clampf_local(
                flex_raw_to_position_unclipped(
                    flex_filtered_raw
                ),
                0.0f,
                100.0f
            );

        /* ---------------------------------------------------------
         * UKF4 20 Hz
         *
         * IMPORTANT:
         *   - reads only quaternion registers from both BNO055;
         *   - estimates s, sdot, eta and bF;
         *   - feeds PI in CTRL,PI_UKF and LESO/LADRC in CTRL,ADRC_UKF.
         * --------------------------------------------------------- */

        if (
            (int32_t)(
                now - next_ukf
            ) >= 0
        )
        {
            ukf_shadow_update(
                flex_filtered_raw
            );

            update_plant_schedule();

            next_ukf +=
                UKF_SHADOW_LOOP_MS;

            if (
                (int32_t)(
                    now - next_ukf
                ) >= (int32_t)UKF_SHADOW_LOOP_MS
            )
            {
                next_ukf =
                    now
                    + UKF_SHADOW_LOOP_MS;
            }
        }

        /* ---------------------------------------------------------
         * LESO SOLO CUANDO ADRC ESTA ACTIVO
         * --------------------------------------------------------- */

        if (
            (controller_mode == CTRL_ADRC || controller_mode == CTRL_ADRC_UKF)
            && control_enabled
            && (int32_t)(
                now - next_eso
            ) >= 0
        )
        {
            update_active_b0();

            if (controller_mode == CTRL_ADRC_UKF && !ukf4_feedback_available())
            {
                /*
                 * Falla segura: no actualizar el observador con una medida
                 * UKF invalida. El lazo externo fijara P_ref a la presion actual.
                 */
                eso_updated_since_log = false;
            }
            else
            {
                leso_update(
                    pressure_ref_kpa,
                    active_position_feedback_pct()
                );

                eso_updated_since_log = true;
            }

            next_eso +=
                ESO_LOOP_MS;

            if (
                (int32_t)(
                    now - next_eso
                ) >= ESO_LOOP_MS
            )
            {
                next_eso =
                    now
                    + ESO_LOOP_MS;
            }
        }

        /* ---------------------------------------------------------
         * CONTROL EXTERNO COMUN 500 ms
         * --------------------------------------------------------- */

        if (
            control_enabled
            && (
                controller_mode == CTRL_PI
                || controller_mode == CTRL_PI_UKF
                || controller_mode == CTRL_ADRC
                || controller_mode == CTRL_ADRC_UKF
            )
            && (int32_t)(
                now - next_outer
            ) >= 0
        )
        {
            if (
                controller_mode == CTRL_PI
                || controller_mode == CTRL_PI_UKF
            )
            {
                if (
                    controller_mode == CTRL_PI_UKF
                    && !ukf4_feedback_available()
                )
                {
                    /*
                     * Falla segura para esta prueba:
                     * si UKF4 deja de ser valido, no seguir llenando.
                     * Se fija P_ref a la presion actual.
                     */
                    pressure_ref_kpa =
                        clampf_local(
                            pressure_filtered,
                            PRESSURE_REF_MIN_KPA,
                            PRESSURE_REF_MAX_KPA
                        );

                    previous_position_error = 0.0f;

                    printf(
                        "#PI_UKF_HOLD,T_MS=%lu,REASON=UKF4_INVALID,PREF=%.3f\n",
                        (unsigned long)experiment_time_ms(now),
                        pressure_ref_kpa
                    );
                }
                else
                {
                    last_position_pi =
                        position_pi_update(
                            position_ref_pct,
                            active_position_feedback_pct()
                        );

                    outer_updated_since_log = true;
                }
            }
            else if (
                controller_mode == CTRL_ADRC
                || controller_mode == CTRL_ADRC_UKF
            )
            {
                if (
                    controller_mode == CTRL_ADRC_UKF
                    && !ukf4_feedback_available()
                )
                {
                    /*
                     * Falla segura para ADRC_UKF: mantener aproximadamente
                     * la presion actual en vez de continuar actuando con una
                     * estimacion invalida.
                     */
                    pressure_ref_kpa =
                        clampf_local(
                            pressure_filtered,
                            PRESSURE_REF_MIN_KPA,
                            PRESSURE_REF_MAX_KPA
                        );

                    ff_gamma_state = 0.0f;

                    printf(
                        "#ADRC_UKF_HOLD,T_MS=%lu,REASON=UKF4_INVALID,PREF=%.3f\n",
                        (unsigned long)experiment_time_ms(now),
                        pressure_ref_kpa
                    );
                }
                else
                {
                    last_ladrc =
                        ladrc_update(
                            position_ref_pct
                        );

                    outer_updated_since_log =
                        true;
                }
            }

            next_outer +=
                OUTER_LOOP_MS;

            if (
                (int32_t)(
                    now - next_outer
                ) >= OUTER_LOOP_MS
            )
            {
                next_outer =
                    now
                    + OUTER_LOOP_MS;
            }
        }

        /* ---------------------------------------------------------
         * LAZO INTERNO COMUN 100 ms
         * --------------------------------------------------------- */

        if (
            control_enabled
            && controller_mode
            != CTRL_NONE
            && (int32_t)(
                now - next_pressure
            ) >= 0
        )
        {
            execute_pressure_control(
                pressure_ref_kpa,
                pressure_filtered,
                now
            );

            pressure_updated_since_log =
                true;

            next_pressure +=
                PRESSURE_LOOP_MS;

            if (
                (int32_t)(
                    now - next_pressure
                ) >= PRESSURE_LOOP_MS
            )
            {
                next_pressure =
                    now
                    + PRESSURE_LOOP_MS;
            }
        }

        /* ---------------------------------------------------------
         * CSV 20 Hz, SOLO SI PC LO HABILITA
         * --------------------------------------------------------- */

        if (
            logging_enabled
            && (int32_t)(
                now - next_log
            ) >= 0
        )
        {
            print_csv_row(
                now
            );

            next_log +=
                LOG_PERIOD_MS;

            if (
                (int32_t)(
                    now - next_log
                ) >= LOG_PERIOD_MS
            )
            {
                next_log =
                    now
                    + LOG_PERIOD_MS;
            }
        }
        else if (
            !logging_enabled
        )
        {
            /*
             * Evita backlog temporal al reactivar logging.
             */
            next_log =
                now
                + LOG_PERIOD_MS;
        }
    }

    return 0;
}
