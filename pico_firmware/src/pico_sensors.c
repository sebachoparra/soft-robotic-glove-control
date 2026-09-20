#include "pico_sensors.h"

#include <math.h>

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico_board_config.h"

#define BNO_CHIP_ID_REG 0x00u
#define BNO_CHIP_ID_VALUE 0xa0u
#define BNO_PAGE_ID_REG 0x07u
#define BNO_QUATERNION_REG 0x20u
#define BNO_UNIT_SEL_REG 0x3bu
#define BNO_OPERATION_MODE_REG 0x3du
#define BNO_POWER_MODE_REG 0x3eu
#define BNO_SYSTEM_TRIGGER_REG 0x3fu
#define BNO_MODE_CONFIG 0x00u
#define BNO_MODE_IMUPLUS 0x08u

static float adc_average(uint input)
{
  adc_select_input(input);
  uint32_t total = 0u;
  for (uint i = 0u; i < PICO_ADC_AVERAGE_SAMPLES; ++i) total += adc_read();
  return (float)total / (float)PICO_ADC_AVERAGE_SAMPLES;
}

static bool bno_read_registers(i2c_inst_t *bus, uint8_t address, uint8_t reg,
  uint8_t *data, size_t length)
{
  if (i2c_write_blocking(bus, address, &reg, 1, true) != 1) return false;
  return i2c_read_blocking(bus, address, data, length, false) == (int)length;
}

static bool bno_write_register(i2c_inst_t *bus, uint8_t address, uint8_t reg, uint8_t value)
{
  uint8_t bytes[2] = {reg, value};
  return i2c_write_blocking(bus, address, bytes, 2, false) == 2;
}

static bool bno_initialize(i2c_inst_t *bus, uint8_t address)
{
  uint8_t chip_id = 0u;
  if (!bno_read_registers(bus, address, BNO_CHIP_ID_REG, &chip_id, 1u) ||
    chip_id != BNO_CHIP_ID_VALUE) return false;
  if (!bno_write_register(bus, address, BNO_OPERATION_MODE_REG, BNO_MODE_CONFIG)) return false;
  sleep_ms(25u);
  if (!bno_write_register(bus, address, BNO_PAGE_ID_REG, 0x00u)) return false;
  if (!bno_write_register(bus, address, BNO_UNIT_SEL_REG, 0x00u)) return false;
  if (!bno_write_register(bus, address, BNO_POWER_MODE_REG, 0x00u)) return false;
  sleep_ms(10u);
  if (!bno_write_register(bus, address, BNO_SYSTEM_TRIGGER_REG, 0x00u)) return false;
  sleep_ms(10u);
  if (!bno_write_register(bus, address, BNO_OPERATION_MODE_REG, BNO_MODE_IMUPLUS)) return false;
  sleep_ms(30u);
  return true;
}

void pico_sensors_init(PicoSensorState *state)
{
  *state = (PicoSensorState){0};
  state->estimator.bno1_quat[0] = 1.0f;
  state->estimator.bno2_quat[0] = 1.0f;
  adc_init();
  adc_gpio_init(PICO_PRESSURE_GPIO);
  adc_gpio_init(PICO_FLEX_GPIO);
  i2c_init(i2c0, PICO_BNO_I2C_BAUD_HZ);
  gpio_set_function(PICO_BNO_SDA_GPIO, GPIO_FUNC_I2C);
  gpio_set_function(PICO_BNO_SCL_GPIO, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_BNO_SDA_GPIO);
  gpio_pull_up(PICO_BNO_SCL_GPIO);
  state->bno1_present = bno_initialize(i2c0, PICO_BNO1_ADDRESS);
  state->bno2_present = bno_initialize(i2c0, PICO_BNO2_ADDRESS);
}

bool pico_sensors_read_adc(PicoSensorState *state, uint32_t now_us, PicoSensorFrame *out)
{
  const float pressure_raw = adc_average(PICO_PRESSURE_ADC);
  const float flex_raw = adc_average(PICO_FLEX_ADC);
  const float pressure_vout =
    (pressure_raw * PICO_ADC_VREF_V / PICO_ADC_MAX_COUNTS) * PICO_PRESSURE_DIVIDER_GAIN;
  state->pressure_valid = isfinite(pressure_raw) && pressure_raw <= PICO_ADC_MAX_COUNTS;
  state->flex_valid = isfinite(flex_raw) && flex_raw <= PICO_ADC_MAX_COUNTS;
  if (!state->pressure_valid || !state->flex_valid) return false;
  state->pressure_filtered = pico_ewma_step(state->pressure_filtered,
    pico_pressure_from_vout(pressure_vout, state->calibration.pressure_zero_vout));
  state->flex_filtered = pico_ewma_step(state->flex_filtered, flex_raw);
  *out = (PicoSensorFrame){
    .seq = state->sequence++, .pico_us = now_us, .pressure_raw = pressure_raw,
    .pressure_vout = pressure_vout,
    .pressure_kpa = pico_pressure_from_vout(pressure_vout, state->calibration.pressure_zero_vout),
    .pressure_filtered_kpa = state->pressure_filtered, .flex_raw = flex_raw,
    .flex_filtered_raw = state->flex_filtered, .pneumatic_state = PICO_PNEUMATIC_HOLD,
    .pulse_active = false, .pulse_ms_remaining = 0u, .pump_on = false,
    .safety_latched = true, .alarm_fail_count = 0u, .watchdog_state = 0u};
  return true;
}

static bool bno_read_quat(i2c_inst_t *bus, uint8_t address, float q[4])
{
  uint8_t bytes[8];
  if (!bno_read_registers(bus, address, BNO_QUATERNION_REG, bytes, sizeof(bytes))) return false;
  for (unsigned i = 0u; i < 4u; ++i) {
    int16_t raw = (int16_t)((uint16_t)bytes[2u * i] | ((uint16_t)bytes[2u * i + 1u] << 8));
    q[i] = (float)raw / 16384.0f;
  }
  return pico_quaternion_normalize(q);
}

bool pico_sensors_read_bno(PicoSensorState *state, uint32_t now_us, PicoEstimatorFrame *out)
{
  float candidate1[4];
  float candidate2[4];
  bool ok1 = state->bno1_present && bno_read_quat(i2c0, PICO_BNO1_ADDRESS, candidate1);
  bool ok2 = state->bno2_present && bno_read_quat(i2c0, PICO_BNO2_ADDRESS, candidate2);
  if (ok1) ok1 = pico_sensors_commit_quaternion(state->estimator.bno1_quat, candidate1);
  if (ok2) ok2 = pico_sensors_commit_quaternion(state->estimator.bno2_quat, candidate2);
  state->estimator.seq = state->estimator_sequence++;
  state->estimator.pico_us = now_us;
  state->estimator.flex_filtered_raw = state->flex_filtered;
  state->estimator.bno1_ok = ok1;
  state->estimator.bno2_ok = ok2;
  *out = state->estimator;
  return ok1 || ok2;
}

bool pico_sensors_calibrate(PicoSensorState *state, const float *pressure, size_t pressure_count,
  const float *flex, size_t flex_count)
{
  if (!pressure || !flex || pressure_count == 0u || flex_count == 0u) return false;
  float p = 0.0f, f = 0.0f;
  for (size_t i = 0u; i < pressure_count; ++i) {
    if (!isfinite(pressure[i]) || pressure[i] < 0.0f ||
      pressure[i] > PICO_ADC_VREF_V * PICO_PRESSURE_DIVIDER_GAIN) return false;
    p += pressure[i];
  }
  for (size_t i = 0u; i < flex_count; ++i) {
    if (!isfinite(flex[i]) || flex[i] < 0.0f || flex[i] > PICO_ADC_MAX_COUNTS) return false;
    f += flex[i];
  }
  p /= (float)pressure_count;
  f /= (float)flex_count;
  if (!isfinite(p) || !isfinite(f)) return false;
  state->calibration = (PicoCalibration){p, f, true};
  state->pressure_filtered = 0.0f;
  state->flex_filtered = f;
  return true;
}

bool pico_sensors_zero_begin(PicoSensorState *state, uint32_t seq, uint32_t now_us,
  bool vented_outputs)
{
  if (state->zero_active || !vented_outputs || !state->pressure_valid) return false;
  state->zero_active = true;
  state->zero_seq = seq;
  state->zero_next_us = now_us;
  state->zero_count = state->zero_pressure_sum = state->zero_flex_sum = 0u;
  return true;
}

bool pico_sensors_zero_step(PicoSensorState *state, uint32_t now_us, bool vented_outputs,
  PicoCalibrationResult *result)
{
  if (!state->zero_active) return false;
  *result = (PicoCalibrationResult){.seq = state->zero_seq};
  if (!vented_outputs || !state->pressure_valid) {
    state->zero_active = false;
    return true;
  }
  if ((int32_t)(now_us - state->zero_next_us) < 0) return false;
  state->zero_next_us = now_us + 200u;
  adc_select_input(PICO_PRESSURE_ADC);
  sleep_us(10u);
  uint16_t pressure = adc_read();
  adc_select_input(PICO_FLEX_ADC);
  sleep_us(10u);
  uint16_t flex = adc_read();
  if (pressure > PICO_ADC_MAX_COUNTS || flex > PICO_ADC_MAX_COUNTS) {
    state->zero_active = false;
    return true;
  }
  state->zero_pressure_sum += pressure;
  state->zero_flex_sum += flex;
  if (++state->zero_count < PICO_ZERO_SAMPLES) return false;
  const float p_raw = (float)state->zero_pressure_sum / (float)PICO_ZERO_SAMPLES;
  const float p = (p_raw * PICO_ADC_VREF_V / PICO_ADC_MAX_COUNTS) * PICO_PRESSURE_DIVIDER_GAIN;
  const float f = (float)state->zero_flex_sum / (float)PICO_ZERO_SAMPLES;
  result->success = pico_sensors_calibrate(state, &p, 1u, &f, 1u);
  result->pressure_zero_vout = state->calibration.pressure_zero_vout;
  result->flex_zero_raw = state->calibration.flex_zero_raw;
  state->zero_active = false;
  return true;
}
