#ifndef PICO_SENSORS_H_
#define PICO_SENSORS_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "pico_protocol.h"

typedef struct {
  float pressure_zero_vout;
  float flex_zero_raw;
  bool calibrated;
} PicoCalibration;

typedef struct {
  PicoCalibration calibration;
  bool zero_active;
  uint32_t zero_seq, zero_next_us, zero_pressure_sum, zero_flex_sum, zero_count;
  float pressure_filtered;
  float flex_filtered;
  bool pressure_valid;
  bool flex_valid;
  bool bno1_present;
  bool bno2_present;
  uint32_t sequence;
  uint32_t estimator_sequence;
  PicoEstimatorFrame estimator;
} PicoSensorState;

float pico_pressure_from_vout(float vout, float zero_vout);
float pico_ewma_step(float previous, float sample);
bool pico_quaternion_normalize(float q[4]);
bool pico_sensors_commit_quaternion(float last_good[4], const float candidate[4]);

bool pico_sensors_zero_begin(PicoSensorState *state, uint32_t seq, uint32_t now_us,
  bool vented_outputs);
bool pico_sensors_zero_step(PicoSensorState *state, uint32_t now_us, bool vented_outputs,
  PicoCalibrationResult *result);
void pico_sensors_init(PicoSensorState *state);
bool pico_sensors_read_adc(PicoSensorState *state, uint32_t now_us, PicoSensorFrame *out);
bool pico_sensors_read_bno(PicoSensorState *state, uint32_t now_us, PicoEstimatorFrame *out);
bool pico_sensors_calibrate(PicoSensorState *state, const float *pressure, size_t pressure_count,
  const float *flex, size_t flex_count);

#endif
