#ifndef PICO_PROTOCOL_H_
#define PICO_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PICO_PROTOCOL_MAX_LINE 512u
#define PICO_PNEUMATIC_VENT (-1)
#define PICO_PNEUMATIC_HOLD 0
#define PICO_PNEUMATIC_FILL 1
#define PICO_PNEUMATIC_OFF 2
#define PICO_PNEUMATIC_UNKNOWN 99
#define PICO_COMMAND_HOLD 0u
#define PICO_COMMAND_FILL 1u
#define PICO_COMMAND_VENT 2u
#define PICO_COMMAND_PUMP_ON 3u
#define PICO_COMMAND_PUMP_OFF 4u
#define PICO_COMMAND_ALL_OFF 5u

typedef struct {
  uint32_t seq;
  uint32_t pico_us;
  float pressure_raw;
  float pressure_vout;
  float pressure_kpa;
  float pressure_filtered_kpa;
  float flex_raw;
  float flex_filtered_raw;
  int8_t pneumatic_state;
  bool pulse_active;
  uint32_t pulse_ms_remaining;
  bool pump_on;
  bool safety_latched;
  uint32_t alarm_fail_count;
  uint8_t watchdog_state;
} PicoSensorFrame;

typedef struct {
  uint32_t seq;
  uint32_t pico_us;
  float flex_filtered_raw;
  bool bno1_ok;
  float bno1_quat[4]; /* [w,x,y,z] */
  bool bno2_ok;
  float bno2_quat[4]; /* [w,x,y,z] */
} PicoEstimatorFrame;

typedef struct {
  uint32_t seq;
  uint8_t command;
  uint32_t pulse_ms;
  uint32_t deadline_us;
} PicoActuatorCommand;

typedef struct {
  uint32_t seq;
  bool success;
  float pressure_zero_vout;
  float flex_zero_raw;
} PicoCalibrationResult;

typedef enum {
  PICO_FRAME_NONE = 0,
  PICO_FRAME_SENSOR = 1,
  PICO_FRAME_ESTIMATOR = 2,
  PICO_FRAME_ACTUATOR = 3,
  PICO_FRAME_CALIBRATE = 4,
  PICO_FRAME_CALIBRATION_RESULT = 5
} PicoFrameKind;

bool pico_protocol_encode_calibration(const PicoCalibrationResult *result,
  char *out, size_t capacity);
uint16_t pico_crc16_ccitt_false(const uint8_t *data, size_t length);
bool pico_protocol_encode_sensor(const PicoSensorFrame *frame, char *out, size_t capacity);
bool pico_protocol_encode_estimator(const PicoEstimatorFrame *frame, char *out, size_t capacity);
bool pico_protocol_encode_actuator(const PicoActuatorCommand *command, char *out, size_t capacity);

#endif
