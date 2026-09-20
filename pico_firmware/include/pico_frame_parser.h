#ifndef PICO_FRAME_PARSER_H_
#define PICO_FRAME_PARSER_H_

#include "pico_protocol.h"

typedef struct {
  PicoFrameKind kind;
  union {
    PicoSensorFrame sensor;
    PicoEstimatorFrame estimator;
    PicoActuatorCommand actuator;
    PicoCalibrationResult calibration;
  } value;
} PicoParsedFrame;

bool pico_frame_parse_line(const char *line, size_t length, PicoParsedFrame *out);
bool pico_protocol_deadline_expired(uint32_t now_us, uint32_t deadline_us);
bool pico_protocol_sequence_accept(uint32_t previous, bool have_previous, uint32_t next);

#endif
