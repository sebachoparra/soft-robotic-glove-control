#include "pico_frame_parser.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

static bool uint_field(const char *s, uint32_t *v)
{
  if (s == NULL || *s == '\0') return false;
  for (const char *p = s; *p; ++p) if (*p < '0' || *p > '9') return false;
  errno = 0; char *end = NULL; unsigned long value = strtoul(s, &end, 10);
  if (errno || *end || value > UINT32_MAX) return false;
  *v = (uint32_t)value; return true;
}

static bool int8_field(const char *s, int8_t *v)
{
  if (s == NULL || *s == '\0') return false;
  errno = 0; char *end = NULL; long value = strtol(s, &end, 10);
  if (errno || *end || value < -128 || value > 127) return false;
  *v = (int8_t)value; return true;
}

static bool bool_field(const char *s, bool *v)
{
  if (strcmp(s, "0") == 0) {*v = false; return true;}
  if (strcmp(s, "1") == 0) {*v = true; return true;}
  return false;
}

static bool float_field(const char *s, float *v)
{
  errno = 0; char *end = NULL; float value = strtof(s, &end);
  if (errno || *end || end == s) return false;
  *v = value; return true;
}

static size_t fields(char *line, char *out[], size_t capacity)
{
  size_t count = 0u;
  for (char *token = strtok(line, ","); token != NULL; token = strtok(NULL, ",")) {
    if (count == capacity) return 0u;
    out[count++] = token;
  }
  return count;
}

static bool valid_crc(char *line, size_t length, size_t *crc_comma)
{
  size_t comma = 0u;
  for (size_t i = length - 2u; i > 0u; --i) if (line[i] == ',') {comma = i; break;}
  if (comma == 0u) return false;
  uint32_t received = 0u;
  if (!uint_field(line + comma + 1u, &received) || received > 65535u) return false;
  if (pico_crc16_ccitt_false((const uint8_t *)line, comma + 1u) != (uint16_t)received) return false;
  *crc_comma = comma;
  return true;
}

bool pico_frame_parse_line(const char *line, size_t length, PicoParsedFrame *out)
{
  if (line == NULL || out == NULL || length < 4u || length > PICO_PROTOCOL_MAX_LINE ||
    line[length - 1u] != '\n') return false;
  char copy[PICO_PROTOCOL_MAX_LINE];
  memcpy(copy, line, length - 1u); copy[length - 1u] = '\0';
  size_t crc_comma;
  if (!valid_crc(copy, length, &crc_comma)) return false;
  (void)crc_comma;
  char *f[20]; size_t count = fields(copy, f, 20u);
  if (count == 0u) return false;
  memset(out, 0, sizeof(*out));
  if (f[0][0] == 'S' && count == 17u) {
    PicoSensorFrame *s = &out->value.sensor; out->kind = PICO_FRAME_SENSOR;
    uint32_t u;
    if (!uint_field(f[1], &s->seq) || !uint_field(f[2], &s->pico_us) ||
      !float_field(f[3], &s->pressure_raw) || !float_field(f[4], &s->pressure_vout) ||
      !float_field(f[5], &s->pressure_kpa) || !float_field(f[6], &s->pressure_filtered_kpa) ||
      !float_field(f[7], &s->flex_raw) || !float_field(f[8], &s->flex_filtered_raw) ||
      !int8_field(f[9], &s->pneumatic_state) || !bool_field(f[10], &s->pulse_active) ||
      !uint_field(f[11], &s->pulse_ms_remaining) || !bool_field(f[12], &s->pump_on) ||
      !bool_field(f[13], &s->safety_latched) || !uint_field(f[14], &s->alarm_fail_count) ||
      !uint_field(f[15], &u) || u > 255u) return false;
    s->watchdog_state = (uint8_t)u; return true;
  }
  if (f[0][0] == 'E' && count == 15u) {
    PicoEstimatorFrame *e = &out->value.estimator; out->kind = PICO_FRAME_ESTIMATOR;
    if (!uint_field(f[1], &e->seq) || !uint_field(f[2], &e->pico_us) ||
      !float_field(f[3], &e->flex_filtered_raw) || !bool_field(f[4], &e->bno1_ok) ||
      !float_field(f[5], &e->bno1_quat[0]) || !float_field(f[6], &e->bno1_quat[1]) ||
      !float_field(f[7], &e->bno1_quat[2]) || !float_field(f[8], &e->bno1_quat[3]) ||
      !bool_field(f[9], &e->bno2_ok) || !float_field(f[10], &e->bno2_quat[0]) ||
      !float_field(f[11], &e->bno2_quat[1]) || !float_field(f[12], &e->bno2_quat[2]) ||
      !float_field(f[13], &e->bno2_quat[3])) return false;
    return true;
  }
  if (strcmp(f[0], "C") == 0 && count == 3u) {
    out->kind = PICO_FRAME_CALIBRATE;
    return uint_field(f[1], &out->value.calibration.seq);
  }
  if (strcmp(f[0], "Z") == 0 && count == 6u) {
    PicoCalibrationResult *c = &out->value.calibration;
    out->kind = PICO_FRAME_CALIBRATION_RESULT;
    return uint_field(f[1], &c->seq) && bool_field(f[2], &c->success) &&
      float_field(f[3], &c->pressure_zero_vout) && float_field(f[4], &c->flex_zero_raw);
  }
  if (f[0][0] == 'A' && count == 6u) {
    PicoActuatorCommand *a = &out->value.actuator; out->kind = PICO_FRAME_ACTUATOR;
    uint32_t command;
    if (!uint_field(f[1], &a->seq) || !uint_field(f[2], &command) || command > PICO_COMMAND_ALL_OFF ||
      !uint_field(f[3], &a->pulse_ms) || !uint_field(f[4], &a->deadline_us)) return false;
    a->command = (uint8_t)command; return true;
  }
  return false;
}

bool pico_protocol_deadline_expired(uint32_t now_us, uint32_t deadline_us)
{
  return deadline_us != 0u && (int32_t)(now_us - deadline_us) > 0;
}

bool pico_protocol_sequence_accept(uint32_t previous, bool have_previous, uint32_t next)
{
  return !have_previous || next != previous;
}
