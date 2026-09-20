#include "pico_protocol.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int float_hex(char *out, size_t capacity, float value)
{
  union { float f; uint32_t u; } bits = {value};
  const bool negative = (bits.u >> 31) != 0u;
  const uint32_t exponent = (bits.u >> 23) & 0xffu;
  const uint32_t fraction = bits.u & 0x7fffffu;
  if (exponent == 0xffu) {
    const char *text = fraction != 0u ? "nan" : "inf";
    return snprintf(out, capacity, "%s%s", negative ? "-" : "", text);
  }
  if (exponent == 0u && fraction == 0u) return snprintf(out, capacity, "%c0x0p+0", negative ? '-' : '+');
  if (exponent == 0u) {
    return snprintf(out, capacity, "%c0x0.%06xp-125", negative ? '-' : '+', (unsigned)fraction);
  }
  const int unbiased = (int)exponent - 127;
  const uint32_t hex_fraction = fraction << 1;
  return snprintf(out, capacity, "%c0x1.%06xp%+d", negative ? '-' : '+',
    (unsigned)hex_fraction, unbiased);
}

static bool append(char *out, size_t capacity, size_t *used, const char *text)
{
  size_t n = strlen(text);
  if (*used + n >= capacity) return false;
  memcpy(out + *used, text, n);
  *used += n; out[*used] = '\0'; return true;
}

static bool append_uint(char *out, size_t capacity, size_t *used, uint32_t value)
{
  char text[16]; int n = snprintf(text, sizeof(text), "%u", value);
  return n > 0 && append(out, capacity, used, text);
}

static bool append_int(char *out, size_t capacity, size_t *used, int value)
{
  char text[16]; int n = snprintf(text, sizeof(text), "%d", value);
  return n > 0 && append(out, capacity, used, text);
}

static bool append_float(char *out, size_t capacity, size_t *used, float value)
{
  char text[32]; int n = float_hex(text, sizeof(text), value);
  return n > 0 && (size_t)n < sizeof(text) && append(out, capacity, used, text);
}

static bool finish(char *out, size_t capacity, size_t used)
{
  uint16_t crc = pico_crc16_ccitt_false((const uint8_t *)out, used);
  char text[16]; int n = snprintf(text, sizeof(text), "%u\n", (unsigned)crc);
  return n > 0 && append(out, capacity, &used, text);
}

bool pico_protocol_encode_sensor(const PicoSensorFrame *f, char *out, size_t capacity)
{
  size_t used = 0u; out[0] = '\0';
#define U(x) (append_uint(out, capacity, &used, (x)) && append(out, capacity, &used, ","))
#define F(x) (append_float(out, capacity, &used, (x)) && append(out, capacity, &used, ","))
  bool ok = append(out, capacity, &used, "S,") && U(f->seq) && U(f->pico_us) &&
    F(f->pressure_raw) && F(f->pressure_vout) && F(f->pressure_kpa) &&
    F(f->pressure_filtered_kpa) && F(f->flex_raw) && F(f->flex_filtered_raw) &&
    append_int(out, capacity, &used, f->pneumatic_state) && append(out, capacity, &used, ",") &&
    U(f->pulse_active ? 1u : 0u) && U(f->pulse_ms_remaining) && U(f->pump_on ? 1u : 0u) &&
    U(f->safety_latched ? 1u : 0u) && U(f->alarm_fail_count) && U(f->watchdog_state);
#undef U
#undef F
  return ok && finish(out, capacity, used);
}

bool pico_protocol_encode_estimator(const PicoEstimatorFrame *f, char *out, size_t capacity)
{
  size_t used = 0u; out[0] = '\0';
#define U(x) (append_uint(out, capacity, &used, (x)) && append(out, capacity, &used, ","))
#define F(x) (append_float(out, capacity, &used, (x)) && append(out, capacity, &used, ","))
  bool ok = append(out, capacity, &used, "E,") && U(f->seq) && U(f->pico_us) &&
    F(f->flex_filtered_raw) && U(f->bno1_ok ? 1u : 0u) && F(f->bno1_quat[0]) &&
    F(f->bno1_quat[1]) && F(f->bno1_quat[2]) && F(f->bno1_quat[3]) &&
    U(f->bno2_ok ? 1u : 0u) && F(f->bno2_quat[0]) && F(f->bno2_quat[1]) &&
    F(f->bno2_quat[2]) && F(f->bno2_quat[3]);
#undef U
#undef F
  return ok && finish(out, capacity, used);
}

bool pico_protocol_encode_actuator(const PicoActuatorCommand *c, char *out, size_t capacity)
{
  size_t used = 0u; out[0] = '\0';
  bool ok = append(out, capacity, &used, "A,") && append_uint(out, capacity, &used, c->seq) &&
    append(out, capacity, &used, ",") && append_uint(out, capacity, &used, c->command) &&
    append(out, capacity, &used, ",") && append_uint(out, capacity, &used, c->pulse_ms) &&
    append(out, capacity, &used, ",") && append_uint(out, capacity, &used, c->deadline_us) &&
    append(out, capacity, &used, ",");
  return ok && finish(out, capacity, used);
}

bool pico_protocol_encode_calibration(const PicoCalibrationResult *c, char *out, size_t capacity)
{
  size_t used = 0u; out[0] = '\0';
  bool ok = append(out, capacity, &used, "Z,") && append_uint(out, capacity, &used, c->seq) &&
    append(out, capacity, &used, ",") && append_uint(out, capacity, &used, c->success) &&
    append(out, capacity, &used, ",") && append_float(out, capacity, &used, c->pressure_zero_vout) &&
    append(out, capacity, &used, ",") && append_float(out, capacity, &used, c->flex_zero_raw) &&
    append(out, capacity, &used, ",");
  return ok && finish(out, capacity, used);
}
