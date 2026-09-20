#include <math.h>
#include <stdbool.h>

bool pico_quaternion_normalize(float q[4])
{
  float norm2 = 0.0f;
  for (unsigned i = 0u; i < 4u; ++i) {
    if (!isfinite(q[i])) return false;
    norm2 += q[i] * q[i];
  }
  if (!isfinite(norm2) || norm2 <= 1.0e-12f) return false;
  const float inverse_norm = 1.0f / sqrtf(norm2);
  for (unsigned i = 0u; i < 4u; ++i) q[i] *= inverse_norm;
  return true;
}

bool pico_sensors_commit_quaternion(float last_good[4], const float candidate[4])
{
  float temporary[4];
  for (unsigned i = 0u; i < 4u; ++i) temporary[i] = candidate[i];
  if (!pico_quaternion_normalize(temporary)) return false;
  for (unsigned i = 0u; i < 4u; ++i) last_good[i] = temporary[i];
  return true;
}

float pico_pressure_from_vout(float vout, float zero_vout)
{
  return (vout - zero_vout) * 50.0f;
}

float pico_ewma_step(float previous, float sample)
{
  return previous == 0.0f ? sample : 0.8f * previous + 0.2f * sample;
}
