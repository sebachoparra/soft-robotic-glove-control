#include <setjmp.h>
#include <stdarg.h>
#define main hardware_test_main
#include "test_pressure_hardware.c"
#undef main

static jmp_buf done;
static uint32_t clock_us;
static unsigned loops, estimator_count;
static bool expected_valid;
static bool calibration_fault;
static char input[128];
static size_t input_offset;
static char estimator_wire[3][PICO_PROTOCOL_MAX_LINE];
int stdio_usb;
void stdio_init_all(void) {}
void stdio_set_translate_crlf(void *driver, bool enabled) {
  assert(driver == &stdio_usb && !enabled);
}
uint32_t time_us_32(void) { return clock_us; }
int getchar_timeout_us(uint32_t us) {
  (void)us; return input[input_offset] ? input[input_offset++] : -1;
}
static int capture_printf(const char *format, ...) {
  char wire[PICO_PROTOCOL_MAX_LINE];
  va_list args; va_start(args, format);
  int n = vsnprintf(wire, sizeof(wire), format, args); va_end(args);
  if (wire[0] == 'E') {
    assert(estimator_count < 3);
    strcpy(estimator_wire[estimator_count], wire);
    PicoParsedFrame parsed;
    assert(pico_frame_parse_line(wire, strlen(wire), &parsed));
    assert(parsed.kind == PICO_FRAME_ESTIMATOR);
    PicoEstimatorFrame *e = &parsed.value.estimator;
    assert(e->seq == estimator_count && e->pico_us == clock_us);
    assert(e->bno1_ok == expected_valid && e->bno2_ok == expected_valid);
    assert(e->flex_filtered_raw == 2000);
    assert(e->bno1_quat[0] == 1 && e->bno1_quat[1] == 0);
    // Both absent: identity. After a valid read: preserve last-good unit x.
    assert(e->bno2_quat[bno_mock ? 1 : 0] == 1);
    assert(strchr(wire, '\r') == NULL);
    ++estimator_count;
  }
  return n;
}
static void test_tick(void) {
  if (calibration_fault && loops > 0) pins(0, 1, 1);
  if (++loops == 3) longjmp(done, 1);
  clock_us += 20000;
  bno_read_ok = expected_valid = false;
  if (calibration_fault) adc_values[0] = 5000;
}
#define main firmware_main
#define printf capture_printf
#define tight_loop_contents test_tick
#include "../src/main.c"
#undef main
#undef printf
int main(void) {
  for (unsigned scenario = 0; scenario < 3; ++scenario) {
    loops = estimator_count = clock_us = 0;
    calibration_fault = scenario == 2;
    adc_values[0] = 400;
    startup_delays = 0;
    input[0] = 0; input_offset = 0;
    if (calibration_fault) {
      PicoActuatorCommand c = {100, PICO_COMMAND_ALL_OFF, 0, 0};
      assert(pico_protocol_encode_actuator(&c, input, sizeof(input)));
      c.seq++; c.command = PICO_COMMAND_VENT;
      size_t used = strlen(input);
      assert(pico_protocol_encode_actuator(&c, input + used, sizeof(input) - used));
      used = strlen(input);
      snprintf(input + used, sizeof(input) - used, "C,1,%u\n",
        pico_crc16_ccitt_false((const uint8_t *)"C,1,", 4));
    }
    bno_mock = bno_read_ok = expected_valid = scenario == 1;
    memset(directed, 0, sizeof(directed));
    if (setjmp(done) == 0) firmware_main();
    assert(startup_delays == 1);
    assert(estimator_count == 3); // T6 actual main: never suppress invalid E.
    if (scenario == 1) {
      // T7 exact firmware bytes, independently CRC-checked by parser and bridge test.
      const char *names[] = {"estimator-valid.txt", "estimator-invalid.txt"};
      for (unsigned i = 0; i < 2; ++i) {
        char path[512], expected[PICO_PROTOCOL_MAX_LINE];
        snprintf(path, sizeof(path), "%s/%s", PICO_FIRMWARE_FIXTURE_DIR, names[i]);
        FILE *f = fopen(path, "r"); assert(f);
        assert(fgets(expected, sizeof(expected), f)); fclose(f);
        assert(strcmp(estimator_wire[i], expected) == 0);
      }
    }
  }
  return 0;
}
