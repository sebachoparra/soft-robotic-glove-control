#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

struct i2c_inst {
    int dummy;
};

static i2c_inst_t i2c0_inst = {0};
static i2c_inst_t i2c1_inst = {0};
i2c_inst_t *i2c0 = &i2c0_inst;
i2c_inst_t *i2c1 = &i2c1_inst;

void stdio_init_all(void) {}
int getchar_timeout_us(uint32_t us) { (void)us; return -1; }
void gpio_init(uint32_t pin) { (void)pin; }
void gpio_set_dir(uint32_t pin, bool out) { (void)pin; (void)out; }
void gpio_put(uint32_t pin, bool v) { (void)pin; (void)v; }
void gpio_set_function(uint32_t pin, int fn) { (void)pin; (void)fn; }
void gpio_pull_up(uint32_t pin) { (void)pin; }

uint32_t time_us_32(void) { return 0u; }
uint64_t time_us_64(void) { return 0ull; }
absolute_time_t get_absolute_time(void) { return 0ull; }
uint32_t to_ms_since_boot(absolute_time_t t) { return (uint32_t)(t / 1000ull); }
void sleep_ms(uint32_t ms) { (void)ms; }
void sleep_us(uint64_t us) { (void)us; }
alarm_id_t add_alarm_in_ms(uint32_t ms, alarm_callback_t cb, void *ud, bool fire_if_past)
{
    (void)ms; (void)cb; (void)ud; (void)fire_if_past;
    return 1;
}
bool cancel_alarm(alarm_id_t id) { (void)id; return true; }

void adc_init(void) {}
void adc_gpio_init(uint32_t pin) { (void)pin; }
void adc_select_input(uint32_t ch) { (void)ch; }
uint16_t adc_read(void) { return 0u; }

uint32_t i2c_init(i2c_inst_t *i2c, uint32_t baud) { (void)i2c; (void)baud; return baud; }
int i2c_write_blocking(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop)
{
    (void)i2c; (void)addr; (void)src; (void)len; (void)nostop;
    return (int)len;
}
int i2c_read_blocking(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop)
{
    (void)i2c; (void)addr; (void)dst; (void)len; (void)nostop;
    return (int)len;
}
