#ifndef STUB_PICO_STDLIB_H
#define STUB_PICO_STDLIB_H
#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
typedef unsigned int uint;          /* Pico SDK pico/types.h -- see Note 1 */
#define GPIO_OUT 1
#define GPIO_IN  0
#define GPIO_FUNC_I2C 3
#define PICO_ERROR_TIMEOUT (-1)
void stdio_init_all(void);
int  getchar_timeout_us(uint32_t us);
void gpio_init(uint32_t pin);
void gpio_set_dir(uint32_t pin, bool out);
void gpio_put(uint32_t pin, bool v);
void gpio_set_function(uint32_t pin, int fn);
void gpio_pull_up(uint32_t pin);
#endif
