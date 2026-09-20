#ifndef STUB_HW_ADC_H
#define STUB_HW_ADC_H
#include <stdint.h>
void adc_init(void);
void adc_gpio_init(uint32_t pin);
void adc_select_input(uint32_t ch);
uint16_t adc_read(void);
#endif
