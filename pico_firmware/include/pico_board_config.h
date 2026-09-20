#ifndef PICO_BOARD_CONFIG_H_
#define PICO_BOARD_CONFIG_H_

#include <stdint.h>

/* Verified board wiring from the validated firmware. */
#define PICO_PUMP_GPIO 16u
#define PICO_V1_GPIO 17u
#define PICO_V2_GPIO 14u
#define PICO_PRESSURE_GPIO 26u
#define PICO_PRESSURE_ADC 0u
#define PICO_FLEX_GPIO 28u
#define PICO_FLEX_ADC 2u
#define PICO_FLEX_ADC_GPIO PICO_FLEX_GPIO
#define PICO_BNO_I2C_PORT_NAME "i2c0"
#define PICO_BNO_SDA_GPIO 4u
#define PICO_BNO_SCL_GPIO 5u
#define PICO_BNO_I2C_BAUD_HZ 400000u
#define PICO_BNO1_ADDRESS 0x28u
#define PICO_BNO2_ADDRESS 0x29u
#ifndef PICO_ACTUATOR_OUTPUT_ENABLE
#define PICO_ACTUATOR_OUTPUT_ENABLE 1u
#endif
#define PICO_MAX_PULSE_MS 5000u
#define PICO_COMMAND_TIMEOUT_US 200000u
#define PICO_HARD_PRESSURE_KPA 200.0f
#define PICO_ADC_AVERAGE_SAMPLES 8u
#define PICO_PRESSURE_DIVIDER_GAIN 1.545f
#define PICO_ZERO_SAMPLES 500u
#define PICO_ADC_VREF_V 3.3f
#define PICO_ADC_MAX_COUNTS 4095.0f

typedef struct {
  uint8_t pressure_adc_gpio;
  uint8_t pressure_adc_input;
  uint8_t flex_gpio;
  uint8_t flex_adc_input;
  uint8_t bno1_i2c_address;
  uint8_t bno2_i2c_address;
  uint8_t bno_i2c_sda_gpio;
  uint8_t bno_i2c_scl_gpio;
  uint32_t bno_i2c_baud_hz;
  uint8_t v1_active_high;
  uint8_t v2_active_high;
  uint8_t pump_active_high;
} PicoBoardConfig;

#endif
