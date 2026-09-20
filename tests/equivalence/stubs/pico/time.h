#ifndef STUB_PICO_TIME_H
#define STUB_PICO_TIME_H
#include <stdint.h>
#include <stdbool.h>
typedef uint64_t absolute_time_t;
typedef int32_t  alarm_id_t;
typedef int64_t (*alarm_callback_t)(alarm_id_t, void *);
uint32_t time_us_32(void);
uint64_t time_us_64(void);
absolute_time_t get_absolute_time(void);
uint32_t to_ms_since_boot(absolute_time_t t);
void sleep_ms(uint32_t ms);
void sleep_us(uint64_t us);
alarm_id_t add_alarm_in_ms(uint32_t ms, alarm_callback_t cb, void *ud, bool fire_if_past);
bool cancel_alarm(alarm_id_t id);
#endif
