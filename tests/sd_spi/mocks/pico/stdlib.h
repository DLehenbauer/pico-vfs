#pragma once

#include <stdbool.h>
#include <stdint.h>

#define GPIO_FUNC_SPI 1
#define GPIO_OUT 1

typedef uint64_t absolute_time_t;

absolute_time_t get_absolute_time(void);
void sleep_ms(uint32_t ms);
void gpio_put(unsigned gpio, bool value);
bool gpio_get(unsigned gpio);

static inline uint32_t to_ms_since_boot(absolute_time_t time) {
    return (uint32_t)(time / 1000);
}

static inline absolute_time_t make_timeout_time_ms(uint32_t ms) {
    return get_absolute_time() + (uint64_t)ms * 1000;
}

static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) {
    return (int64_t)(to - from);
}

static inline void gpio_set_function(unsigned gpio, unsigned function) {
    (void)gpio;
    (void)function;
}

static inline void gpio_init(unsigned gpio) {
    (void)gpio;
}

static inline void gpio_set_dir(unsigned gpio, bool out) {
    (void)gpio;
    (void)out;
}

static inline void gpio_pull_up(unsigned gpio) {
    (void)gpio;
}

static inline void gpio_set_drive_strength(unsigned gpio, unsigned strength) {
    (void)gpio;
    (void)strength;
}
