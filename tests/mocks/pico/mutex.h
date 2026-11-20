#pragma once

#include <stdbool.h>

typedef struct {
    bool locked;
} mutex_t;

void mutex_init(mutex_t *mutex);
void mutex_enter_blocking(mutex_t *mutex);
void mutex_exit(mutex_t *mutex);
