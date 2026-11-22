#pragma once

#include <stdbool.h>

typedef struct {
    bool locked;
} mutex_t;

void mutex_init(mutex_t *mutex);
void mutex_enter_blocking(mutex_t *mutex);
void mutex_exit(mutex_t *mutex);

typedef struct {
    unsigned depth;
} recursive_mutex_t;

#define auto_init_recursive_mutex(mutex) ((void)(mutex))

void recursive_mutex_enter_blocking(recursive_mutex_t *mutex);
void recursive_mutex_exit(recursive_mutex_t *mutex);
