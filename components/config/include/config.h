#pragma once
#include "esp_err.h"
#include "actuators.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float neutral_angles[SERVO_COUNT];
} platform_config_t;

esp_err_t config_load(platform_config_t *cfg);
esp_err_t config_save(const platform_config_t *cfg);
