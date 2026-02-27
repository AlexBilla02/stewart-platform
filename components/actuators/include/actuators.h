#pragma once
#include "esp_err.h"

#define SERVO_COUNT 1

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

#define SERVO_0_GPIO 17
#define SERVO_1_GPIO 18
#define SERVO_2_GPIO 19





esp_err_t actuators_init(void);

esp_err_t actuators_set_angles(const float angles[]);
esp_err_t actuators_set_neutral(void);
