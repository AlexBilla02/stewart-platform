#pragma once
#include "esp_err.h"

#define SERVO_COUNT 3

#define SERVO_MIN_ANGLE 55
#define SERVO_MAX_ANGLE 135

#define SERVO_0_GPIO 16
#define SERVO_1_GPIO 17
#define SERVO_2_GPIO 18

esp_err_t actuators_init(void);

esp_err_t actuators_set_angles(const float angles[]);
esp_err_t actuators_set_angles_single(uint8_t servo_id, int16_t target_angle);
esp_err_t actuators_set_neutral(void);
void actuators_get_angles(float angles_out[]);
