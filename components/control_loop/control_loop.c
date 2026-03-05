#include <stdio.h>
#include "control_loop.h"
#include "controller.h"
#include "uart_comm.h"
#include "actuators.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define LOOP_PERIOD_MS   20
#define LOOP_DT          (LOOP_PERIOD_MS / 1000.0f)

static const char *TAG = "control_loop";

void control_loop_task(void *pvParameters)
{
    platform_config_t *cfg = (platform_config_t *)pvParameters;

    controller_t ctrl;
    controller_init(&ctrl, cfg);

    float target_angles[SERVO_COUNT];
    ball_pos_t current_pos = {0};

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_PERIOD_MS);

    ESP_LOGI(TAG, "Control loop started (%d ms period)", LOOP_PERIOD_MS);

    while (1) {
        pid_cfg_t new_pid;
        if (xQueueReceive(pid_cfg_queue, &new_pid, 0) == pdTRUE) {
            controller_set_pid(&ctrl, (float)new_pid.kp, (float)new_pid.ki, (float)new_pid.kd);
        }

        uint8_t save_flag;
        if (xQueueReceive(save_cmd_queue, &save_flag, 0) == pdTRUE) {
            platform_config_t save_cfg;
            save_cfg.kp = ctrl.pid_x.kp;
            save_cfg.ki = ctrl.pid_x.ki;
            save_cfg.kd = ctrl.pid_x.kd;
            actuators_get_angles(save_cfg.neutral_angles);
            config_save(&save_cfg);
            for (int i = 0; i < SERVO_COUNT; i++)
                ctrl.neutral_angles[i] = save_cfg.neutral_angles[i];
        }

        if (xQueuePeek(ball_pos_queue, &current_pos, 0) == pdTRUE) {
            controller_compute(&ctrl, &current_pos, LOOP_DT, target_angles);
            actuators_set_angles(target_angles);
        }

        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}