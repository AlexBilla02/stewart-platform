#include <stdio.h>
#include "control_loop.h"
#include "controller.h"
#include "uart_comm.h"
#include "actuators.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define LOOP_PERIOD_MS   20 // strict deadline for the control task
#define LOOP_DT          (LOOP_PERIOD_MS / 1000.0f)

static const char *TAG = "control_loop";

/*
    This is the shared data structure between control task and uart task. For each new position received the uart task
    overwrites this single-item queue and for each 20ms cycle the control task get the updated value without removing it
    using xQueuePeek.
*/

void control_loop_task(void *pvParameters)
{
    // controller initialization
    controller_t ctrl;
    controller_init(&ctrl);

    float target_angles[SERVO_COUNT];
    ball_pos_t current_pos = {0};

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_PERIOD_MS);

    ESP_LOGI(TAG, "Control loop started (%d ms period)", LOOP_PERIOD_MS);

    actuators_set_neutral();

    while (1) {
        // reads the last ball position received from the camera
        // Note: we use xQueuePeek instead of xQueueReceive to keep the last known position in the queue
        if (xQueuePeek(ball_pos_queue, &current_pos, 0) == pdTRUE) {
            
            // if we have a position, we compute the control step
            // controller step: current_pos -> PID -> inverted cinematic -> target_angles
            controller_compute(&ctrl, &current_pos, LOOP_DT, target_angles);

            // actuator step: target_angles -> physical PWM signal
            actuators_set_angles(target_angles);
        }

        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}