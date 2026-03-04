#include <stdio.h>
#include "control_loop.h"
#include "controller.h"
#include "uart_comm.h"
#include "actuators.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define LOOP_PERIOD_MS   20
#define LOOP_DT          (LOOP_PERIOD_MS / 1000.0f)

static const char *TAG = "control_loop";
extern QueueHandle_t ball_pos_queue;

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

    while (1) {
        // reads the last ball position received from the camera
        // Note: we use xQueuePeek instead of xQueueReceive to keep the last known position in the queue
        if (xQueuePeek(ball_pos_queue, &current_pos, 0) == pdTRUE) {
            
            // if we have a position, we compute the control step
            // controller step: current_pos -> PID -> inverted cinematic -> target_angles
            controller_compute(&ctrl, &current_pos, LOOP_DT, target_angles);

            // actuator step: target_angles -> physical PWM signal
            actuators_set_angles(target_angles);
        } else {
            // No data has EVER been received yet (queue is empty).
            //actuators_set_neutral();
        }

        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}