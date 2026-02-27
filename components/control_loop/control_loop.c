#include <stdio.h>
#include "control_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "actuators.h"
#include "esp_log.h"

#define LOOP_PERIOD_MS 20

static const char *TAG="control_loop";




void control_loop_task(void *pvParameters){

    float target_angles[SERVO_COUNT]={90.0f};
    TickType_t xLastWakeTime=xTaskGetTickCount();
    const TickType_t xFrequency=pdMS_TO_TICKS(LOOP_PERIOD_MS);

    while(1){
        //if(xQueueReceive(get_uart_queue_handle(),&current_pos,0)==pdTRUE){
        //    controller_compute(current_pos, target_angles);
            actuators_set_angles(target_angles);
        //}
        //else{
        //
        //}
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}