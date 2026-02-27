#include <stdio.h>
#include "actuators.h"
#include "control_loop.h"
#include "uart_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG="main";

void app_main(void) {
    ESP_ERROR_CHECK(actuators_init());
    xTaskCreate(control_loop_task,"control_loop_task",configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(uart_rx_task,"uart_rx_task",configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);
}