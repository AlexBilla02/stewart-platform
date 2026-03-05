#include <stdio.h>
#include "actuators.h"
#include "control_loop.h"
#include "uart_comm.h"
#include "config.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG="main";
static platform_config_t app_config;

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    config_load(&app_config);

    ESP_ERROR_CHECK(actuators_init());
    actuators_set_angles(app_config.neutral_angles);

    ESP_ERROR_CHECK(uart_comm_init());

    xTaskCreate(control_loop_task,"control_loop_task",configMINIMAL_STACK_SIZE * 3, &app_config, 5, NULL);
    xTaskCreate(uart_rx_task,"uart_rx_task",configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);
}