#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct{
    float x;
    float y;
} ball_pos_t;

extern QueueHandle_t ball_pos_queue;

void uart_rx_task(void *pvParameters);

QueueHandle_t get_uart_queue_handle(void);
