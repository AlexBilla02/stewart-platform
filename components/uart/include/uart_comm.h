#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

#define CMD_TRACKING    0x01
#define CMD_SERVO_TEST  0x02
#define CMD_SAVE_NVS    0x03
#define PACKET_HEADER   0xAA

typedef struct __attribute__((packed)){
    int32_t x;
    int32_t y;
} ball_pos_t;

typedef struct __attribute__((packed)){
    int32_t servo_id;   
    int32_t angle;      
} servo_row_t;

typedef struct __attribute__((packed)){
    uint8_t header;         
    uint8_t cmd_type;       
    union {                 
        ball_pos_t tracking;
        servo_row_t servo;
    } payload;
    uint8_t checksum;
} uart_packet_t;

extern QueueHandle_t ball_pos_queue;

esp_err_t uart_comm_init(void);
void uart_rx_task(void *pvParameters);
