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
    float x;
    float y;
} ball_pos_t;

typedef struct __attribute__((packed)){
    int32_t servo_id;   // 1, 2 o 3
    int32_t angle;      // Angolo in gradi (es. -30 a +30)
} servo_row_t;

typedef struct __attribute__((packed)){
    uint8_t header;         // Byte 0: 0xAA
    uint8_t cmd_type;       // Byte 1: 0x01 o 0x02
    union {                 // Byte 2-9: 8 byte interpretati diversamente
        ball_pos_t tracking;
        servo_row_t servo;
    } payload;
    uint8_t checksum;       // Byte 10: XOR byte 0-9
} uart_packet_t;

extern QueueHandle_t ball_pos_queue;

esp_err_t uart_comm_init(void);
void uart_rx_task(void *pvParameters);
