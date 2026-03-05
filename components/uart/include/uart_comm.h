#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

#define CMD_SAVE        0x00
#define CMD_TRACKING    0x01
#define CMD_SERVO_TEST  0x02
#define CMD_PID_CFG     0x03
#define PACKET_HEADER   0xAA

typedef struct __attribute__((packed)){
    float x;
    float y;
} ball_pos_t;

typedef struct __attribute__((packed)){
    int32_t servo_id;
    int32_t angle;
} servo_row_t;

typedef struct __attribute__((packed)){
    uint8_t kp;
    uint8_t ki;
    uint8_t kd;
} pid_cfg_t;

typedef struct __attribute__((packed)){
    uint8_t header;
    uint8_t cmd_type;
    union {
        ball_pos_t tracking;
        servo_row_t servo;
        pid_cfg_t pid;
    } payload;
    uint8_t checksum;
} uart_packet_t;

extern QueueHandle_t ball_pos_queue;
extern QueueHandle_t pid_cfg_queue;
extern QueueHandle_t save_cmd_queue;

esp_err_t uart_comm_init(void);
void uart_rx_task(void *pvParameters);
