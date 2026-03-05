#include "uart_comm.h"
#include "actuators.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "string.h"

static const char *TAG="uart_comm";

#define UART_PORT_NUM UART_NUM_1
#define UART_BAUD_RATE 115200
#define BUF_SIZE 1024

#define UART_TX_PIN 43
#define UART_RX_PIN 44

QueueHandle_t ball_pos_queue=NULL;
QueueHandle_t pid_cfg_queue=NULL;
QueueHandle_t save_cmd_queue=NULL;
static QueueHandle_t uart_event_queue;

esp_err_t uart_comm_init(void){
    ball_pos_queue=xQueueCreate(1,sizeof(ball_pos_t));
    pid_cfg_queue=xQueueCreate(1,sizeof(pid_cfg_t));
    save_cmd_queue=xQueueCreate(1,sizeof(uint8_t));
    if(ball_pos_queue==NULL || pid_cfg_queue==NULL || save_cmd_queue==NULL)
        return ESP_FAIL;

    uart_config_t uart_config={
        .baud_rate=UART_BAUD_RATE,
        .data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,
        .stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,
        .source_clk=UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE *2, 0, 20, &uart_event_queue, 0));
    return ESP_OK;

}

void uart_rx_task(void *pvParameters){
    uart_event_t event;
    uart_packet_t packet;

    ESP_LOGI(TAG,"Task UART RX listening");

    while(1){
        if(xQueueReceive(uart_event_queue, (void *) &event,portMAX_DELAY)){
            if(event.type==UART_DATA){
                int len = uart_read_bytes(UART_PORT_NUM, &packet, sizeof(uart_packet_t), portMAX_DELAY);
                if(len == sizeof(uart_packet_t) && packet.header == PACKET_HEADER){
                    
                    uint8_t crc = 0;
                    for (int i = 0; i < sizeof(uart_packet_t) - 1; i++) {
                        crc ^= ((uint8_t*)&packet)[i];
                    }

                    if (crc != packet.checksum) {
                        ESP_LOGW(TAG, "Wrong checksum. Packet discarded");
                        continue;
                    }

                    switch (packet.cmd_type) {
                        case CMD_TRACKING:{
                            xQueueOverwrite(ball_pos_queue, &packet.payload.tracking);
                            break;
                        }
                            
                        case CMD_SERVO_TEST:{
                            ESP_LOGI(TAG, "New calibration command (servo_id: %d) angle (%d)", packet.payload.servo.servo_id, packet.payload.servo.angle);
                            actuators_set_angles_single(packet.payload.servo.servo_id, packet.payload.servo.angle);
                            break;
                        }

                        case CMD_PID_CFG:{
                            ESP_LOGI(TAG, "PID config: Kp=%u Ki=%u Kd=%u",
                                    packet.payload.pid.kp, packet.payload.pid.ki, packet.payload.pid.kd);
                            xQueueOverwrite(pid_cfg_queue, &packet.payload.pid);
                            break;
                        }

                        case CMD_SAVE:{
                            ESP_LOGI(TAG, "Save command received");
                            uint8_t flag = 1;
                            xQueueOverwrite(save_cmd_queue, &flag);
                            break;
                        }
                    }
                }
            }
        }
    }
}