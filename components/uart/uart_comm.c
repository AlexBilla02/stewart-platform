#include "uart_comm.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "string.h"

static const char *TAG="uart_comm";

#define UART_PORT_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define BUF_SIZE 1024


#define UART_TX_PIN 43
#define UART_RX_PIN 44

QueueHandle_t ball_pos_queue=NULL;
static QueueHandle_t uart_event_queue;


static esp_err_t uart_comm_init(void){
    ball_pos_queue=xQueueCreate(1,sizeof(ball_pos_t));
    if(ball_pos_queue==NULL)
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
    uint8_t* data_buffer= (uint8_t*) malloc(BUF_SIZE);

    ESP_ERROR_CHECK(uart_comm_init());

    char line_buffer[128];
    int line_pos=0;

    ESP_LOGI(TAG,"Task UART RX listening");

    while(1){
        if(xQueueReceive(uart_event_queue, (void * )&event,portMAX_DELAY)){
            if(event.type==UART_DATA){
                int len=uart_read_bytes(UART_PORT_NUM,data_buffer,event.size,portMAX_DELAY);
                for(int i=0; i<len; i++){
                    char c=(char)data_buffer[i];
                    if(c == '\n'){
                        line_buffer[line_pos]='\0';
                        
                        ball_pos_t new_pos;
                        if (sscanf(line_buffer, "%f,%f", &new_pos.x, &new_pos.y)==2){
                            xQueueOverwrite(ball_pos_queue, &new_pos);
                        }
                        else{
                            ESP_LOGW(TAG,"UART format not valid: %s",line_buffer);
                        }
                        line_pos=0;
                    }
                    else if (c!='\r'){
                        if(line_pos < sizeof(line_buffer) - 1){
                            line_buffer[line_pos++]=c;
                        }
                    }
                }
            }
        }
    }
}