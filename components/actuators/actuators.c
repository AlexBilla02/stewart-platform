#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "actuators.h"
#include "esp_log.h"
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1 MHz (1 tick = 1 microsecondo)
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 tick = 20 millisecondi (50 Hz)

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500



static const char *TAG="actuators";

static const int servo_gpios[SERVO_COUNT]={SERVO_0_GPIO};

static mcpwm_timer_handle_t timer;
static mcpwm_oper_handle_t operators[SERVO_COUNT]={NULL};
static mcpwm_cmpr_handle_t comparators[SERVO_COUNT]={NULL};
static mcpwm_gen_handle_t generators[SERVO_COUNT]={NULL};

static uint32_t angle_to_compare(float angle){
    float pulse_us=SERVO_MIN_PULSEWIDTH_US + (angle / SERVO_MAX_PULSEWIDTH_US) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US);
    
    return (uint32_t)pulse_us;
}



esp_err_t actuators_init(void){
    // 1. Configurazione Timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    for(int i=0; i<SERVO_COUNT; i++){
        // 2. Configurazione Operatore
        mcpwm_operator_config_t operator_config = {
            .group_id = 0, 
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    
        mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &comparator_config, &comparators[i]));

        mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = servo_gpios[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &generator_config, &generators[i]));


        // 5. Imposta le azioni del generatore (HIGH all'inizio, LOW al match)
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generators[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generators[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW)));
    }
    // 7. Abilita e fai partire il timer (Il segnale PWM inizia a uscire ora)
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    
    
    actuators_set_neutral();

    return ESP_OK;
}

esp_err_t actuators_set_neutral(void){
    const float neutral_angles[SERVO_COUNT]={90.0f};
    return actuators_set_angles(neutral_angles);
}

esp_err_t actuators_set_angles(const float angles[]){
    for(int i=0; i<SERVO_COUNT; i++){
        if(angles[i]<SERVO_MIN_ANGLE || angles[i]>SERVO_MAX_ANGLE){
            ESP_LOGE(TAG,"Angle %f out of range (servo %d)",angles[i],i);
            return ESP_ERR_INVALID_ARG;
        }
    }
    for(int i=0; i<SERVO_COUNT; i++){
        uint32_t compare_val=angle_to_compare(angles[i]);
        mcpwm_comparator_set_compare_value(comparators[i],compare_val);
    }
    return ESP_OK;
}

