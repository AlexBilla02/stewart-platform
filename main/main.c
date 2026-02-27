#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"

// --- DEFINIZIONI HARDWARE ---
#define SERVO_PULSE_GPIO             17       // Pin collegato al segnale del servo
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1 MHz (1 tick = 1 microsecondo)
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 tick = 20 millisecondi (50 Hz)

// --- PARAMETRI DEL SERVO SG90 ---
#define SERVO_MIN_PULSEWIDTH_US      500  // Microsecondi per 0 gradi
#define SERVO_MAX_PULSEWIDTH_US      2400 // Microsecondi per 180 gradi
#define SERVO_MAX_DEGREE             180

// ==========================================
// IMPOSTA QUI L'ANGOLO FISSO DESIDERATO
// ==========================================
#define TARGET_ANGLE                 90   // Cambia questo valore (tra 0 e 180)


// Funzione per calcolare la larghezza dell'impulso in microsecondi
uint32_t calculate_pulsewidth(uint32_t angle) {
    // Evita valori fuori range per sicurezza
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    
    return SERVO_MIN_PULSEWIDTH_US + 
        (((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / SERVO_MAX_DEGREE);
}

void app_main(void) {
    printf("Inizializzazione MCPWM per il Servo su GPIO %d...\n", SERVO_PULSE_GPIO);

    // 1. Configurazione Timer
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // 2. Configurazione Operatore
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, 
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // 3. Configurazione Comparatore
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // 4. Configurazione Generatore
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // 5. Imposta le azioni del generatore (HIGH all'inizio, LOW al match)
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    // 6. Calcola e imposta il valore del comparatore in base all'angolo fisso
    uint32_t pulsewidth = calculate_pulsewidth(TARGET_ANGLE);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulsewidth));

    // 7. Abilita e fai partire il timer (Il segnale PWM inizia a uscire ora)
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    printf(">>> Fatto! Servo posizionato a %d gradi (Impulso: %lu us)\n", TARGET_ANGLE, pulsewidth);

    // Il modulo MCPWM è una periferica hardware: continuerà a generare
    // il segnale PWM autonomamente senza pesare sulla CPU.
    // Il ciclo while serve solo per mantenere attivo il task di FreeRTOS.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}