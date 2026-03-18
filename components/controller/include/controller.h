#pragma once
#include "uart_comm.h"    /* ball_pos_t       */
#include "actuators.h"    /* SERVO_COUNT, limiti angolari */

typedef enum{
    QUIET,
    DEFAULT
} pid_mode_t;

typedef struct {
    float kp;            
    float ki;            
    float kd;            
    float setpoint;      
    float integral;      
    float prev_error;    
    float integral_limit;
    float output_limit;
    pid_mode_t mode;  
} pid_state_t;

typedef struct {
    pid_state_t pid_x;  
    pid_state_t pid_y;  
    float neutral_angle;
    float gain; // gain for the inverted cinematic        
} controller_t;

void controller_init(controller_t *ctrl);
void controller_compute(controller_t *ctrl, const ball_pos_t *pos, float dt, float angles_out[]);
void controller_reset(controller_t *ctrl);
