#pragma once
#include "uart_comm.h"
#include "actuators.h"
#include "config.h"

typedef struct {
    float kp;            
    float ki;            
    float kd;            
    float setpoint;      
    float integral;      
    float prev_error;    
    float integral_limit;
    float output_limit;  
} pid_state_t;

typedef struct {
    pid_state_t pid_x;  
    pid_state_t pid_y;  
    float neutral_angles[SERVO_COUNT];
    float gain;        
} controller_t;

void controller_init(controller_t *ctrl, const platform_config_t *cfg);
void controller_compute(controller_t *ctrl, const ball_pos_t *pos, float dt, float angles_out[]);
void controller_reset(controller_t *ctrl);
void controller_set_pid(controller_t *ctrl, float kp, float ki, float kd);
