#include "controller.h"
#include <math.h>
#include "esp_log.h"

// best values: 0.08, 0.03, 0.08

// these values are valid for the zone outside the central
#define DEFAULT_KP            0.08f  // proportional gain
#define DEFAULT_KI            0.03f  // integral gain
#define DEFAULT_KD            0.08f  // derivative gain

// these values are valid for the central zone
#define QUIET_KP            0.02f  // proportional gain
#define QUIET_KI            0.03f  // integral gain
#define QUIET_KD            0.01f  // derivative gain

// two thresholds are used in order to avoid chattering
#define QUITE_PID_ERORR_THRESHOLD 15 // central zone entrance threshold
#define CENTRAL_ZONE_ENTRANCE 10 // central zone entrance threshold
#define CENTRAL_ZONE_EXIT 20 // central zone entrance threshold

#define DEFAULT_SETPOINT_X    0.0f  // platform center X
#define DEFAULT_SETPOINT_Y    0.0f  // platform center Y

#define DEFAULT_INTEGRAL_LIM  50.0f // anti windup clamp limit
#define DEFAULT_OUTPUT_LIM    40.0f // output limit

#define NEUTRAL_ANGLE         95.0f // neutral servo position
#define IK_GAIN               1.0f  // inverted cinematic gain 

static const char *TAG = "controller";

static const float servo_mount_angle[3] = {
    // 0, 120, 240
    0.0f,
    2.0f * M_PI / 3.0f,
    4.0f * M_PI / 3.0f
};

// initialize PID (by default it is assumed that the ball is in the DEFAULT zone)
static void pid_init(pid_state_t *pid,
                     float setpoint,
                     float integral_limit,
                     float output_limit)
{
    pid->kp             = DEFAULT_KP;
    pid->ki             = DEFAULT_KI;
    pid->kd             = DEFAULT_KD;
    pid->setpoint       = setpoint;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->mode           = DEFAULT;
}

// reset PID
static void pid_reset(pid_state_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

void controller_reset(controller_t *ctrl)
{
    pid_reset(&ctrl->pid_x);
    pid_reset(&ctrl->pid_y);
}

// zone change function, changes PID parameters according to the zone
static void pid_set_zone(pid_state_t *pid, pid_mode_t mode){
    switch(mode){
        case DEFAULT:{
            pid->kp = DEFAULT_KP;
            pid->ki = DEFAULT_KI;            
            pid->kd = DEFAULT_KD;
            break;            
        }

        case QUIET:{
            pid->kp = QUIET_KP;
            pid->ki = QUIET_KI;            
            pid->kd = QUIET_KD;
            break;            
        }
    }

    // pid_reset(pid);
    pid->mode = mode;
}

/* 
    Calculate the PID for a single step
    
    formula:  u = Kp·e + Ki·∫e·dt + Kd·de/dt

    protections:
    - anti wind-up clamp (limit on integral term)
    - output clamp (limit on overall PID output due to saturation)
*/
static float pid_compute(pid_state_t *pid, float measurement, float dt)
{
    // get current error
    float error = pid->setpoint - measurement;

    float prev_error_diff = fabs(pid->prev_error - error);
    // ESP_LOGI(TAG, "prev error: %.2f", prev_error_diff);

    // check for the current zone
    if(pid->mode == QUIET && (prev_error_diff > QUITE_PID_ERORR_THRESHOLD || fabs(error) > CENTRAL_ZONE_EXIT)){
        pid_set_zone(pid, DEFAULT);
    }
    if(pid->mode == DEFAULT && prev_error_diff <= QUITE_PID_ERORR_THRESHOLD && fabs(error) <= CENTRAL_ZONE_ENTRANCE){
        pid_set_zone(pid, QUIET);
    }

    // integral term with anti windup (used strategy: clamping)
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // derivative term
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // total PID return value
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // check on output saturation
    // if (output >  pid->output_limit) output =  pid->output_limit;
    // if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

// initialize controller
void controller_init(controller_t *ctrl)
{
    pid_init(&ctrl->pid_x,
            DEFAULT_SETPOINT_X,
            DEFAULT_INTEGRAL_LIM, DEFAULT_OUTPUT_LIM);
    pid_set_zone(&ctrl->pid_x, DEFAULT);

    pid_init(&ctrl->pid_y,
            DEFAULT_SETPOINT_Y,
            DEFAULT_INTEGRAL_LIM, DEFAULT_OUTPUT_LIM);
    pid_set_zone(&ctrl->pid_y, DEFAULT);

    ctrl->neutral_angle = NEUTRAL_ANGLE;
    ctrl->gain          = IK_GAIN;

    ESP_LOGI(TAG, "Controller initialized  Kp=%.2f Ki=%.2f Kd=%.2f",
            DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
}

/*
    Performs the totality of the control step:
    1. PID calculation -> pitch (x) and roll (y)
    2. Inverted cinematic
*/
void controller_compute(controller_t *ctrl, const ball_pos_t *pos, float dt, float angles_out[])
{
    // PID
    // the error on the X axis requires a correction rotating the plate w.r.t the Y axis (pitch)
    float pitch = pid_compute(&ctrl->pid_x, pos->x, dt);

    // the error on the Y axis requires a correction rotating the plate w.r.t the X axis (roll)
    float roll = pid_compute(&ctrl->pid_y, pos->y, dt);
    
    ESP_LOGI(TAG, "[pid_x] mode: %d | [pid_y] mode: %d", ctrl->pid_x.mode, ctrl->pid_y.mode);

    /* 
        Inverted cinematic step:
        Please keep in mind that this configuration works for three servos positioned
        at a 120° distance between each others, under a circular plate and with the servo 0 positioned
        at 0° (corresponding to the max right pos of the ball, in the center line)
        
        Each servo produces an angle based on its position, for example, servo 0 will not have effect for
        any value of roll, since it would involve a rotation around the X axis

        The gain can be used for correction beyond the PID parameters
    */
    for (int i = 0; i < SERVO_COUNT; i++) {
        float alpha = servo_mount_angle[i];
        float tilt  = ctrl->gain * (cosf(alpha) * pitch + sinf(alpha) * roll);
        float angle = ctrl->neutral_angle + tilt;

        // output clamp
        if (angle < (float)SERVO_MIN_ANGLE) angle = (float)SERVO_MIN_ANGLE;
        if (angle > (float)SERVO_MAX_ANGLE) angle = (float)SERVO_MAX_ANGLE;

        angles_out[i] = angle;
    }
}
