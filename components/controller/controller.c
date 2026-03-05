#include "controller.h"
#include <math.h>
#include "esp_log.h"

#define DEFAULT_SETPOINT_X    0.0f
#define DEFAULT_SETPOINT_Y    0.0f

#define DEFAULT_INTEGRAL_LIM  50.0f
#define DEFAULT_OUTPUT_LIM    30.0f

#define IK_GAIN               1.0f

static const char *TAG = "controller";

static const float servo_mount_angle[3] = {
    // 0, 120, 240
    0.0f,
    2.0f * M_PI / 3.0f,
    4.0f * M_PI / 3.0f
};

// initialize PID
static void pid_init(pid_state_t *pid,
                     float kp, float ki, float kd,
                     float setpoint,
                     float integral_limit,
                     float output_limit)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->setpoint       = setpoint;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
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
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

// reset PID
static void pid_reset(pid_state_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

// initialize controller
void controller_init(controller_t *ctrl, const platform_config_t *cfg)
{
    pid_init(&ctrl->pid_x,
            cfg->kp, cfg->ki, cfg->kd,
            DEFAULT_SETPOINT_X,
            DEFAULT_INTEGRAL_LIM, DEFAULT_OUTPUT_LIM);

    pid_init(&ctrl->pid_y,
            cfg->kp, cfg->ki, cfg->kd,
            DEFAULT_SETPOINT_Y,
            DEFAULT_INTEGRAL_LIM, DEFAULT_OUTPUT_LIM);

    for (int i = 0; i < SERVO_COUNT; i++)
        ctrl->neutral_angles[i] = cfg->neutral_angles[i];
    ctrl->gain = IK_GAIN;

    ESP_LOGI(TAG, "Controller initialized  Kp=%.2f Ki=%.2f Kd=%.2f",
            cfg->kp, cfg->ki, cfg->kd);
}

/*
    Performs the totality of the control step:
    1. PID calculation -> pitch (x) and roll (y)
    2. Inverted cinematic
*/
void controller_compute(controller_t *ctrl, const ball_pos_t *pos, float dt, float angles_out[])
{
    // PID
    // the error on the X axis requires a correction rotating the plate w.r.t the Y axis
    float roll = pid_compute(&ctrl->pid_x, pos->x, dt);

    // the error on the Y axis requires a correction rotating the plate w.r.t the X axis
    float pitch = pid_compute(&ctrl->pid_y, pos->y, dt);

    /* 
        Inverted cinematic step:
        Please keep in mind that this configuration works for three servos positioned
        at a 120° distance between each others, under a circular plate and with the servo 0 positioned
        at 0° (corresponding to the max right pos of the ball, in the center line)
        
        Each servo produces an angle based on its position, for example, servo 0 will not have effect for
        any value of pitch, since ti would involve a rotation around the X axis

        The gain can be used for correction beyond the PID parameters
    */
    for (int i = 0; i < SERVO_COUNT; i++) {
        float alpha = servo_mount_angle[i];
        float tilt  = ctrl->gain * (cosf(alpha) * roll + sinf(alpha) * pitch);
        float angle = ctrl->neutral_angles[i] + tilt;

        // output clamp
        if (angle < (float)SERVO_MIN_ANGLE) angle = (float)SERVO_MIN_ANGLE;
        if (angle > (float)SERVO_MAX_ANGLE) angle = (float)SERVO_MAX_ANGLE;

        angles_out[i] = angle;
    }
}

void controller_reset(controller_t *ctrl)
{
    pid_reset(&ctrl->pid_x);
    pid_reset(&ctrl->pid_y);
}

void controller_set_pid(controller_t *ctrl, float kp, float ki, float kd)
{
    ctrl->pid_x.kp = kp;
    ctrl->pid_x.ki = ki;
    ctrl->pid_x.kd = kd;
    ctrl->pid_y.kp = kp;
    ctrl->pid_y.ki = ki;
    ctrl->pid_y.kd = kd;
    pid_reset(&ctrl->pid_x);
    pid_reset(&ctrl->pid_y);
    ESP_LOGI(TAG, "PID updated Kp=%.2f Ki=%.2f Kd=%.2f", kp, ki, kd);
}
