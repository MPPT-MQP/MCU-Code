// PID with Anti-windup


#include "PID.h"

void PID_init(PIDController *pid, float Kp, float Ki, float Kd, float Kb, float output_min, float output_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kb = Kb;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->output = 0.0f;
    pid->output_saturated = 0.0f;
}

float PID_update(PIDController *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    float P = pid->Kp * error;
    
    pid->integral += (error + pid->Kb * (pid->output - pid->output_saturated)) * dt;
    float I = pid->Ki * pid->integral;
    
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    
    pid->prev_error = error;
    
    pid->output = P + I + D;
    
    if (pid->output > pid->output_max) {
        pid->output_saturated = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output_saturated = pid->output_min;
    } else {
        pid->output_saturated = pid->output;
    }
    
    return pid->output_saturated;
}
