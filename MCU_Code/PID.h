// PID with anti-windup example

#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kb;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float output;
    float output_saturated;
} PIDController;

void PID_init(PIDController *pid, float Kp, float Ki, float Kd, float Kb, float output_min, float output_max);
float PID_update(PIDController *pid, float setpoint, float measurement, float dt);

#endif
