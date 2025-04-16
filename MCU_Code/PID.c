// PID with Anti-windup


#include "PID.h"

/*#include "PID.h"
#include <stdio.h>

int main() {
    PIDController pid;

    // Define output limits for your system
    float output_min = 0.0f;  // Minimum duty cycle (e.g., 0%)
    float output_max = 1.0f;  // Maximum duty cycle (e.g., 100%)

    // Initialize the PID controller with gains and output limits
    PID_init(&pid, 0.1f, 0.01f, 0.05f, 0.1f, output_min, output_max);

    float setpoint = 100.0f; // Desired voltage for MPPT
    float measurement = 0.0f; // Current voltage measurement
    float dt = 0.1f; // Time step

    while (1) {
        // Get new voltage measurement here (e.g., from ADC)

        // Compute control output using PID
        float control_output = PID_update(&pid, setpoint, measurement, dt);

        // Apply control_output to your system (e.g., adjust PWM duty cycle)
        printf("Control Output: %f\n", control_output);

        // Simulate system response for testing (replace with actual hardware interaction)
        measurement += control_output * 0.1f;

        // Wait for next time step (simulate delay)
    }

    return 0;
}


*/

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
