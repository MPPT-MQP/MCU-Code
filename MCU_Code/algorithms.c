#include "algorithms.h"

//Global Variables
float duty_min = 0.1;
float duty_max = 0.95;
static float P_0_step_val = 0.035;
float P_O_step;
static float I_C_step_val = 0.04;
float I_C_step;

float duty;
float voltage;
float current;
float power;
float temperature;
float irradiance;

float prevDuty = 0.5;
float prevVoltage = 0;
float prevCurrent = 0;
float prevPower = 0;

// RCC Variables
float prevVoltage_gain = 0;
float prevCurrent_gain = 0;
float prevPower_gain = 0;

// Structure for PID controller 
typedef struct {
    float kp, ki, kd; // Gains for P, I, and D terms
    float integral;   // Integral term accumulator
    float prev_error; // Previous error for derivative calculation
} PIDController;

// PID Controller Instances for Each Algorithm 
PIDController cv_pid;
PIDController rcc_pid1;
PIDController rcc_pid2;

// Structure to hold particle information for PSO
struct Particle {
    double x[10];
    double v[10];
    double y[10];
    double bx[10];
    double by[10];
    double bg;
    double bgx;
    unsigned int k;
    unsigned int iteration;
};

/* PID functions */
void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
}

float pid_compute(PIDController *pid, float setpoint, float actual_value, float dt) {
    float error = setpoint - actual_value;

    // Proportional term
    float proportional = pid->kp * error;

    // Integral term
    pid->integral += error * dt;
    float integral = pid->ki * pid->integral;

    // Derivative term
    float derivative = pid->kd * (error - pid->prev_error) / dt;

    // Calculate total output
    float output = proportional + integral + derivative;

    // Update previous error
    pid->prev_error = error;

    return output;
}
/* End PID Functions*/


/* ALGORITHM FUNCTIONS */

void constant_voltage() {
    pid_init(&cv_pid, 1, 1, 0); // Initialize with example gains 
    float Vref = 17.2;
    float dt = 0.000001; // not sure what to set this too
    duty = pid_compute(&cv_pid, 0, voltage-Vref, dt); 

    if (duty >= duty_max || duty <= duty_min) {
        duty = prevDuty;
    }

    prevDuty = duty;
}

void perturb_and_observe(int variable){

    float N = 0.005;
    float deltaV = voltage - prevVoltage;
    float deltaP = power - prevPower;

    if(variable == 1){
        P_O_step = N * abs(deltaP);
    }
    else {
        P_O_step = P_0_step_val;
    }

    if (deltaP < 0) {
        if (deltaV < 0){
            duty = prevDuty - P_O_step;
        }
        else {
            duty = prevDuty + P_O_step;
        }
    }
    else {
        if(deltaV < 0) {
            duty = prevDuty + P_O_step;
        }
        else {
            duty = prevDuty - P_O_step;
        }
    }

    if (duty >= duty_max || duty <= duty_min) {
        duty = prevDuty;
    }

    prevDuty = duty;
    prevVoltage = voltage;
    prevPower = power;
}

void incremental_conductance(int variable){

    float N = 0.00025;
    float deltaV = voltage - prevVoltage;
    float deltaI = current - prevCurrent;
    float deltaP = power - prevPower;

    //float change = deltaI * voltage;
   // float cond = -current * deltaV;

    if(variable == 1){
        I_C_step = N * abs(deltaI/deltaV + current/voltage);
    }
    else {
        I_C_step = I_C_step_val;
    }

    if (deltaV ==  0) {
        if (deltaI == 0){
            duty = prevDuty; 
        }
        else {
            if(deltaI > 0){
                duty = prevDuty - I_C_step;
            }
            else {
                duty = prevDuty + I_C_step;
            } 
        }
    }
    else {
        if(voltage*deltaI == -current*deltaV) {
            duty = prevDuty;
        }
        else {
            if(voltage*deltaI > -current*deltaV) {
                duty = prevDuty - I_C_step; 
            }
            else {
                duty = prevDuty + I_C_step; 
            }
        }
    }

    if (duty >= duty_max || duty <= duty_min) {
        duty = prevDuty;
    }

    prevDuty = duty;
    prevVoltage = voltage;
    prevPower = power; 
}

void beta_method() {

    float Bmin = -320.467;
    float Bmax = -230.369;
    float Bg= (Bmin+Bmax)/2;

    float q=1.6e-19;
    float k=1.38e-23;
    float A=0.945;
    float N=36;
    float T=25;
    float c=q/(k*T*A*N);
    float E;

    if (voltage == 0) {
        voltage = 0.0001;
    }
    
    float B = log(abs(current/voltage))-(c*voltage);

    float deltaV = voltage - prevVoltage;
    float deltaP = power - prevPower;

    if ((B < Bmax) && (B > Bmin)) {

        if (deltaP < 0) {
            if (deltaV < 0){
                duty = prevDuty - P_O_step;
            }
            else {
                duty = prevDuty + P_O_step;
            }
        }
        else {
            if(deltaV < 0) {
                duty = prevDuty + P_O_step;
            }
            else {
                duty = prevDuty - P_O_step;
            }
        }
    }
    else  {

        E = (Bg-B)*4;
        duty=prevDuty+E;

    }
    
    if (duty >= duty_max || duty <= duty_min) {
        duty = prevDuty;
    }

    prevDuty = duty;
    prevVoltage = voltage;
    prevPower = power;

}

void temperature_parametric() {

    float B0 = 19.69;
    float B1 = -0.0003;
    float B2 = -0.088;
    float Vmpp = B0 + B1*irradiance +B2*temperature;
    float duty_raw = voltage - Vmpp;

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    prevDuty = duty;

}

void particle_swarm_optimization() {
    
    // PSO Specification
    double w =  0.5;  // Inertia weight
    double c1 = 1.5; // Cognitive parameter
    double c2 = 1.5; // Social parameter
    int N = 10;   // Number of particles

    float pmin = 0.01 * irradiance + 5.25;
    float pmax = 20;

    float out;

    // Partical Variables
    struct Particle p;
    float prevIrradiance;
    int initialized = 0;

    // Ensure prev_G is initialized
    if (!initialized) {
        prevIrradiance = irradiance;
    }

    // Ensure p is initialized
    if (!initialized || (irradiance != prevIrradiance)) {
        // Reset particles
        // p.x = linspace(pmin, pmax, N);
        for (int i = 0; i < N; i++) {
            p.x[i] = pmin + i * ((pmax - pmin) / (N - 1));
        }
        for (int i = 0; i < N; i++) {
            p.v[i] = 0.0;
            p.y[i] = 0.0;
            p.bx[i] = 0.0;
            p.by[i] = 0.0;
        }
        p.bg = pmin;
        p.bgx = pmin;
        p.k = 0; // Corresponds to Matlab's 1 (using 0-indexing in C)
        p.iteration = 1;

        // Update previous G
        prevIrradiance = irradiance;
        initialized = 1;

        // Output first particle position
        out = p.x[p.k];
        return;
    }

    // Input Update
    p.y[p.k] = power;

    // Best Particle Update
    if (p.y[p.k] > p.by[p.k]) {
        p.bx[p.k] = p.x[p.k];
        p.by[p.k] = p.y[p.k];

        // Update global best if necessary
        if (p.y[p.k] > p.bg) {
            p.bg = p.y[p.k];
            p.bgx = p.x[p.k];
        }
    }

    // PSO Algorithm
    double r1 = (double)rand() / (double)RAND_MAX; // Random number from 0 to 1
    double r2 = (double)rand() / (double)RAND_MAX; // Random number from 0 to 1

    p.v[p.k] = w * p.v[p.k] + c1 * r1 * (p.bx[p.k] - p.x[p.k]) + c2 * r2 * (p.bgx - p.x[p.k]);
    p.x[p.k] = p.x[p.k] + p.v[p.k];

    // Limit Position
    p.x[p.k] = fmax(pmin, fmin(pmax, p.x[p.k]));

    // Update Particle Turn, k
    p.k = p.k + 1;
    if (p.k >= N) {
        p.k = 0;
        p.iteration = 1;
    }

    // Output Best Solution
    double best = p.bgx;
    float duty_raw = best / 21.96;

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    else {
        duty = duty_raw;
        prevDuty = duty;
    }

}

void ripple_correlation_control() {
    
    float voltage_gain = voltage * 0.9;
    float current_gain = current * 100;
    float power_gain = voltage_gain * current_gain;

    float LPF_Beta = 0.0015;

    float LPF1_output = LPF_Beta * power_gain + (1 - LPF_Beta) * prevPower_gain;
    float LPF2_output = LPF_Beta * voltage_gain + (1 - LPF_Beta) * prevVoltage_gain;
    
    float error1 = power_gain - LPF1_output;
    float error2 = voltage_gain - LPF2_output;

    float dt = 0.000001;
    float PID1_input = error1 * error2;
    pid_init(&rcc_pid1, 200, 5, 0);  
    float PID1_output = pid_compute(&rcc_pid1, 0, PID1_input, dt); // not sure about setpoint here

    float PID2_input = PID1_output - error2;
    pid_init(&rcc_pid2, 0.000000002, -0.001, 0); 
    float duty_raw = pid_compute(&rcc_pid2, 0.69, voltage, dt); // not sure about setpoint here 

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    else {
        duty = duty_raw;
        prevDuty = duty;
    }

}

/* END ALGORITHM FUNCTIONS */