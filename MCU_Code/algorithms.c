#include "algorithms.h"
#include <stdint.h>

//Global Variables
float duty_min = 0.1;
float duty_max = 0.95;
static float P_0_step_val = 0.035;
float P_O_step;
static float I_C_step_val = 0.025;
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
    float output_limit;
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
/*
typedef struct {
    double *x;
    double *v;
    double *y;
    double *bx;
    double *by;
    double bg;
    double bgx;
    uint32_t k;
    uint32_t iteration;
} Particle;
*/

/* PID functions */
void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_limit = -0.95;
}

float pid_compute(PIDController *pid, float setpoint, float actual_value, float dt) {
    float error = setpoint - actual_value;

    // Proportional term
    float proportional = pid->kp * error;

    // Integral term
    // Update integral term with anti-windup
    if (fabs(pid->output_limit) > pid->integral) { 
        pid->integral += error; 
    } else {
        pid->integral = pid->output_limit * sign(pid->output_limit); // Clamp integral
    }
    float integral = pid->ki * pid->integral;

    // Derivative term
    float derivative = pid->kd * (error - pid->prev_error) * dt;

    // Calculate total output
    float output = proportional + integral + derivative;

    // Saturate output to avoid windup
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    // Update previous error
    pid->prev_error = error;

    return output;
}

// Helper function to get sign of a number
int sign(float num) {
    if (num > 0) return 1;
    else if (num < 0) return -1;
    else return 0;
}
/* End PID Functions*/


/* ALGORITHM FUNCTIONS */

void constant_voltage() {
    pid_init(&cv_pid, 1, 1, 0); // Initialize with example gains 
    float Vref = 17.2;
    float dt = 0.000000001; // not sure what to set this too
    float difference = voltage-Vref;
    printf("Difference: %0.3f\n", difference);
    float duty_raw = pid_compute(&cv_pid, Vref, voltage, dt); 
    //printf("Raw Duty Cycle PID: %0.3f\n", duty_raw_pid);
    //float duty_raw = fabsf(duty_raw_pid);
    printf("Raw Duty Cycle: %0.3f\n", duty_raw);

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    } else {
        duty = duty_raw;
    }

    prevDuty = duty;
}

void perturb_and_observe(int variable){

    float duty_raw;
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
            duty_raw = prevDuty - P_O_step;
        }
        else {
            duty_raw = prevDuty + P_O_step;
        }
    }
    else {
        if(deltaV < 0) {
            duty_raw = prevDuty + P_O_step;
        }
        else {
            duty_raw = prevDuty - P_O_step;
        }
    }

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    else {
        duty = duty_raw;
    }
    
    prevDuty = duty;
    prevVoltage = voltage;
    prevPower = power;
}

void incremental_conductance(int variable){

    float duty_raw;
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
            duty_raw = prevDuty; 
        }
        else {
            if(deltaI > 0){
                duty_raw = prevDuty - I_C_step;
            }
            else {
                duty_raw = prevDuty + I_C_step;
            } 
        }
    }
    else {
        if((voltage*deltaI) == (-current*deltaV)) {
            duty_raw = prevDuty;
        }
        else {
            if((voltage*deltaI) > (-current*deltaV)) {
                duty_raw = prevDuty - I_C_step; 
            }
            else {
                duty_raw = prevDuty + I_C_step; 
            }
        }
    }

    printf("Duty Raw: %0.3f\n", duty_raw);

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    else {
        duty = duty_raw;
    }
    
    prevDuty = duty;
    prevVoltage = voltage;
    prevPower = power; 
}

void beta_method() {

    float duty_raw;

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


    float B = log(fabs(current/voltage))-(c*voltage);
    printf("Beta: %0.3f\n", B);

    float deltaV = voltage - prevVoltage;
    float deltaP = power - prevPower;

    if ((B < Bmax) && (B > Bmin)) {

        if (deltaP < 0) {
            if (deltaV < 0){
                duty_raw = prevDuty - P_0_step_val;
            }
            else {
                duty_raw = prevDuty + P_0_step_val;
            }
        }
        else {
            if(deltaV < 0) {
                duty_raw = prevDuty + P_0_step_val;
            }
            else {
                duty_raw = prevDuty - P_0_step_val;
            }
        }
    }
    else  {
        E = (Bg-B)*4;
        printf("Error: %0.3f\n", E);
        duty_raw=prevDuty+E;
    }

    printf("Duty Raw: %0.3f\n", duty_raw);
    
    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    else {
        duty = duty_raw;
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
    else {
        duty = duty_raw;
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
    }
    
    prevDuty = duty;

}
/*
void particle_swarm_optimization() {
    static Particle p;
    static int initialized = 0;
    const double w = 0.25;  // Inertia weight
    const double c1 = 0.375; // Cognitive parameter
    const double c2 = 0.375; // Social parameter
    const int N = 20;   // Number of particles
    const double pmin = 0.1; 
    const double pmax = 0.95;
    float in = voltage * current;

    if (!initialized) {
        p.x = (double *)malloc(N * sizeof(double));
        p.v = (double *)calloc(N, sizeof(double));
        p.y = (double *)calloc(N, sizeof(double));
        p.bx = (double *)calloc(N, sizeof(double));
        p.by = (double *)calloc(N, sizeof(double));
        p.bg = pmin;
        p.bgx = pmin;
        p.k = 0;
        p.iteration = 1;

        for (int i = 0; i < N; i++) {
            p.x[i] = pmin + (pmax - pmin) * i / (N - 1);
        }
        initialized = 1;
    } else {
        p.y[p.k] = in;

        if (p.y[p.k] > p.by[p.k]) {
            p.bx[p.k] = p.x[p.k];
            p.by[p.k] = p.y[p.k];

            if (p.y[p.k] > p.bg) {
                p.bg = p.y[p.k];
                p.bgx = p.x[p.k];
            }
        }

        double r1 = (double)rand() / (double)RAND_MAX;
        double r2 = (double)rand() / (double)RAND_MAX;

        p.v[p.k] = w * p.v[p.k] + c1 * r1 * (p.bx[p.k] - p.x[p.k]) + c2 * r2 * (p.bgx - p.x[p.k]);
        p.x[p.k] += p.v[p.k];

        //p.x[p.k] = fmax(pmin, fmin(pmax, p.x[p.k]));

        if (p.x[p.k] <= pmin) {
            p.x[p.k] = pmin;
        }
        if (p.x[p.k] >= pmax) {
            p.x[p.k] = pmax;
        }

        p.k++;
        if (p.k >= N) {
            p.k = 0;
            p.iteration++;
        }
    }

    float duty_raw = p.bgx;

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    else {
        duty = duty_raw;
    }
    prevDuty = duty;

}
*/
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

    float PID2_input = PID1_output - voltage_gain;
    pid_init(&rcc_pid2, 0.000000002, -0.001, 0); 
    float duty_raw = pid_compute(&rcc_pid2, 0, PID2_input, dt); // not sure about setpoint here 

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    else {
        duty = duty_raw;  
    }
    prevDuty = duty;
}

/* END ALGORITHM FUNCTIONS */