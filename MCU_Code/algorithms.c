#include "algorithms.h"

//Global Variables
float duty_min = 0.1;
float duty_max = 0.95;
static float P_0_step_val = 0.03;
float P_O_step;
static float I_C_step_val = 0.0001;
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

// Function below for each algorithm

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
        if(deltaI/deltaV == -(current/voltage)) {
            duty = prevDuty;
        }
        else {
            if(deltaI/deltaV > -(current/voltage)) {
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

    float Bmin = -1550.62;
    float Bmax = -145.50;
    float Bg= (Bmin+Bmax)/2;

    float q=1.6e-19;
    float k=1.38e-23;
    float A=0.945;
    float N=30;
    float T=25;
    float c=q/(k*T*A*N);
    float E;

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
    float error = voltage - Vmpp;
    duty = error;

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
    duty = best / 21.96;

}