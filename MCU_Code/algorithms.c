#include "algorithms.h"
#include "def.h"
#include <stdint.h>
#include "sdCard.h"

//Global Variables
float duty_min = 0.1;
float duty_max = 0.95;
static float P_0_step_val = 0.035;
float P_O_step;
static float I_C_step_val = 0.025;
float I_C_step;

float duty = 0.1;
float voltage;
float current;
float power;
float temperature;
float irradiance;

float prevDuty = 0.5;
float prevVoltage = 0;
float prevCurrent = 0;
float prevPower = 0;
float prevIrradiance = 0;
float prevTemperature = 0;
// float test_irradiance;
// float test_temperature;

// TMP Variables
float TMP_Vmpp;

//Algorithm of Algorithms
float temperature_hystersis = 0.25;
float irradiance_hysteresis = 10;
int prevAlgo = 0;

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
struct Particle p;

/* ALGORITHM FUNCTIONS */

void duty_test() {
    #ifndef CONSTANT_DUTY
        // Sweep duty cycle 
        if (duty >= 0.95) {
            duty = 0.05;
        } else {
            duty += 0.01;
        }
    #endif

    #ifdef CONSTANT_DUTY
        // Set constant duty cycle 
        duty = CONSTANT_DUTY;
    #endif
}


void constant_voltage() {
    PIDClass_compute(cv_pidClass);
}

void perturb_and_observe(int variable){

    float duty_raw;
    float N = 0.02; // 0.005
    float deltaV = voltage - prevVoltage;
    float deltaP = power - prevPower;

    if(variable == 1){
        if (N*fabsf(deltaP) < P_0_step_val) {
            P_O_step = N * fabsf(deltaP);
        }
        else {
            P_O_step = P_0_step_val;
        }
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
    float N = 0.002;
    float deltaV = voltage - prevVoltage;
    float deltaI = current - prevCurrent;
    float deltaP = power - prevPower;

    if(variable == 1 && deltaV != 0){
        if(N*fabsf(deltaP/deltaV) < I_C_step_val) {
            I_C_step = N * fabsf(deltaP/deltaV);
        } else {
            I_C_step = I_C_step_val;
        }
    }
    else {
        I_C_step = I_C_step_val;
    }

   printf("Step Size: %0.3f, ", I_C_step);

    if (deltaV ==  0) {
        if(deltaI == 0) {
            duty_raw = prevDuty;
        } else if (deltaI > 0) {
            duty_raw = prevDuty+I_C_step;
        }
        else {
            duty_raw = prevDuty-I_C_step;
        }
    } else {
        if (deltaI/deltaV == -current/voltage) {
            duty_raw = prevDuty;
        } else if(deltaI/deltaV > -current/voltage) {
            duty_raw = prevDuty-I_C_step;
        } 
        else {
            duty_raw = prevDuty+I_C_step;
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
    prevCurrent = current;
    prevPower = power; 
}

void beta_method() {

    float duty_raw;

    float q=1.6e-19;
    float k=1.38e-23;
    float A=0.985;
    float N=36;
    float T=temperature;
    float c=q/(k*(T+273.15)*A*N);
    float E;

    // Beta Min: 200 W/m^2, -25 deg C
    float cmin = q/(k*248.15*A*N); 
    float Vmpp_min = 23.373;
    float Impp_min = 0.3389;

    // Beta Max: 1000 W/m^2, 45 deg C
    float cmax = q/(k*318.15*A*N);
    float Vmpp_max = 15.57;
    // float Vmpp_max = 16.826;
    float Impp_max = 1.7478;
    
    float Bmin = log(Impp_min/Vmpp_min)-(cmin*Vmpp_min);
    float Bmax = log(Impp_max/Vmpp_max)-(cmax*Vmpp_max);
    float Bg= (Bmin+Bmax)/2;
    
    float Ba = log(fabs(current/voltage))-(c*voltage);

    float deltaV = voltage - prevVoltage;
    float deltaP = power - prevPower;

    if ((Ba < Bmax) && (Ba > Bmin)) {
        // Switch to P&O
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
        E = (Ba-Bg)*4;
        printf("\nerror: %0.3f", E);
        duty_raw=prevDuty+E;
    }

    printf("Bmin: %0.3f, Bmax: %0.3f, Ba: %0.3f\n", Bmin, Bmax, Ba);
    
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
    TMP_Vmpp = B0 + B1*irradiance +B2*temperature;
    PIDClass_compute(TMP_pidClass);
    prevDuty = duty;

}

void particle_swarm_optimization() {

    //float test_irradiance = 1000;

    // PSO Specification
    double w =  0.5;  // Inertia weight
    double c1 = 1.5; // Cognitive parameter
    double c2 = 1.5; // Social parameter
    int N = 10;   // Number of particles

    float pmin = 0.01 * irradiance + 5.25;
    float pmax = 20;

    float out;

    // Partical Variables
    float prevIrradiance = irradiance;
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
    float duty_raw = best / 21.96; //*********************Ask what the 21.96 is

    if (duty_raw >= duty_max || duty_raw <= duty_min) {
        duty = prevDuty;
    }
    else {
        duty = duty_raw;
    }

    prevDuty = duty;

} 


void algorithm_of_algorithms() {

    // For testing with actual irradiance and temp measurements
    float test_irradiance = irradiance;
    float test_temperature = temperature;

    // For testing AofA without sensors 
    // test_irradiance = (float)rand() / RAND_MAX * 1000;
    // test_temperature = (float)rand() / RAND_MAX * 25;
    // float test_irradiance = 401.2;
    // float test_temperature = 0.2;
    
    /* Set temperature and irradiance thresholds
    for when AofA will switch which algorithm is run */
    int switch_algo_flag = 1;
    float deltaG = test_irradiance - prevIrradiance;
    float deltaT = test_temperature - prevTemperature;

    if(fabs(deltaG) > irradiance_hysteresis || fabs(deltaT) > temperature_hystersis) {
        switch_algo_flag = 1;
    }

    /* Test Conditions Array: store the algorithm that performs best in MATLAB at
    each temperature and irradiance combination */
    int conditions[34][3] = {   
        // Irradiance (W/m^2), Temperature (deg C), Algorithm Toggle
        {1000, 25, PSO},
        {900, 25, PSO},
        {800, 25, PSO},
        {700, 25, PSO},
        {600, 25, CV},
        {500, 25, PSO},
        {400, 25, CV},
        {300, 25, PSO},
        {200, 25, B},

        {1000, 40, TMP},
        {1000, 35, PSO},
        {1000, 30, PSO},
        {1000, 20, CV},
        {1000, 15, TMP},
        {1000, 10, TMP},
        {1000, 5, TMP},
        {1000, 0, PSO},
        {1000, -5, TMP},
        {1000, -10, TMP},
        {1000, -15, TMP},
        {1000, -20, TMP},
        {1000, -25, TMP},

        {800, 20, CV},
        {600, 10, PSO},
        {400, 0, B},
        {400, 30, INC},
        {400, 35, PSO},
        {400, 40, PSO},
        {300, 30, PSO},
        {300, 35, TMP},
        {300, 40, TMP},
        {200, 30, B},
        {200, 35, B},
        {200, 40, B},

    };

    if(switch_algo_flag == 1) {

        float currentDiff;
        float lowestDiff = 1500;
        int currentAlgo;

        for(int i = 0; i<34; i++) { 
            currentDiff = (fabs(conditions[i][0] - test_irradiance)) + fabs(conditions[i][1] - test_temperature);
            if(currentDiff <= lowestDiff) {
                currentAlgo = conditions[i][2];
                lowestDiff = currentDiff;
            }
        }
        sprintf(selectedAlgo, "%s", algorithms[currentAlgo]);
        selectAlgo(currentAlgo);
        prevAlgo = currentAlgo;
       
        switch_algo_flag = 0; //Set flag back to 0 after potentially switching algorithm
    }
    else {
        /* If temperature and irradiance did not change enough
        to switch algorithms, run previous algorithm */
        selectAlgo(prevAlgo);
        sprintf(selectedAlgo, "%s", algorithms[prevAlgo]);
    }

    printf("Selected Algorithm: %s, ", selectedAlgo);
    prevIrradiance = test_irradiance;
    prevTemperature = test_temperature;

}

/* END ALGORITHM FUNCTIONS */