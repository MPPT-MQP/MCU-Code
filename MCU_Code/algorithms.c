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

// RCC Variables
float prevVoltage_gain = 0;
float prevCurrent_gain = 0;
float LPF1_prevOutput = 0;
float LPF2_prevOutput = 0;
float prevPower_gain = 0;
float rcc1_input;
float rcc1_setpoint;
float rcc1_output;
float rcc2_input;
float rcc2_setpoint;
float TMP_Vmpp;

//Algorithm of Algorithms
float temperature_hystersis = 0.25;
float irradiance_hysteresis = 20;
int prevAlgo = 0;

// Structure for PID controller 
typedef struct {
    float kp, ki, kd; // Gains for P, I, and D terms
    float integral;   // Integral term accumulator
    float prev_error; // Previous error for derivative calculation
    float output;
} PIDController;

// PID Controller Instances for Each Algorithm 
PIDController cv_pid = {1, 1, 0, 0, 0};
PIDController rcc_pid1 = {200, 5, 0, 0, 0};
PIDController rcc_pid2 = {2e-9, -0.001, 0, 0, 0};

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

/*typedef struct {
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

static Particle p;

int initialized = 0;
float global_save = 0.5;*/

/* PID functions */
float pid_compute(PIDController *pid, float setpoint, float actual_value, float dt) {
    float output;
    float error = setpoint - actual_value;
    //printf("PID Error: %0.3f ", error);

    // Proportional term
    float proportional = pid->kp * fabs(error);

    // Integral term
    if (pid->output > 0.95) {

        pid->integral -= fabs(error) * dt;  // Adjust integral to prevent windup

    } else {

        pid->integral += fabs(error) * dt;  // Adjust integral to prevent windup

    }
    float integral = pid->ki * pid->integral;

    // Derivative term
    float derivative = pid->kd * (error - pid->prev_error) * dt;

    // Calculate total output
    pid->output = proportional + integral + derivative;
    
    // Saturate output if needed
    if (pid->output > 0.95) {
        output = 0.95;
    }
    else if (pid->output < 0.1) {
        output = 0.1;
    }
    else {
        output = pid->output;
    }

    //printf("Error: %0.3f, Proportional: %0.3f, Integral: %0.3f, Output: %0.3f\n", error, proportional, integral, pid->output);

    // Update previous error
    pid->prev_error = error;

    return output;
}

/* End PID Functions*/

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
    // float duty_raw;
    // float Vref = 19.39;
    // float dt = 0.2; // not sure what to set this too
    //float error = voltage-Vref;
    //printf("Voltage: %0.3f ", voltage);
    //printf("Error: %0.3f ", error);
    //duty_raw = pid_compute(&cv_pid, 0, voltage-Vref, dt);
    PIDClass_compute(cv_pidClass);
    // printf("Voltage: %0.3f, Current: %0.3f, Duty Raw: %0.3f\n", voltage, current, duty_raw);

    // if (duty_raw >= duty_max || duty_raw <= duty_min) {
    //     duty = prevDuty;
    // } else {
    //     duty = duty_raw;
    // }
    // //printf("Duty Cycle: %0.3f\n", duty);

    // prevDuty = duty;
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

    // printf("Voltage: %0.3f, Current: %0.3f, Duty Raw: %0.3f\n", voltage, current, duty_raw);

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

    //printf("Voltage: %0.3f, Current: %0.3f, Duty Raw: %0.3f\n", voltage, current, duty_raw);
    
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
    // float duty_raw = voltage - Vmpp;
    PIDClass_compute(TMP_pidClass);
    // printf("Voltage: %0.3f, Current: %0.3f, Duty Raw: %0.3f, Temperature: %0.3f, Irradiance: %0.3f", voltage, current, duty_raw, temperature, irradiance);

    // if (duty_raw >= duty_max || duty_raw <= duty_min) {
    //     duty = prevDuty;
    // }
    // else {
    //     duty = duty_raw;
    // }

    printf("Vmpp: %0.3f\n", TMP_Vmpp);
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

    printf("Voltage: %0.3f, Current: %0.3f, Duty Raw: %0.3f\n", voltage, current, duty_raw);

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
    // float test_irradiance = (float)rand() / RAND_MAX * 1000;
    // float test_temperature = (float)rand() / RAND_MAX * 25;
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
        {1000, 25, CV},
        {900, 25, CV},
        {800, 25, TMP},
        {700, 25, TMP},
        {600, 25, TMP},
        {500, 25, PSO},
        {400, 25, INC},
        {300, 25, TMP},
        {200, 25, INC},

        {1000, 40, TMP},
        {1000, 35, PSO},
        {1000, 30, PSO},
        {1000, 20, PSO},
        {1000, 15, TMP},
        {1000, 10, PSO},
        {1000, 5, PSO},
        {1000, 0, TMP},
        {1000, -5, TMP},
        {1000, -10, TMP},
        {1000, -15, TMP},
        {1000, -20, TMP},
        {1000, -25, TMP},

        {800, 20, TMP},
        {600, 10, TMP},
        {400, 0, INC},
        {400, 30, INC},
        {400, 35, INC},
        {400, 40, PSO},
        {300, 30, TMP},
        {300, 35, PNO},
        {300, 40, PNO},
        {200, 30, INC},
        {200, 35, INC},
        {200, 40, PNO},

    };

    if(switch_algo_flag == 1) {
        
        float minVal_irradiance;
        float minVal_temp;
        float irradiance_currDiff;
        float temp_currDiff;
        int currentAlgo;

        // float irradiance_prevDiff = fabs(conditions[0][0] - test_irradiance);
        // float temp_prevDiff = fabs(conditions[0][1] - test_temperature);
        float irradiance_prevDiff = 1500;
        float temp_prevDiff = 1500;

        for(int i = 0; i<34; i++) { 
            irradiance_currDiff = fabs(conditions[i][0] - test_irradiance);
            temp_currDiff = fabs(conditions[i][1] - test_temperature);
            if(irradiance_currDiff <= irradiance_prevDiff || temp_currDiff <= temp_prevDiff) {
                currentAlgo = conditions[i][2];
                irradiance_prevDiff = irradiance_currDiff;
                temp_prevDiff = temp_currDiff;
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

// void algorithm_of_algorithms() {

//     // For testing with actual irradiance and temp measurements
//     float test_irradiance = irradiance;
//     float test_temperature = temperature;

//     // For testing AofA without sensors 
//     // float test_irradiance = (float)rand() / RAND_MAX * 1000;
//     // float test_temperature = (float)rand() / RAND_MAX * 25;
//     // float test_irradiance = 401.2;
//     // float test_temperature = 0.2;
    
//     /* Set temperature and irradiance thresholds
//     for when AofA will switch which algorithm is run */
//     int switch_algo_flag = 1;
//     float deltaG = test_irradiance - prevIrradiance;
//     float deltaT = test_temperature - prevTemperature;

//     if(fabs(deltaG) > irradiance_hysteresis || fabs(deltaT) > temperature_hystersis) {
//         switch_algo_flag = 1;
//     }

//     /* Test Conditions Array: store the algorithm that performs best in MATLAB at
//     each temperature and irradiance combination */
//     int conditions[34][3] = {   
//         // Irradiance (W/m^2), Temperature (deg C), Algorithm Toggle
//         {1000, 25, CV},
//         {900, 25, CV},
//         {800, 25, TMP},
//         {700, 25, TMP},
//         {600, 25, TMP},
//         {500, 25, PSO},
//         {400, 25, INC},
//         {300, 25, TMP},
//         {200, 25, INC},

//         {1000, 40, TMP},
//         {1000, 35, PSO},
//         {1000, 30, PSO},
//         {1000, 20, PSO},
//         {1000, 15, TMP},
//         {1000, 10, PSO},
//         {1000, 5, PSO},
//         {1000, 0, TMP},
//         {1000, -5, TMP},
//         {1000, -10, TMP},
//         {1000, -15, TMP},
//         {1000, -20, TMP},
//         {1000, -25, TMP},

//         {800, 20, TMP},
//         {600, 10, TMP},
//         {400, 0, INC},
//         {400, 30, INC},
//         {400, 35, INC},
//         {400, 40, PSO},
//         {300, 30, TMP},
//         {300, 35, PNO},
//         {300, 40, PNO},
//         {200, 30, INC},
//         {200, 35, INC},
//         {200, 40, PNO},

//     };

//     if(switch_algo_flag == 1) {

//         /* Create array of differences between each 
//         test condition irradiance and measured irradiance */
//         float irradiance_differences[34] = {0};
        
//         for(int i = 0; i<34; i++) {
//             irradiance_differences[i] = fabs(conditions[i][0] - test_irradiance); 
//         }
        
//         /* Determine the lowest irradiance difference */
//         float minVal_irradiance = irradiance_differences[0];
        
//         for (int i = 0; i<34; i++) {
//             if(irradiance_differences[i] < minVal_irradiance) {
//                 minVal_irradiance = irradiance_differences[i];
//             }
//         }

//         /* Add test conditions with the closest irradiance
//         to a new array of best candidates */
//         int best_list[34][3] = {0};
//         int best_list_size = 0;
        
//         for(int i = 0; i<34; i++) {
//             if(irradiance_differences[i] == minVal_irradiance) {
//                 for(int j = 0; j<3; j++) {
//                     best_list[best_list_size][j] = conditions[i][j];
//                 }
//                 best_list_size++;
//             }
//         }

//         /* Create array of differences between each 
//         test condition temperature and measured temperature */
//         float temp_differences[best_list_size]; 
//         for(int i = 0; i< best_list_size; i++) {
//             temp_differences[i] = fabs(best_list[i][1] - test_temperature);
//         }

//         /* Determine the lowest temperature difference */
//         float minVal_temp = temp_differences[0];
//         for (int i = 0; i<best_list_size; i++) {
//             if(temp_differences[i] < minVal_temp) {
//                 minVal_temp = temp_differences[i];
//             }
//         }

//         /* Find test condition with closest temperature 
//         from best candidates array */
//         for(int i = 0; i<best_list_size; i++) {
//             if(temp_differences[i] == minVal_temp) {
//                 // Print selected algorithm to array which is written to SD card
//                 sprintf(selectedAlgo, "%s", algorithms[best_list[i][2]]);
//                 // Run selected algorithm 
//                 selectAlgo(best_list[i][2]);
//                 // Save as previously selected algorithm
//                 prevAlgo = best_list[i][2];
//             }
//         }
//         switch_algo_flag = 0; //Set flag back to 0 after potentially switching algorithm
//     }
//     else {
//         /* If temperature and irradiance did not change enough
//         to switch algorithms, run previous algorithm */
//         selectAlgo(prevAlgo);
//         sprintf(selectedAlgo, "%s", algorithms[prevAlgo]);
//     }

//     printf("Selected Algorithm: %s, ", selectedAlgo);
//     prevIrradiance = test_irradiance;
//     prevTemperature = test_temperature;

// }

/*
void ripple_correlation_control() {

    float voltage_gain = voltage * 0.33;
    printf("Voltage Gain: %0.3f, ", voltage_gain);
    float current_gain = current * 100;
    printf("Current Gain: %0.3f, ", current_gain);
    float power_gain = voltage_gain * current_gain;
    printf("Power Gain: %0.3f\n", power_gain);
    
    float LPF1_output = power_gain/(1+(0.000005*LPF1_prevOutput));
    printf("LPF1 Output: %0.3f, ", LPF1_output);
    float LPF2_output = voltage_gain/(1+(0.0015*LPF2_prevOutput));
    printf("LPF2 Output: %0.3f\n", LPF2_output);

    float error1 = power_gain - LPF1_output;
    printf("Error 1: %0.3f, ", error1);
    float error2 = voltage_gain - LPF2_output;
    printf("Error 2: %0.3f, ", error2);

    rcc1_input = error1 * error2;
    rcc1_setpoint = 0;
    PIDClass_compute(rcc1_pidClass);
    printf("RCC1 PID: %0.3f\n", rcc1_output);

    rcc2_input = voltage_gain;
    rcc2_setpoint = rcc1_output;
    PIDClass_compute(rcc2_pidClass);

    prevVoltage_gain = voltage_gain;
    prevPower_gain = power_gain;
    LPF1_prevOutput = LPF1_output;
    LPF2_prevOutput = LPF2_output;

    // printf("Voltage: %0.3f, Current: %0.3f, Duty Raw: %0.3f\n", voltage, current, duty_raw);

    // if (duty_raw >= duty_max || duty_raw <= duty_min) {
    //     duty = prevDuty;
    // }
    // else {
    //     duty = duty_raw;  
    // }
    // prevDuty = duty;
}
*/


/* END ALGORITHM FUNCTIONS */