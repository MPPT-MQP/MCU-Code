#include "algorithms.h"
#include "math.h"

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