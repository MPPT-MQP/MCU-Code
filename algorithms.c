
//Global Variables
float duty_cycle = 0.5;
float duty_min = 0;
float duty_max = 0.95;
float P_O_step = 0.03;

float duty;
float voltage;
float current;
float power;

float prevDuty;
float prevVoltage;
float prevCurrent;
float prevPower;

// Function below for each algorithm

float perturb_and_observe(){

    power = voltage * current;
    float deltaV = voltage - prevVoltage;
    float deltaP = power - prevPower;

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

    return duty;
}

