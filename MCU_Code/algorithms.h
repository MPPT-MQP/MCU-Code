
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "lib/PID/src/PID_v1.h"

extern float duty;
extern float voltage;
extern float current;
extern float power;
extern float temperature;
extern float irradiance;
extern float rcc1_input;
extern float rcc1_setpoint;
extern float rcc1_output;
extern float rcc2_input;
extern float rcc2_setpoint;
extern float TMP_Vmpp;
extern void* cv_pidClass;
extern void* rcc1_pidClass;
extern void* rcc2_pidClass;
extern void* TMP_pidClass;

void perturb_and_observe(int variable);
void incremental_conductance(int variable);
void temperature_parametric();
void beta_method();
void particle_swarm_optimization();
void constant_voltage();
//void ripple_correlation_control();
void duty_sweep();
void algorithm_of_algorithms();
void selectAlgo(int algoToggleNum);

