
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
extern float TMP_Vmpp;
extern void* cv_pidClass;
extern void* TMP_pidClass;
// extern float test_irradiance;
// extern float test_temperature;

void perturb_and_observe(int variable);
void incremental_conductance(int variable);
void temperature_parametric();
void beta_method();
void particle_swarm_optimization();
void constant_voltage();
void duty_test();
void algorithm_of_algorithms();
void selectAlgo(int algoToggleNum);

