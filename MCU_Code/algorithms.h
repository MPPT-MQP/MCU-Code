
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

extern float duty;
extern float voltage;
extern float current;
extern float power;
extern float temperature;
extern float irradiance;

void perturb_and_observe(int variable);
void incremental_conductance(int variable);
void temperature_parametric();
void beta_method();
void particle_swarm_optimization();
void constant_voltage();
void ripple_correlation_control();
void duty_sweep();