#include "f_util.h"
#include "ff.h"
#include "ffconf.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"


void mountSD();

struct sensorData {
    float PM1voltage;
    float PM1current;
    float PM1power;
    float PM2voltage;
    float PM2current;
    float PM2power;
    float PM3voltage;
    float PM3current;
    float PM3power;
    float temperature;
    float irradiance;
};