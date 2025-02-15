#include "f_util.h"
#include "ff.h"
#include "ffconf.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include "pico/aon_timer.h"


extern bool saveFlag;

void mountSD();
void initSDFile();
void copySDBuffer();
void writeSD(uint16_t bytes);

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


//Sensor Data Buffer
extern struct sensorData sensorBuffer[20];
extern uint16_t BufferCounter;

extern queue_t shareQueue;

//Extern PicoTime Struct
extern struct tm PicoTime;


