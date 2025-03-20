#include "f_util.h"
#include "ff.h"
#include "ffconf.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include "pico/aon_timer.h"

#define SAMPLES_TO_SAVE 300
#define SAMPLE_SIZE 150
#define QUEUE_BUFFER_SIZE 20    


extern bool saveFlag;

void mountSD();
void initSDFile();
void copySDBuffer();
void writeSD(uint32_t bytes);
void createCSVName(int algoToggleNum);

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
extern struct sensorData sensorBuffer[QUEUE_BUFFER_SIZE];
extern uint16_t BufferCounter;
extern uint16_t localSensorCounter;

extern queue_t shareQueue;

//Extern Time Structs
extern struct pcf8523_time_t RTCtime;
extern struct tm PicoTime;


