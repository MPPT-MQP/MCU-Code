//Function Declarations for Sensors
#include "PowerMonitor.h"
#include "LightSensor.h"
#include <stdint.h>
#include "hardware/adc.h"
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C0_PORT i2c0
#define I2C0_SDA 4
#define I2C0_SCL 5

#define I2C1_PORT i2c1
#define I2C1_SDA 
#define I2C1_SCL 

//TEMP Defines
#define tmp_offset 0.5
#define tmp_scaling 0.01
#define TMP_NUM_SAMPLES 20 // Number of samples to average
#define TEMP_PIN 26


//Sensor Config I2C
void configI2C0();



//Power Monitor
float readVoltage(uint8_t address);
void printManID(uint8_t address);
float readCurrent(uint8_t address);

//Light Sensor
//TESTING A BRANCH

//Temp Sensor
void ADC_setup();
uint32_t readTempature(uint16_t num_samples, uint16_t sampleDelay);
