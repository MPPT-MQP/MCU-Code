//Function Declarations for Sensors
#include "PowerMonitor.h"
#include "LightSensor.h"
#include <stdint.h>
#include "hardware/adc.h"

// I2C0 defines (GP Numbers, not pico pins) (Power monitors only)
#define I2C0_PORT i2c0
#define I2C0_SDA 20
#define I2C0_SCL 21

// I2C1 defines (OLED screen, light sensor, pyranometer)
#define I2C1_PORT i2c1
#define I2C1_SDA 10
#define I2C1_SCL 11

//TEMP Defines
#define tmp_offset 0.5
#define tmp_scaling 0.01
#define TMP_NUM_SAMPLES 20 // Number of samples to average
#define TEMP_PIN 26
#define TEMP_SAMPLE_DELAY 0 //sleep delay between adc reads in ms


//Sensor Config I2C both channels
void configI2C();


//Power Monitor
float PM_readVoltage(uint8_t address);
void PM_printManID(uint8_t address);
float PM_readCurrent(uint8_t address);

//Light Sensor


//Pyranometer

//Temp Sensor
void TMP_ADC_setup();
uint32_t readTempature(uint16_t num_samples, uint16_t sampleDelay);
