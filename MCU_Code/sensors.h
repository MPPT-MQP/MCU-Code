//Function Declarations for Sensors
#include "PowerMonitor.h"
#include "LightSensor.h"
#include <stdint.h>
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "ADS1115.h"

// I2C0 defines (GP Numbers, not pico pins) (Power monitors only)
#define I2C0_PORT i2c0
#define I2C0_SDA 20
#define I2C0_SCL 21

// #define I2C0_SDA 4
// #define I2C0_SCL 5

// I2C1 defines (OLED screen, light sensor, pyranometer)
#define I2C1_PORT i2c1
#define I2C1_SDA 10
#define I2C1_SCL 11

//Tempature Defines
#define tmp_offset 0.5
#define tmp_scaling 0.01
#define TMP_NUM_SAMPLES 20 // Number of samples to average
#define TEMP_PIN 26
#define TEMP_SAMPLE_DELAY 0 //sleep delay between adc reads in ms

//PM Address Defines
#define PM1 0x40
#define PM2 0x41
#define PM3 0x43

//External ADC (ADS1115) Defines
#define EXT_ADC_ADDDRESS 0x48
#define CONVFACTOR 125 //(uV/ LSB)

//PWM PIN
#define PWM_PIN 28

//Sensor Config I2C both channels
void configI2C();

//Power Monitor
float PM_readVoltage(uint8_t address);
void PM_printManID(uint8_t address);
float PM_readCurrent(uint8_t address);

//Pyranometer
float readPyranometer(float voltage);

//Temp Sensor
void TMP_ADC_setup();
float readTempature(uint16_t num_samples, uint16_t sampleDelay);

// PWM Generator
void pico_pwm_init();

//External ADC
void configExtADC(uint16_t register);
float readExtADC();
