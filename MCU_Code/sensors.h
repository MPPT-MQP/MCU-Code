//Function Declarations for Sensors
#include "PowerMonitor.h"
#include <stdint.h>
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "TMP102.h"

// I2C0 defines (GP Numbers, not pico pins) (Power monitors only)
#define I2C0_PORT i2c0
#define I2C0_SDA 20
#define I2C0_SCL 21

// I2C1 defines (OLED screen, light sensor, pyranometer)
#define I2C1_PORT i2c1
#define I2C1_SDA 10
#define I2C1_SCL 11

//Pyranometer Deifines
#define PYR_PIN 26

//PM Address Defines
#define PM1 0x40
#define PM2 0x41
#define PM3 0x43

// DC-DC Converter Pins
#define PWM_PIN 28
#define EN_PIN 27

// PCF8523 RTC Address Define
#define PCF8523_ADDRESS 0x68

//PWM 
extern uint slice_num;

// Grab MSB or LSB from 16 bit value
#define MSB(u16) (((u16) & 0xFF00U) >> 8)
#define LSB(u16) ((u16) & 0xFFU)

//Sensor Config I2C both channels
void configI2C0();
void configI2C1();

//Power Monitor
float PM_readVoltage(uint8_t address);
void PM_printManID(uint8_t address);
float PM_readCurrent(uint8_t address);
float PM_readPower(uint8_t address);
void PM_config(uint8_t address);

//Pyranometer
float readPyranometer(float voltage);
float readIrradiance();
void PYR_ADC_setup();

// PWM Generator
void pico_pwm_init();

// Time Struct for PCF8523 RTC
struct pcf8523_time_t {
  int8_t second;
  int8_t minute;
  int8_t hour;
  int8_t day;
  int8_t month;
  // last two digits of the year eg 23 for 2023
  // this will fall over in 2100
  int8_t year;
  //0..6, 0 is Sunday
  int8_t dotw;
};

extern struct pcf8523_time_t pcf_datetime;

// PCF8523
void pcf8523_reset();
void pcf8523_write(struct pcf8523_time_t *time);
void pcf8523_read_raw(uint8_t *buffer);
void pcf8523_read(struct pcf8523_time_t *time);
void pcf8523_raw_to_time(uint8_t *raw_time, struct pcf8523_time_t *time);
void pcf8523_time_to_raw(struct pcf8523_time_t *time, uint8_t *raw);
void pcf8523_set_from_PC();
void pcf8523_set_manually(int year, int month, int day, int hour, int minute, int second);

//TMP102 Temperature Sensor
void initTMP102();
float readTMP102();

