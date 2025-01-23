//Function Declarations for Sensors
#include <powerMonitor.h>
#include <stdint.h>
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C0_SDA 4
#define I2C0_SCL 5



//Sensor Config I2C
void configI2C0();



//Power Monitor
float readVoltage(uint8_t address);
void printManID(uint8_t address);
float readCurrent(uint8_t address);

//Light Sensor


//Temp Sensor


//OLED Screen??


//SD Card??