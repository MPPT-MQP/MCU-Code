//Function Declarations for Sensors

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9


//Sensor Config I2C
void configI2C();



//Power Monitor
int32_t readVoltage(int deviceNum);

//Light Sensor


//Temp Sensor


//OLED Screen??


//SD Card??