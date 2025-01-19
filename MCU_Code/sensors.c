#include <sensors.h>
#include "hardware/i2c.h"
#include <stdio.h>
#include "pico/stdlib.h"


//I2C Configuration
void configI2C(){
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c
}

//Power Monitor
int32_t readVoltage(int deviceNum){

}