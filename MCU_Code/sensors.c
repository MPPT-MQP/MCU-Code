#include <sensors.h>
#include "hardware/i2c.h"
#include <stdio.h>
#include "pico/stdlib.h"


//I2C Configuration
void configI2C0(){
    // I2C Initialisation. Using it at 300Khz.
    i2c_init(I2C_PORT, 300*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c
}


uint8_t sample[6];

void printManID(uint8_t address){
    uint8_t buffer[2];
    uint8_t reg = INA740_manufacturer_id_register;
    // uint8_t reg[1] = {INA740_config_register};
    i2c_write_blocking(I2C_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, address, buffer, 2, false);
    char text[2] = {buffer[0], buffer[1]};
    printf("\nSTATUS: %s", text);
}



//Power Monitor
uint32_t readVoltage(uint8_t address){
    uint16_t combinedBuffer;
    float voltage;
    uint8_t buffer[2];
    uint8_t reg = INA740_vbus_register;
    i2c_write_blocking(I2C_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, address, buffer, 2, false);
    
    //combine the two bytes
    //Combine bytes
    combinedBuffer = ((uint16_t)buffer[0] << 8) | buffer[1];

    voltage = (float)combinedBuffer;
    
    //scale factor
    voltage = (voltage * 3.125) / 1000;
    
    printf("\nVoltage: %f", voltage);
    
    return voltage;
}