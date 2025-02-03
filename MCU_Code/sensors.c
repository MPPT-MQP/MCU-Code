#include "sensors.h"

//Sensor Globals

// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
const float conversion_factor = 3.3f / (1 << 12);



/// @brief Configure I2C0 and I2C1 at specified ports, pins, and speeds
void configI2C(){
    // I2C0 Initialisation. Using it at 300Khz.
    i2c_init(I2C0_PORT, 300*1000);
    
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // I2C1 Initialisation. Using it at 300Khz.
    i2c_init(I2C1_PORT, 300*1000);
    
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);
}

/*Power Monitor Functions*/

/// @brief Print TI by reading the status register of the power monitor
/// @param address I2C address of the power monitor
void PM_printManID(uint8_t address){
    uint8_t buffer[2];
    uint8_t reg = INA740_manufacturer_id_register;
    // uint8_t reg[1] = {INA740_config_register};
    i2c_write_blocking(I2C0_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C0_PORT, address, buffer, 2, false);
    char text[2] = {buffer[0], buffer[1]};
    printf("\nSTATUS: %s", text);
}



/// @brief Read the power monitor voltage
/// @param address I2C address of the power monitor
/// @return voltage read by monitor
float PM_readVoltage(uint8_t address){
    uint16_t combinedBuffer;
    float voltage;
    uint8_t buffer[2];
    uint8_t reg = INA740_vbus_register;
    i2c_write_blocking(I2C0_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C0_PORT, address, buffer, 2, false);
    
    //combine the two bytes
    //Combine bytes
    combinedBuffer = ((uint16_t)buffer[0] << 8) | buffer[1];

    voltage = (float)combinedBuffer;
    
    //scale factor
    voltage = (voltage * 3.125) / 1000;
    
    // printf("\nVoltage: %f", voltage);
    
    return voltage;
}

/// @brief Read power monitor current
/// @param address I2C address of the power monitor
/// @return current read by monitor
float PM_readCurrent(uint8_t address){
    uint16_t combinedBuffer;
    float current;
    uint8_t buffer[2];
    uint8_t reg = INA740_current_register;
    i2c_write_blocking(I2C0_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C0_PORT, address, buffer, 2, false);
    
    //combine the two bytes
    //Combine bytes
    combinedBuffer = ((uint16_t)buffer[0] << 8) | buffer[1];

    current = (float)combinedBuffer;
    
    //scale factor (1.2mA / LSB)
    current = (current * 1.2) / 1000;
    
    // printf("\nCurrent: %f", current);
    
    return current;
}

/*End Power Monitor Functions*/

/*Temperature ADC Reading and Conversion*/

void TMP_ADC_setup(){
    adc_init(); 

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(TEMP_PIN);
}

/// @brief Read the temperature of the TMP36
/// @param num_samples number of samples to read from the ADC
/// @param sampleDelay delay in ms between samples
/// @return average temperature value in celcius
float readTempature(uint16_t num_samples, uint16_t sampleDelay){

    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

    uint32_t sum = 0; // Variable to store the sum of ADC readings

    // Take multiple ADC readings and sum them
    for (int i = 0; i < num_samples; i++) {
        sum += adc_read();
        sleep_ms(sampleDelay); // Small delay between samples
    }

    // Calculate the average result
    uint16_t avg_result = sum / num_samples;

    // Calculate voltage and temperature using the averaged result
    float voltage = avg_result * conversion_factor;
    float temperature = (voltage - tmp_offset) / tmp_scaling;

    // // Print the averaged values
    // printf("Average Raw value: 0x%03x, voltage: %f V, temperature: %f °C\n", avg_result, voltage, temperature);

    return temperature;
}

/*End Temperature ADC Functions*/

/*Pyranometer Functions*/

/*End Pyranometer Functions*/

/*Light Sensor Functions*/

/*End Light Sensor Functions*/

/* PWM Init Function*/

// PWM Usage Function
// pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
// where duty cycle is out of 3125

void pico_pwm_init(){
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    // Set frequency to 40kHz
    pwm_set_wrap(slice_num, 3125);

    pwm_set_enabled(slice_num, true);
}

/* End PWM Init Function*/

/*External ADC*/

/// @brief Init ADS1115 and write desired config register
/// @param registerADC config register bits (16) define using masks and bit selectors
/// @param i2cPort I2C 0 or 1 on Pico 2
void configExtADC(uint16_t registerADC, i2c_inst_t i2cPort){
    
    uint8_t registerOut[2];
    registerOut[0]= (uint8_t)(registerADC >> 8); //bit shift MSB 8 bits into 8 bit value
    registerOut[1] = (uint8_t)(registerADC & 0xff); //take LSB 8 bits into 8 bit value
    
    // Select config register, then write 16 bits to register
    i2c_write_blocking(&i2cPort, EXT_ADC_ADDDRESS, registerOut, 3, false);
    
}

/// @brief Read ADS1115 voltage over I2C
/// @param i2cPort I2C 0 or 1 on Pico 2
/// @return voltage as a float
float readExtADC(i2c_inst_t i2cPort){
    uint16_t combinedBuffer;
    float voltage;
    uint8_t buffer[2];

    uint8_t reg1 = CONVERSION_ADDRESS;

    //select conversion buffer and read 16 bits from it
    i2c_write_blocking(&i2cPort, EXT_ADC_ADDDRESS, &reg1, 1, false);
    i2c_read_blocking(&i2cPort, EXT_ADC_ADDDRESS, buffer, 2, false);

    //Combine the two bytes (MSB is recieved first)
    combinedBuffer = ((uint16_t)buffer[0] << 8) | buffer[1];

    voltage = (float)combinedBuffer;
    
    //scale factor (125uV / LSB) **At specific FSR of 4.096V only**
    voltage = (voltage * CONVFACTOR) / 1000000;

    printf("\nVoltage: %f", voltage);
    printf("   |   Buffer: %X, %u", combinedBuffer);

    return readPyranometer(voltage);
}

/*End external ADC (ADS1115)*/

/*Pyranometer Functions*/
const float voltage_multiplier = 427.8;
float readPyranometer(float voltage){
    return (voltage * voltage_multiplier);
}
/*End Pyranometer Functions*/