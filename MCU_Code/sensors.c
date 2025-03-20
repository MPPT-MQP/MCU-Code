#include "sensors.h"
#include "sdCard.h"
#include <string.h>
#include "def.h"

struct pcf8523_time_t pcf_datetime;


/// @brief Configure I2C0 and I2C1 at specified ports, pins, and speeds
void configI2C0(){
    // I2C0 Initialisation. Using it at 300Khz.
    i2c_init(I2C0_PORT, 300*1000);
    
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c
}

void configI2C1(){
    // I2C1 Initialisation. Using it at 300Khz.
    i2c_init(I2C1_PORT, 400*1000); //might have to run at 100kHz for screen
    
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

    // Convert for 2's compliment and signed value
    if(combinedBuffer > 0x7FFF){
        current = (float)combinedBuffer - 0x10000;
    } 
    else {
        current = (float)combinedBuffer;
    }

    //scale factor (1.2mA / LSB)
    current = (current * 1.2) / 1000;
    
    // printf("\nCurrent: %f", current);
    
    return current;
}

/// @brief Read power monitor power
/// @param address I2C address of the power monitor
/// @return power read by monitor
float PM_readPower(uint8_t address){
    uint32_t combinedBuffer = 0;
    float power;
    uint8_t buffer[3];
    uint8_t reg = INA740_power_register;
    i2c_write_blocking(I2C0_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C0_PORT, address, buffer, 3, false);
    
    //Combine the three bytes
    combinedBuffer = ((uint16_t)buffer[0] << 16) | ((uint16_t)buffer[1] << 8) | buffer[2];

    // Value is returned unsigned and always positive
    power = (float)combinedBuffer;
    //scale factor (240uW / LSB)
    power = (power * 240) / 1000000;
    
    // printf("\nPower: %f", power);
    
    return power;
}

/// @brief Config power monitor settings
/// @param address I2C address of the power monitor
void PM_config(uint8_t address){
    uint16_t INAsettings;
    float power;
    uint8_t reg[3];
    reg[0] = INA740_adc_config_register;

    //DEFAULT CONFIGURATION
    // INAsettings = INA740_adc_config_register_mode_Continuoustemperaturecurrentandbusvoltage |
    //          INA740_adc_config_register_vbusct_1052us |
    //          INA740_adc_config_register_vsenct_1052us |
    //          INA740_adc_config_register_tct_1052us |
    //          INA740_adc_config_register_avg_1;

    INAsettings =   INA740_adc_config_register_mode_Continuoustemperaturecurrentandbusvoltage |
                    INA740_adc_config_register_vbusct_1052us |
                    INA740_adc_config_register_vsenct_1052us |
                    INA740_adc_config_register_tct_1052us |
                    INA740_adc_config_register_avg_64;

    reg[1] = MSB(INAsettings);
    reg[2] = LSB(INAsettings);
    i2c_write_blocking(I2C0_PORT, address, reg, 2, true);

}

/*End Power Monitor Functions*/

/*Temperature ADC Reading and Conversion*/

void TMP_ADC_setup(){
    adc_init(); 

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(TEMP_PIN);
}

// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
const float conversion_factor = 3.3f / (1 << 12);
/// @brief Read the temperature of the TMP36
/// @param num_samples number of samples to read from the ADC
/// @param sampleDelay delay in ms between samples
/// @return average temperature value in celsius
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

/* PWM Init Function*/

// PWM Usage Function
// pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
// where duty cycle is out of 3125
uint slice_num;

void pico_pwm_init(){
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

    slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    // Set frequency to 40kHz
    pwm_set_wrap(slice_num, DCDCFreq);

    pwm_set_enabled(slice_num, true);
}

/* End PWM Init Function*/

/*External ADC*/

/// @brief Init ADS1115 and write desired config register
/// @param registerADC config register bits (16) define using masks and bit selectors
/// @param i2cPort I2C 0 or 1 on Pico 2
void configExtADC(uint16_t registerADC){
    
    uint8_t registerOut[3];
    registerOut[0] = CONFIG_ADDRESS;
    registerOut[1]= (uint8_t)(registerADC >> 8); //bit shift MSB 8 bits into 8 bit value
    registerOut[2] = (uint8_t)(registerADC & 0xff); //take LSB 8 bits into 8 bit value
    
    // Select config register, then write 16 bits to register
    i2c_write_blocking(I2C1_PORT, EXT_ADC_ADDDRESS, registerOut, 3, false);
    
}

/// @brief Read ADS1115 voltage over I2C
/// @param i2cPort I2C 0 or 1 on Pico 2
/// @return voltage as a float
float readExtADC(){
    uint16_t combinedBuffer;
    float voltage;
    uint8_t buffer[2];

    uint8_t reg1 = CONVERSION_ADDRESS;

    //select conversion buffer and read 16 bits from it
    i2c_write_blocking(I2C1_PORT, EXT_ADC_ADDDRESS, &reg1, 1, false);
    i2c_read_blocking(I2C1_PORT, EXT_ADC_ADDDRESS, buffer, 2, false);

    //Combine the two bytes (MSB is received first)
    combinedBuffer = ((uint16_t)buffer[0] << 8) | buffer[1];

    voltage = (float)combinedBuffer;
    
    //scale factor (125uV / LSB) **At specific FSR of 4.096V only**
    voltage = (voltage * CONVFACTOR) / 1000000;

    // printf("\nVoltage: %f", voltage);
    // printf("   |   Buffer: %X, %u", combinedBuffer);

    return readPyranometer(voltage);
}

/*End external ADC (ADS1115)*/

/*Pyranometer Functions*/
// const float voltage_multiplier = 427.8;
const float voltage_multiplier = 547.1956224;
float readPyranometer(float voltage){
    return (voltage * voltage_multiplier);
}
/*End Pyranometer Functions*/

/* PCF8523 RTC Functions*/
// From https://github.com/a-mueller/pico-lib-pcf8523/tree/main

void pcf8523_reset() {
  uint8_t buf[] = {0x00, 0x58};
  i2c_write_blocking(I2C1_PORT, PCF8523_ADDRESS, buf, 2, false);

    //write to control 3
    uint8_t buf2[] = {0x02, 0x80}; //0x80 is default control3 and setting 100 to turn on battery switch-over function is enabled in standard mode;
    //battery low detection function is disabled
    i2c_write_blocking(I2C1_PORT, PCF8523_ADDRESS, buf2, 2, false);
}

void pcf8523_write(struct pcf8523_time_t *time) {
    // buf[0] is the register to write to
    // buf[1] is the value that will be written to the register
    uint8_t buf[2];

    //Write values for the current time in the array
    //index 0 -> second: bits 4-6 are responsible for the ten's digit and bits 0-3 for the unit's digit
    //index 1 -> minute: bits 4-6 are responsible for the ten's digit and bits 0-3 for the unit's digit
    //index 2 -> hour: bits 4-5 are responsible for the ten's digit and bits 0-3 for the unit's digit
    //index 3 -> day of the month: bits 4-5 are responsible for the ten's digit and bits 0-3 for the unit's digit
    //index 4 -> day of the week: where Sunday = 0x00, Monday = 0x01, Tuesday... ...Saturday = 0x06
    //index 5 -> month: bit 4 is responsible for the ten's digit and bits 0-3 for the unit's digit
    //index 6 -> year: bits 4-7 are responsible for the ten's digit and bits 0-3 for the unit's digit

    //NOTE: if the value in the year register is a multiple for 4, it will be considered a leap year and hence will include the 29th of February

    uint8_t current_val[7];
    pcf8523_time_to_raw(time, current_val);

    for (int i = 3; i < 10; ++i) {
        buf[0] = i;
        buf[1] = current_val[i - 3];
        i2c_write_blocking(I2C1_PORT, PCF8523_ADDRESS, buf, 2, false);
    }
}

void pcf8523_read_raw(uint8_t *buffer) {
    // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto incrementing
  // so we don't need to keep sending the register we want, just the first.

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x03;
  i2c_write_blocking(I2C1_PORT, PCF8523_ADDRESS, &val, 1, true); // true to keep master control of bus
  int result = i2c_read_blocking(I2C1_PORT, PCF8523_ADDRESS, buffer, 7, false);
  //return (result == 7);

}

/* Reads the current time from the RTC */
void pcf8523_read(struct pcf8523_time_t *time) {
  uint8_t raw_time[7];
  pcf8523_read_raw(raw_time);
  pcf8523_raw_to_time(raw_time, time);
}

/* Convert the raw bytes from the RTC into numbers we can understand */
void pcf8523_raw_to_time(uint8_t *raw_time, struct pcf8523_time_t *time) {
    time -> second = (10 * (int8_t) ((raw_time[0] & 0x70) >> 4)) + ((int8_t) (raw_time[0] & 0x0F));
    time -> minute = (10 * (int8_t) ((raw_time[1] & 0x70) >> 4)) + ((int8_t) (raw_time[1] & 0x0F));
    time -> hour = (10 * (int8_t) ((raw_time[2] & 0x30) >> 4)) + ((int8_t) (raw_time[2] & 0x0F));
    time -> day = (10 * (int8_t) ((raw_time[3] & 0x30) >> 4)) + ((int8_t) (raw_time[3] & 0x0F));
    time -> dotw = (int8_t) (raw_time[4] & 0x07);
    time -> month = (10 * (int8_t) ((raw_time[5] & 0x10) >> 4)) + ((int8_t) (raw_time[5] & 0x0F));
    time -> year = (10 * (int8_t) ((raw_time[6] & 0xF0) >> 4)) + ((int8_t) (raw_time[6] & 0x0F));
}

/* Convert the time from somewhere into raw bytes the RTC can understand */
void pcf8523_time_to_raw(struct pcf8523_time_t *time, uint8_t *raw) {
    raw[0] = ((time -> second / 10) << 4) | ((time -> second % 10) & 0x0F);
    raw[1] = ((time -> minute / 10) << 4) | ((time -> minute % 10) & 0x0F);
    raw[2] = ((time -> hour / 10) << 4) | ((time -> hour %10) & 0x0F);
    raw[3] = ((time -> day / 10) << 4) | ((time -> day % 10) & 0x0F);
    raw[4] = time -> dotw & 0x07;
    raw[5] = ((time -> month / 10) << 4) | ((time -> month % 10) & 0x0F);
    raw[6] = ((time -> year / 10) << 4) | ((time -> year % 10) & 0x0F);
}

/* Read current PC time from serial communication and set RTC accordingly */
// Need accompanying python script running on host PC 
void pcf8523_set_from_PC(){
    
    pcf8523_reset();
    char time_buffer[20];
    int year, month, day, hour, minute, second;

    fgets(time_buffer, sizeof(time_buffer), stdin);
    time_buffer[strcspn(time_buffer, "\n")] = 0; // Remove trailing newline
    
    sscanf(time_buffer, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);
  
    RTCtime.second = second+1;
    RTCtime.minute = minute;
    RTCtime.hour = hour;
    RTCtime.day = day;
    RTCtime.month = month;
    RTCtime.year = year-2000;
    //datetime.dotw = 2;
  
     pcf8523_write(&RTCtime);
}

/* Set RTC time manually upon upload*/
void pcf8523_set_manually(int year, int month, int day, int hour, int minute, int second) {

    pcf8523_reset();
    RTCtime.second = second+1;
    RTCtime.minute = minute;
    RTCtime.hour = hour;
    RTCtime.day = day;
    RTCtime.month = month;
    RTCtime.year = year-2000;

    pcf8523_write(&RTCtime);

}
/*End PCF8523 Functions*/

/*Start TMP102 Functions*/

void initTMP102(){
    uint8_t registerCONFIG[3];
    uint16_t test = TMP102_CONFIG_OS_DISABLE | \
    TMP102_CONFIG_F_2FAULTS | \
    TMP102_CONFIG_POL_LOW | \
    TMP102_CONFIG_TM_COMP | \
    TMP102_CONFIG_SD_CC | \
    TMP102_CONFIG_CR_4HZ | \
    TMP102_CONFIG_EM_12BIT;

    uint16_t tlow = 0xE200; /* Temperature Low Limit is -30 Celsius */
    uint16_t thigh = 0x5000; /* Temperature High Limit is 80 Celsius */
    registerCONFIG[0] = TMP102_CONFIG;
    registerCONFIG[1]= MSB(test);
    registerCONFIG[2] = LSB(test);
    // Select config register, then write 16 bits to register
    i2c_write_blocking(I2C1_PORT, TMP102_ADDRESS, registerCONFIG, 3, false);

    registerCONFIG[0] = TMP102_THIGH;
    registerCONFIG[1]= (uint8_t)(thigh >> 8); //bit shift MSB 8 bits into 8 bit value
    registerCONFIG[2] = (uint8_t)(thigh & 0xff); //take LSB 8 bits into 8 bit value
    i2c_write_blocking(I2C1_PORT, TMP102_ADDRESS, registerCONFIG, 3, false);

    registerCONFIG[0] = TMP102_TLOW;
    registerCONFIG[1]= (uint8_t)(tlow >> 8); //bit shift MSB 8 bits into 8 bit value
    registerCONFIG[2] = (uint8_t)(tlow & 0xff); //take LSB 8 bits into 8 bit value
    i2c_write_blocking(I2C1_PORT, TMP102_ADDRESS, registerCONFIG, 3, false);
}

float readTMP102(){
    uint8_t buffer[2];
    uint8_t tempRegister = TMP102_TEMP;
    i2c_write_blocking(I2C1_PORT, TMP102_ADDRESS, &tempRegister, 1, false);
    i2c_read_blocking(I2C1_PORT, TMP102_ADDRESS, buffer, 2, false);

    //Combine the two bytes (MSB is received first)
    uint16_t combinedBuffer = ((uint16_t)buffer[0] << 8) | buffer[1];
    combinedBuffer = combinedBuffer >> 4; //Shift the 4 LSB out, not used
    
    return ((float)combinedBuffer * 0.0625); //Multiply by scaling factor
}

/*End TMP102 Functions*/

/*Start ADC Function for Irradiance*/
float readIrradiance(){
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

    uint32_t adcReading = adc_read();

    // Calculate voltage and temperature using the averaged result
    float voltage = adcReading * conversion_factor;

    return readPyranometer(voltage);
}
/*End Irradiance Functions*/