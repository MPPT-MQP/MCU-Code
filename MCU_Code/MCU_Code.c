#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "sensors.h"
#include "sdCard.h"
#include "sd_card.h"
#include "buttons.h"
#include "oled_screen.h"
#include "user_interface.h"
#include "pico/util/queue.h"
#include "algorithms.h"
#include <time.h>
#include "pico/multicore.h"

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

struct tm date[800];
queue_t shareQueue;
bool saveFlag = false;

/// @brief Core 1 Main Function
void core1_main(){
    // Initialize OLED Screen and Display Welcome
    oled_init();
    welcome_screen();

    //Setup Buttons
    buttonsInit();


    while(1){
        run_main_screens();
        copySDBuffer();

        if(saveFlag == true){
            writeSD(64800);
        }
    }
}

int main()
{
    stdio_init_all();
    saveFlag = false;
    //Launch core 1 (OLED, SD Card)
    multicore_launch_core1(core1_main);

    // // Timer example code - This example fires off the callback after 2000ms
    // add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks
    
    /*Start of non example code*/
    //Init both I2C0 and I2C1
    configI2C();

    /* RTC Initialization Options - Uncomment to set RTC */
    // pcf8523_set_from_PC();
    // pcf8523_set_manually(2025, 2, 13, 9, 34, 23);

    // // Initialize OLED Screen and Display Welcome
    // oled_init();
    // welcome_screen();
    
    // //Temp Sensor ADC Setup
    TMP_ADC_setup();

    //Init SD Card Setup (hw_config.c sets the SPI pins)
    sd_init_driver();
    mountSD();
    initSDFile();

    //Init PWM
    pico_pwm_init();
    
    // //Setup Buttons
    // buttonsInit();

    //Setup External ADC (ADS1115)
    configExtADC((((((((CONFIG_DEFAULT & ~CONFIG_MUX_MASK) | CONFIG_MUX_AIN0_GND) & ~CONFIG_PGA_MASK) | CONFIG_PGA_4p096V) & ~CONFIG_MODE_MASK) | CONFIG_MODE_CONT) & ~CONFIG_DR_MASK) | CONFIG_DR_475SPS);
    
    //Init Queue
    queue_init(&shareQueue, 90, 20);
    aon_timer_start_with_timeofday();
    struct tm time;

    //set enable pin high
    gpio_init(27);
    gpio_set_dir(27, GPIO_OUT);
    gpio_put(27, false);
   
    while (true) {
        
        
        // PM_printManID(PM1);
        // PM_printManID(PM2);
        // PM_printManID(PM3);

        /* Sensor Loop*/

        // //Power Monitors
        sensorBuffer[BufferCounter].PM1voltage = PM_readVoltage(PM1);
        sensorBuffer[BufferCounter].PM1current = PM_readCurrent(PM1);

        sensorBuffer[BufferCounter].PM2voltage = PM_readVoltage(PM2);
        sensorBuffer[BufferCounter].PM2current = PM_readCurrent(PM2);

        // sensorBuffer[BufferCounter].PM3voltage = PM_readVoltage(PM3);
        // sensorBuffer[BufferCounter].PM3current = PM_readCurrent(PM3);

        //Temperature
        sensorBuffer[BufferCounter].temperature = readTempature(2, 1);
        
        //Irradiance
        sensorBuffer[BufferCounter].irradiance = readExtADC();
        // Returns irradance converted voltage value


        // //Write inital header data below
        // printf("\nTimestamp, PM1 (V), PM1(I), PM1(W), PM2 (V), PM2(I), PM2(W), PM3 (V), PM3(I), PM3(W), Temp (C), Light (W/m^2)");
        
        // //Write row data
        // printf("\nTIME, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
        // sensorBuffer[BufferCounter].PM1voltage, sensorBuffer[BufferCounter].PM1current, sensorBuffer[BufferCounter].PM1power, 
        // sensorBuffer[BufferCounter].PM2voltage, sensorBuffer[BufferCounter].PM2current, sensorBuffer[BufferCounter].PM2power, sensorBuffer[BufferCounter].PM3voltage, 
        // sensorBuffer[BufferCounter].PM3current, sensorBuffer[BufferCounter].PM3power, sensorBuffer[BufferCounter].temperature, sensorBuffer[BufferCounter].irradiance);

        //Sprintf to format the data
        char formatString[32];
        aon_timer_get_time_calendar(&time);
        sprintf(formatString, "\n%02d:%02d:%02d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
        time.tm_hour, time.tm_min, time.tm_sec,
        sensorBuffer[BufferCounter].PM1voltage, sensorBuffer[BufferCounter].PM1current, sensorBuffer[BufferCounter].PM1power, 
        sensorBuffer[BufferCounter].PM2voltage, sensorBuffer[BufferCounter].PM2current, sensorBuffer[BufferCounter].PM2power, sensorBuffer[BufferCounter].PM3voltage, 
        sensorBuffer[BufferCounter].PM3current, sensorBuffer[BufferCounter].PM3power, sensorBuffer[BufferCounter].temperature, sensorBuffer[BufferCounter].irradiance);

        queue_try_add(&shareQueue, &formatString);

        if(BufferCounter++ > 20){
            BufferCounter = 0;
        }
        /*End sensor loop*/

        /*Run Algorithm*/
        if (tracking_toggle == 1) {
            gpio_put(27, true);
            voltage = sensorBuffer[BufferCounter-1].PM1voltage;
            current = sensorBuffer[BufferCounter-1].PM1current;
            //power = sensorBuffer[BufferCounter-1].PM1power;
            power = voltage * current;
            temperature = sensorBuffer[BufferCounter-1].temperature;
            irradiance = sensorBuffer[BufferCounter-1].irradiance;
            printf("\nAlgorithm Values: %0.2f, %0.2f, %0.2f, %0.4f\n", voltage, current, power, duty);
            perturb_and_observe(0);
            pwm_set_chan_level(slice_num, PWM_CHAN_A, duty*3125);
        }
        else {
            gpio_put(27, false);
        }

    }
}
