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
#include "pico/time.h"

//Amount of time until alarm isr runs (flag is toggling between true and false)
//Set the time in ms to half of the desired update rate??
#define ALARM_TIME_MS 1000

struct pcf8523_time_t RTCtime;
struct tm PicoTime;

queue_t shareQueue;
bool saveFlag = false;
bool screenUpdateFlag = false;


/// @brief ISR handler for the repeating timer on core 1.
// Return true from ISR to keep the repeating timer running
bool alarmISR(__unused repeating_timer_t *t){
    printf("\nAlarm repeat timer ISR");
    screenUpdateFlag = !screenUpdateFlag;
    return true;
}

/// @brief Core 1 Main Function
void core1_main(){
    // Initialize OLED Screen and Display Welcome
    sleep_ms(50);
    oled_init();
    welcome_screen();

    //Initialize Alarm Pool and repeating timer (IRQ will call on core 1)
    struct repeating_timer alarmTimer;
    alarm_pool_t *core1Pool = alarm_pool_create_with_unused_hardware_alarm(2);
    alarm_pool_add_repeating_timer_ms(core1Pool, ALARM_TIME_MS, alarmISR, NULL, &alarmTimer);

    //Setup Buttons
    buttonsInit();

    printf("test core 1");

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

    //Init Queue
    queue_init(&shareQueue, 90, 20);

    /*Start of non example code*/
    //Init both I2C0 and I2C1
    configI2C();

    //Set Pico Clock
    pcf8523_read(&RTCtime);
    PicoTime.tm_hour = RTCtime.hour;
    PicoTime.tm_min = RTCtime.minute;
    PicoTime.tm_sec = RTCtime.second;
    PicoTime.tm_year = RTCtime.year;
    PicoTime.tm_mon = RTCtime.month;
    PicoTime.tm_mday = RTCtime.day;
    aon_timer_start_calendar(&PicoTime);

    saveFlag = false;
    //Launch core 1 (OLED, SD Card)
    multicore_launch_core1(core1_main);

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

    /* RTC Initialization Options - Uncomment to set RTC */
    // pcf8523_set_from_PC();
    // pcf8523_set_manually(2025, 2, 13, 9, 34, 23);

    // //Initialize OLED Screen and Display Welcome
    // oled_init();
    // welcome_screen();
    
    //Temp Sensor ADC Setup
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
    
    

    //set enable pin high????????????????????????????????????????????????????????????????
    gpio_init(27);
    gpio_set_dir(27, GPIO_OUT);
    gpio_put(27, false);
   
    while (true) {
        //Power Monitor Status Printouts (Should print "TI")
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
        
        if(BufferCounter++ > 20){
            BufferCounter = 0;
        }
        /*End sensor loop*/

        /*Run Algorithm*/
        if (button3_state == 1) {
            //IS THIS GPIO ENABLE??
            gpio_put(27, true);
            voltage = sensorBuffer[BufferCounter-1].PM1voltage;
            current = sensorBuffer[BufferCounter-1].PM1current;
            //power = sensorBuffer[BufferCounter-1].PM1power;
            power = voltage * current;
            //temperature = sensorBuffer[BufferCounter-1].temperature;
            //irradiance = sensorBuffer[BufferCounter-1].irradiance;
            
            perturb_and_observe(0);
            pwm_set_chan_level(slice_num, PWM_CHAN_A, duty*3125);
            
            //Sprintf to format sensor data
            char formatString[90];
            aon_timer_get_time_calendar(&PicoTime);
            sprintf(formatString, "\n%02d:%02d:%02d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
            PicoTime.tm_hour, PicoTime.tm_min, PicoTime.tm_sec,
            sensorBuffer[BufferCounter].PM1voltage, sensorBuffer[BufferCounter].PM1current, sensorBuffer[BufferCounter].PM1power, 
            sensorBuffer[BufferCounter].PM2voltage, sensorBuffer[BufferCounter].PM2current, sensorBuffer[BufferCounter].PM2power, sensorBuffer[BufferCounter].PM3voltage, 
            sensorBuffer[BufferCounter].PM3current, sensorBuffer[BufferCounter].PM3power, sensorBuffer[BufferCounter].temperature, sensorBuffer[BufferCounter].irradiance, duty);
            printf("\nAlgorithm Values: %0.2f, %0.2f, %0.2f, %0.4f\n", voltage, current, power, duty);
        
            //Returns false if the queue is full
            bool resultsAdd = queue_try_add(&shareQueue, &formatString);
            if(resultsAdd == false){
                printf("\nCORE 0: Queue Full Add Error \n");
            }
        }
        else {
            //WHAT IS THIS??
            gpio_put(27, false);
        }

    }
}
