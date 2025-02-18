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

//Doorbell to trigger interrupt on core 1
static uint32_t doorbellNumber;

struct pcf8523_time_t RTCtime;
struct tm PicoTime;

queue_t shareQueue;
bool saveFlag = false;
bool screenUpdateFlag = false;
bool algoFlag = false;

//Sync flags so both cores wait until all inits have finished
bool core1InitFlag = false;
bool core0InitFlag = false;


/// @brief ISR handler for the repeating timer on core 1.
// Return true from ISR to keep the repeating timer running
bool alarmISR(__unused repeating_timer_t *t){
    screenUpdateFlag = !screenUpdateFlag;
    return true;
}

/// @brief Doorbell ISR handler - read external adc and clear the doorbell
void doorbellISR(){
    sensorBuffer[BufferCounter].irradiance = readExtADC();
    multicore_doorbell_clear_current_core(doorbellNumber);
}

bool AlgoISR(__unused repeating_timer_t *t){
    algoFlag = true;
    return true;
}

/// @brief Core 1 Main Function
void core1_main(){
    //Setup External ADC (ADS1115)
    doorbellNumber = multicore_doorbell_claim_unused(0x02, true);
    configExtADC((((((((CONFIG_DEFAULT & ~CONFIG_MUX_MASK) | CONFIG_MUX_AIN0_GND) & ~CONFIG_PGA_MASK) | CONFIG_PGA_4p096V) & ~CONFIG_MODE_MASK) | CONFIG_MODE_CONT) & ~CONFIG_DR_MASK) | CONFIG_DR_475SPS);
    
    //Init doorbell
    uint32_t irqNumber = multicore_doorbell_irq_num(doorbellNumber);
    irq_set_exclusive_handler(irqNumber, doorbellISR);
    irq_set_enabled(irqNumber, true);
    
    // Initialize OLED Screen and Display Welcome
    oled_init();
    welcome_screen();

    //Initialize Alarm Pool and repeating timer (IRQ will call on core 1)
    struct repeating_timer alarmTimer;
    alarm_pool_t *core1Pool = alarm_pool_create_with_unused_hardware_alarm(2);
    alarm_pool_add_repeating_timer_ms(core1Pool, ALARM_TIME_MS, alarmISR, NULL, &alarmTimer);

    //Setup Buttons
    buttonsInit();

    //Set flag and wait for both to be true
    core1InitFlag = true;
    while(core0InitFlag == false){
        //Wait for both flags to be true, then continue
        tight_loop_contents();
        printf("wait core 1");
    }

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

    // //Set Pico Clock
    // //pcf8523_set_manually(2025, 2, 16, 12, 45, 11);
    // //pcf8523_set_from_PC();
    // pcf8523_read(&RTCtime);
    // PicoTime.tm_hour = RTCtime.hour;
    // PicoTime.tm_min = RTCtime.minute;
    // PicoTime.tm_sec = RTCtime.second;
    // PicoTime.tm_year = RTCtime.year;
    // PicoTime.tm_mon = RTCtime.month;
    // PicoTime.tm_mday = RTCtime.day;
    // aon_timer_start_calendar(&PicoTime);

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
    // TMP_ADC_setup();

    //Init SD Card Setup (hw_config.c sets the SPI pins)
    sd_init_driver();
    mountSD();
    initSDFile();

    //Init PWM
    pico_pwm_init();
    
    // //Setup Buttons
    // buttonsInit();

    // Gate Driver Enable Pin Setup
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, false);

    // //Add Repeating timer to slow down the rate that the algoythm runs at
    // struct repeating_timer algoTimer;
    // add_repeating_timer_us(1000000, AlgoISR, NULL, &algoTimer);

    core0InitFlag = true;
    while(core1InitFlag == false){
        //Do nothing and wait for both flags to be true
        tight_loop_contents();
        printf("loop");
    }
   
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

        // //Temperature
        // sensorBuffer[BufferCounter].temperature = readTempature(2, 1);
        
        //Delay if tracking isn't running (so OLED has enough time to update since interrupt fires fast)
        if(tracking_toggle == 0){
            sleep_ms(100);
        }

        //Irradiance (trigger doorbell)
        multicore_doorbell_set_other_core(doorbellNumber);
        while(multicore_doorbell_is_set_other_core(doorbellNumber)){
            //Wait for light sensor reading to complete
            tight_loop_contents();
            
        }
        //printf("Irradiance: %0.3f", sensorBuffer[BufferCounter].irradiance);
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
        // if(algoFlag == true){
        //     algoFlag = false;
        if (tracking_toggle == 1) {
            sleep_ms(100);
            //Turn on DC-DC Converter
            gpio_put(EN_PIN, true);
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
            // Turn off DC-DC Converter 
            gpio_put(EN_PIN, false);
        }
        // }

    }
}
