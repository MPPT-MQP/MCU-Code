#include <stdio.h>
#include <time.h>
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
#include "pico/multicore.h"
#include "pico/time.h"
#include "def.h"

/*
* MCU_Code.c
* Main firmware to run MPPT algorithms, real-time data logging to an SD card, 
* hardware sensor readings, and an OLED display.
* Authors: Kyle Rabbitt, Frank Parsons, Saketh Dinasarapu, Micaela Tourtellot
* Organization: MPPT MQP @ WPI
*/

//PID class
void* cv_pidClass;
void* TMP_pidClass;

//Amount of time until alarm isr runs (flag is toggling between true and false)
//This is how often the screen sensor values update
#define ALARM_TIME_MS 1000

//Rate that the sensors and algorithm run
#define SENSOR_ALGORITHM_RUN_RATE 200000

//Doorbell to trigger interrupt on core 1
static uint32_t doorbellNumber;

//Mutex for temp sensor
mutex_t temperatureMutex;

//Structs for RTC and A_ON Pico Clock
struct pcf8523_time_t RTCtime;
struct tm PicoTime;
uint32_t elapsedtime = 0;

//Global for shared core queue
queue_t shareQueue;

//Flags sets high when SD card buffer is full
bool saveFlag = false;

//Flags to run algorithms or update OLED with sensor values (updated by ISRs with repeating timers)
volatile bool screenUpdateFlag = false;
volatile bool algoFlag = false;

//Sync flags so both cores wait until all inits have finished
volatile bool core1InitFlag = false;
volatile bool core0InitFlag = false;

//SD card bytes to save
uint32_t bytesToSave = SAMPLES_TO_SAVE * SAMPLE_SIZE;

//Current Algorithm
char selectedAlgo[5];

//Algo Selection and abbreviations
char algorithms[10][5] = {"CV", "B", "PNO", "PNOV", "INC", "INCV", "PSO", "TMP", "AofA", "DTY"};

//Initial variable to store the first temperature reading
float tempVAL;


/// @brief Select which algorithm to run
/// @param algoToggleNum Number of algorithm to run
void selectAlgo(int algoToggleNum){
    switch (algoToggleNum){
        case CV:
            constant_voltage();
            break;
        case B:
            beta_method();
            break;
        case PNO:
            //0 sets to P&O fixed step
            perturb_and_observe(0);
            break;
        case PNOV:
            //1 sets to P&O variable step
            perturb_and_observe(1);
            break;
        case INC:
            //0 sets to INC fixed step
            incremental_conductance(0);
            break;
        case INCV:
            //1 sets to INC variable step
            incremental_conductance(1);
            break;
        case PSO:
            particle_swarm_optimization();
            break;
        case TMP:
            temperature_parametric();
            break;
        case AofA:
            algorithm_of_algorithms();
            break;
        case DTY:
            duty_test();
        break;
    }
}

/// @brief Create PID controller instance for selected algorithm
/// @param algoToggleNum Algorithm number to initialize PID
void init_algo(int algoToggleNum){
    switch (algoToggleNum){
        case CV:
            //Change PID controller
            float cv_setpoint = 17.2;
            cv_pidClass = PIDClass_create(&voltage, &duty, &cv_setpoint, 0.035, 0.0001, 0, 1); // 0.01 0.1
            PIDClass_setOutputLimits(cv_pidClass, 0.1, 0.9);
            PIDClass_setMode(cv_pidClass, 1);
            break;
        case TMP:
            //TMP PID controller
            TMP_pidClass = PIDClass_create(&voltage, &duty, &TMP_Vmpp, 0.01, 0.1, 0, 1); // 0.01 0.1
            PIDClass_setOutputLimits(TMP_pidClass, 0.1, 0.9);
            PIDClass_setMode(TMP_pidClass, 1);
            break;
        default:
            //Other algorithm, just leave switch
            break;
    }
}


/// @brief ISR handler for the repeating timer on core 1 (Screen update).
// Return true from ISR to keep the repeating timer running
bool alarmISR(__unused repeating_timer_t *t){
    screenUpdateFlag = !screenUpdateFlag;
    return true;
}

/// @brief Set flag to true so that the sensor / algorithm loop runs
/// @param t Unused
/// @return true so the timer continues to repeat
bool AlgoISR(__unused repeating_timer_t *t){
    algoFlag = true;
    return true;
}

//Init LED on Pico if the oled screen is not used
#ifndef OLED_SCREEN
int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}
#endif


/// @brief Core 1 Main Function
void core1_main(){
    configI2C1();

    //Init Temp sensor registers
    initTMP102();

    //Init Mutex for temp sensor readings
    mutex_init(&temperatureMutex);

    //Take first temp reading if core 0 needs it
    //Mutex to prevent shared data problems with core 0 (block until ownership is claimed)
    mutex_enter_blocking(&temperatureMutex);
    tempVAL = readTMP102();
    //Set previous value from 0 to temp value if core 0 needs it on startup
    sensorBuffer[QUEUE_BUFFER_SIZE - 1].temperature = tempVAL;
    mutex_exit(&temperatureMutex);

    #ifndef OLED_SCREEN
    //LED Init
    pico_led_init();
    #endif

    /* RTC Initialization Options - Uncomment to set RTC */
    //pcf8523_set_manually(2025, 2, 16, 12, 45, 11);
    //pcf8523_set_from_PC();

    //Set Pico Clock to RTC values
    pcf8523_read(&RTCtime);
    PicoTime.tm_hour = RTCtime.hour;
    PicoTime.tm_min = RTCtime.minute;
    PicoTime.tm_sec = RTCtime.second;
    PicoTime.tm_year = RTCtime.year;
    PicoTime.tm_mon = RTCtime.month;
    PicoTime.tm_mday = RTCtime.day;
    aon_timer_start_calendar(&PicoTime);
    
    #ifdef OLED_SCREEN
    // Initialize OLED Screen and Display Welcome
    oled_init();
    welcome_screen();
    #endif

    //Initialize Alarm Pool and repeating timer for screen update (IRQ will call on core 1)
    struct repeating_timer alarmTimer;
    alarm_pool_t *core1Pool = alarm_pool_create_with_unused_hardware_alarm(2);
    alarm_pool_add_repeating_timer_ms(core1Pool, ALARM_TIME_MS, alarmISR, NULL, &alarmTimer);

    //Setup Buttons
    buttonsInit();

    //After all startup functions are called, wait for both cores to be done before running loops
    //Set flag and wait for both to be true
    core1InitFlag = true;
    while(core0InitFlag == false){
        //Wait for both flags to be true, then continue
        tight_loop_contents();
        printf("wait core 1");
    }

    while(1){
        #ifdef OLED_SCREEN
        run_main_screens();
        #endif

        //Create a new csv file when button 3 is pressed to start tracking
        if(initSDFlag == 1){
            initSDFlag = 0;
            initSDFile();
        }
        
        //Mutex to prevent shared data problems with core 0 (block until ownership is claimed)
        mutex_enter_blocking(&temperatureMutex);  
        tempVAL = readTMP102();
        mutex_exit(&temperatureMutex);
        
        //If OLED screen disabled, turn on LED when running algorithm and turn off when not
        #ifndef OLED_SCREEN
        if(tracking_toggle == 1){
            gpio_put(PICO_DEFAULT_LED_PIN, true);
        }else{
            gpio_put(PICO_DEFAULT_LED_PIN, false);
        }
        sleep_us(4);
        #endif
        
        //If algorithm is running, copy out of the queue into a local SD card buffer
        if(tracking_toggle == 1){
            copySDBuffer();
            if(saveFlag == true){
                //Once the SD card buffer is filled up to the defined size, save all data to the SD card
                writeSD(bytesToSave);
            }
        }

        /*
        If algorithm has been turned off before the SD card buffer is full,
        save the partial data in the SD card buffer
        */
        if(partialSaveFlag == 1){
            //Calculate the number of bytes to save to the SD card
            uint32_t localBytestoWrite = localSensorCounter * SAMPLE_SIZE;
            writeSD(localBytestoWrite);
            partialSaveFlag = 0;
        } 
    }
}

int main()
{
    stdio_init_all();

    //Init Queue
    queue_init(&shareQueue, SAMPLE_SIZE, QUEUE_BUFFER_SIZE);

    //Init both I2C0
    configI2C0();

    //Make sure SD card save flag is false before starting the core
    saveFlag = false;

    //Launch core 1 (OLED, SD Card, Temp Sensor, RTC)
    multicore_launch_core1(core1_main);
    
    //Pyranometer Sensor ADC Setup
    PYR_ADC_setup();

    //Configure PM for sample rate, averaging, etc
    PM_config(PM1);
    PM_config(PM2);
    PM_config(PM3);

    //SD Card Setup (hw_config.c sets the SPI pins)
    sd_init_driver();
    mountSD();

    //Init PWM
    pico_pwm_init();

    // Gate Driver Enable Pin Setup
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, false);

    //Add repeating timer to slow down the rate that the algorithm & sensor collection runs at
    struct repeating_timer algoTimer;
    add_repeating_timer_us(SENSOR_ALGORITHM_RUN_RATE, AlgoISR, NULL, &algoTimer);

    //Initialize any necessary PID controllers and values for desired algorithm to run
    init_algo(CV);
    init_algo(TMP);

    //Format name of CSV file to include current time and algorithm
    createCSVName(ALGO_TOGGLE);

    //Set name of algo to print in sd file
    snprintf(selectedAlgo, 5, "%s", algorithms[ALGO_TOGGLE]);

    //Set the init flag high and wait for the other core to finish setup
    core0InitFlag = true;
    while(core1InitFlag == false){
        //Do nothing and wait for both flags to be true
        tight_loop_contents();
        printf("loop");
    }
   
    while (true) {
        if(algoFlag){
            //Set flag back to false (how fast algo loop runs)
            algoFlag = false;

            //DEBUG: Power Monitor Status Printouts (Should print "TI")
            //PM_printManID(PM1);
            //PM_printManID(PM2);
            //PM_printManID(PM3);

            /* Collect Sensor Readings */

            //Read Power Monitor Values
            sensorBuffer[BufferCounter].PM1voltage = PM_readVoltage(PM1);
            sensorBuffer[BufferCounter].PM1current = PM_readCurrent(PM1);
            sensorBuffer[BufferCounter].PM1power = PM_readPower(PM1);

            sensorBuffer[BufferCounter].PM2voltage = PM_readVoltage(PM2);
            sensorBuffer[BufferCounter].PM2current = PM_readCurrent(PM2);
            sensorBuffer[BufferCounter].PM2power = PM_readPower(PM2);

            sensorBuffer[BufferCounter].PM3voltage = PM_readVoltage(PM3);
            sensorBuffer[BufferCounter].PM3current = PM_readCurrent(PM3);
            sensorBuffer[BufferCounter].PM3power = PM_readPower(PM3);

            //Read Irradiance Value from ADC
            sensorBuffer[BufferCounter].irradiance = readIrradiance();
            
            
            //Delay if tracking isn't running (so OLED has enough time to update since interrupt fires fast)
            if(tracking_toggle == 0){
                sleep_ms(100);
            }

            /*Temperature Sensor
                Note: Temperature sensor values are not updated every time data is collected
                Try to enter mutex
                If core 1 is currently in the mutex, use the previously collected value from the sensorBuffer
                (A wrap around counter is used to prevent any issues with getting the previous sensor value)
                If core 0 can enter the mutex, use the new temperature reading from core 1 and save it to the sensorBuffer
            */
            bool enterMutex = mutex_try_enter(&temperatureMutex, NULL);
            if(enterMutex == true){
                sensorBuffer[BufferCounter].temperature = tempVAL;
                mutex_exit(&temperatureMutex);
            }else if(enterMutex == false){
                uint8_t oldBufferCounter;
                if(BufferCounter == 0){
                    //Wrap around to avoid trying to read the -1 value in an array
                    oldBufferCounter = QUEUE_BUFFER_SIZE - 1;
                }else{
                    oldBufferCounter = BufferCounter - 1; 
                }
                sensorBuffer[BufferCounter].temperature = sensorBuffer[oldBufferCounter].temperature;
            }
                        
            /*End sensor loop*/

            /*Run Algorithm*/
            if (tracking_toggle == 1) {
                
                //Get starting timestamp for elapsed time
                absolute_time_t startTime;
                if(timeFlag == true){
                    startTime = get_absolute_time();
                    timeFlag = false;
                }

                //Turn on DC-DC Converter (enable pin)
                gpio_put(EN_PIN, true);

                //Copy sensor readings into globals for algorithm access
                voltage = sensorBuffer[BufferCounter].PM1voltage;
                current = sensorBuffer[BufferCounter].PM1current;
                power = sensorBuffer[BufferCounter].PM1power;
                temperature = sensorBuffer[BufferCounter].temperature;
                irradiance = sensorBuffer[BufferCounter].irradiance;

                //Run algorithm
                selectAlgo(ALGO_TOGGLE);

                //Set serial monitor to print formatted values if live plotting is not enabled
                #ifndef LIVE_PLOT
                    printf("Voltage: %0.3f, Current: %0.3f, Power: %0.3f, Duty: %0.3f, Irradiance: %0.3f, Temperature: %0.3f\n", 
                       voltage, current, power, duty, irradiance, temperature);
                #endif

                // Strip formatting so data is readable by realtime plotter
                #ifdef LIVE_PLOT
                    printf("%0.3f, %0.3f, %0.3f, %0.3f\n", voltage, current, power, duty); 
                #endif

                //Set DC-DC converter PWM duty cycle and enable it 
                pwm_set_chan_level(slice_num, PWM_CHAN_A, duty*DCDCFreq);
                
                //Get current timestamp and calculate elapsed time
                aon_timer_get_time_calendar(&PicoTime);

                absolute_time_t currTime = get_absolute_time();
                uint64_t elapsedTime_us = absolute_time_diff_us(startTime, currTime);
                uint32_t elaspedTime_ms = us_to_ms(elapsedTime_us);
                elapsedtime = elaspedTime_ms;

                //Sprintf to format sensor data
                char formatString[SAMPLE_SIZE];
                sprintf(formatString, "\n%02d:%02d:%02d, %i, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %s,", 
                PicoTime.tm_hour, PicoTime.tm_min, PicoTime.tm_sec, elaspedTime_ms,
                sensorBuffer[BufferCounter].PM1voltage, sensorBuffer[BufferCounter].PM1current, sensorBuffer[BufferCounter].PM1power, 
                sensorBuffer[BufferCounter].PM2voltage, sensorBuffer[BufferCounter].PM2current, sensorBuffer[BufferCounter].PM2power, sensorBuffer[BufferCounter].PM3voltage, 
                sensorBuffer[BufferCounter].PM3current, sensorBuffer[BufferCounter].PM3power, sensorBuffer[BufferCounter].temperature, sensorBuffer[BufferCounter].irradiance, duty, selectedAlgo);
            
                //Try to add the formatted string to the queue for core 1 to use
                //Returns false if the queue is full
                bool resultsAdd = queue_try_add(&shareQueue, &formatString);
                if(resultsAdd == false){
                    printf("\nCORE 0: Queue Full Add Error \n");
                }
            }else{
                //Turn off DC-DC Converter (if tracking_toggle = 0)
                gpio_put(EN_PIN, false);
            }
            /*End algorithm loop*/

            //Reset buffer counter
            if(BufferCounter++ >= QUEUE_BUFFER_SIZE){
                BufferCounter = 0;
            }

        }else{
            //If algoToggle is 0
            //Wait for flag to be high so main loop runs
            tight_loop_contents();
        }//End flag check
    }//End while loop
}//End main
