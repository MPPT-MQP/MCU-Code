#include "user_interface.h"
#include "buttons.h"
#include "oled_screen.h"
#include <string.h>
#include <time.h>
#include "sdCard.h"
#include "sensors.h"
#include "def.h"

/*
* user_interface.c
* Operates a custom user interface on the SSD1306 OLED Screen
* Utilizes the SSD1306 library for Raspberry Pi Pico in oled_screen.c
*/

/* Variables to Keep Track of Settings*/

// Which screen is selected
int screen_num = 0;

// Which setting is selected
int select_num = 0;

// 0 = Tracking Off, 1 = Tracking On
int tracking_toggle = 0;

// 0 = CV, 1=B, 2=PNO, 3=PNOV, 4=INC, 5=INCV, 6=PSO, 7=TMP, 8=AofA, 9=DTY
int algorithm_toggle = 0;

 // Array for displaying sensor data
 char displayString1[15];
 char displayString2[15];

 // Array for displaying time and date
 char date_string[15];
 char time_string[15];

// Run boot-up screen
void welcome_screen()
{
    write_text(1, 0, " ");
    write_text(15, 8, "WPI MPPT MQP");
    write_text(30, 16, "2024-2025");
    write_text(1, 8, " ");
    refresh_screen();
    sleep_ms(3000);
}

// Main user interface loop
void run_main_screens()
{
    // Check button 1 (toggles entire screen)
    if (button1_state)
    {
        screen_num++;
        if (screen_num > 4)
        {
            screen_num = 0;
        }
        button1_state = !button1_state;
    }

    // Refresh display for new info
    clear_display();

    // Toggle between 5 possible screens
    switch (screen_num)
    {
    // Screen 1: Start/Stop Tracking, Select Algorithm, Elapsed Time
    case 0:

        // Check button 2 (toggles selected field on screen 1)
        if (button2_state)
        {
            select_num = !select_num;
            button2_state = !button2_state;
        }
        switch (select_num)
        {
        // Start/Stop Tracking
        case 0:
            // Check button 3 (toggles current tracking status)
            if (button3_state)
            {
                if(button3_state == 1){
                    partialSaveFlag = true;
                }
                if(button3_state == 0){
                    initSDFlag = true;
                }
                timeFlag = true;
                button3_state = !button3_state;
                tracking_toggle = !tracking_toggle;
            }
            // Add selector arrow if currently selected
            if(tracking_toggle) {
                // If currently tracking, show option to stop tracking
                write_text(1, 0, "STOP TRACKING<");
            }
            else {
                // If not currently tracking, show option to start tracking
                write_text(1, 0, "START TRACKING<");
            }
            // Write set algorithm to screen without selector arrow
            write_text(1, 8, "SET ALGORITHM");
            break;
        // Select Algorithm
        case 1:
            // Check button 3 (toggles current algorithm)
            if (button3_state)
            {
                algorithm_toggle++;
                if (algorithm_toggle > 9)
                {
                    algorithm_toggle = 0;
                }
                button3_state = !button3_state;
            }
            // Print start/stop tracking without selector arrow
            if(tracking_toggle) {
                write_text(1, 0, "STOP TRACKING");
            }
            else {
                write_text(1, 0, "START TRACKING");
            }
            // Print set algorithm with selector arrow
            write_text(1, 8, "SET ALGORITHM<");
            break;    
        }
        // Print currently selected algorithm
        write_text(25, 16, algorithms[algorithm_toggle]);
        // Print elapsed time in HH:MM:SS format
        if(tracking_toggle) {
            uint32_t seconds = (elapsedtime / 1000) % 60;
            uint32_t minutes = (elapsedtime / (1000 * 60)) % 60;
            uint32_t hours = (elapsedtime/ (1000 * 60 * 60));
            sprintf(time_string, "TIME: %02d:%02d:%02d", hours, minutes, seconds);
            write_text(1, 24, time_string);
        } else {
            sprintf(time_string, "TIME: %02d:%02d:%02d", 0, 0, 0);
            write_text(1, 24, time_string);
        }
        // Load current screen text into buffer to be displayed
        refresh_screen();
        break;

    // Screen 2: Temperature and Irradiance
    case 1:
        // Write current temp and irradiance measurements to array if screenUpdateFlag is 1
        if(screenUpdateFlag){
            sprintf(displayString1, "%0.2f*C", sensorBuffer[BufferCounter].temperature);
            sprintf(displayString2, "%0.2f W/m^2", sensorBuffer[BufferCounter].irradiance);
            screenUpdateFlag = !screenUpdateFlag;
        }
        // Write temp and irradiance measurements to screen with labels
        write_text(20, 0, "TEMPERATURE:");
        write_text(40, 8, displayString1);
        write_text(20, 16, "IRRADIANCE:");
        write_text(20, 24, displayString2);
        refresh_screen();
        break;
    
    // Screen 3: Power Monitor 1 and 2 
    case 2:
         // Write current PM1 and 2 measurements to array if screenUpdateFlag is 1
        if (screenUpdateFlag)
        {
            sprintf(displayString1, "%0.2fV, %0.2fA", sensorBuffer[BufferCounter].PM1voltage, sensorBuffer[BufferCounter].PM1current);
            sprintf(displayString2, "%0.2fV, %0.2fA", sensorBuffer[BufferCounter].PM2voltage, sensorBuffer[BufferCounter].PM2current);
            screenUpdateFlag = !screenUpdateFlag;
        }
        // Write PM1 and 2 measurements to screen with labels
        write_text(20, 0, "PM1 PV-Buck:");
        write_text(1, 8, displayString1);
        write_text(20, 16, "PM2 Buck-CC:");
        write_text(1, 24, displayString2);
        refresh_screen();
        break;

    // Screen 4: Power Monitor 3 and Battery SoC
    case 3:
        // Write current PM3 measurement and battery SoC to array if screenUpdateFlag is 1
        if (screenUpdateFlag)
        {
            // Calculate battery SoC (%) based on linear regression model 
            float batterySOC = 44.328 * (sensorBuffer[BufferCounter].PM3voltage) - 475.61;
            if (batterySOC < 0) {
                batterySOC = 0.00;
            }
            sprintf(displayString1, "%0.2fV, %0.2fA", sensorBuffer[BufferCounter].PM3voltage, sensorBuffer[BufferCounter].PM3current);
            sprintf(displayString2, "%0.2f%", batterySOC);
            screenUpdateFlag = !screenUpdateFlag;
        }
        // Write PM3 measurements and battery SoC to screen with labels
        write_text(10, 0, "PM3 CC-Battery:");
        write_text(1, 8, displayString1);
        write_text(20, 16, "BATTERY SOC:");
        write_text(40, 24, displayString2);
        refresh_screen();
        break;

    // Screen 5: Date & Time
    case 4:
        // Print current date and time to buffer to be displayed on screen
        aon_timer_get_time_calendar(&PicoTime);
        sprintf(date_string, "%02d-%02d-%02d", PicoTime.tm_year, PicoTime.tm_mon, PicoTime.tm_mday);
        sprintf(time_string, "%02d:%02d:%02d", PicoTime.tm_hour, PicoTime.tm_min, PicoTime.tm_sec);
        // Print current date and time to screen with label 
        write_text(1, 0, "DATE & TIME");
        write_text(1, 8, date_string);
        write_text(1, 16, time_string);
        refresh_screen();
        break;
    }
}