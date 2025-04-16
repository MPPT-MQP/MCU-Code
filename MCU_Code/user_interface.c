#include "user_interface.h"
#include "buttons.h"
#include "oled_screen.h"
#include <string.h>
#include <time.h>
#include "sdCard.h"
#include "sensors.h"

/* Variables to keep track of which screen and setting is selected*/
int screen_num = 0;
int select_num = 0;

/*Toggle Variables to Keep Track of Settings*/
// 0 = Tracking Off, 1 = Tracking On
int tracking_toggle = 0;
// 0 = CV, 1 = Beta, 2= P&O, etc.
int algorithm_toggle = 0;
// 0 = Buck Only, 1 = Charge Controller Only, 2 = Both
int mode_toggle = 0;
// 0 = OFF, 1 = ON
int sd_card_toggle = 0;

// Run boot-up screen
void welcome_screen()
{
    write_text(1, 0, " ");
    write_text(15, 8, "WPI MPPT MQP");
    write_text(30, 16, "2024-2025");
    write_text(1, 8, " ");
    refresh_screen();
}

// Main user interface loop
void run_main_screens()
{
    // Array for displaying sensor data
    char displayString1[15];
    char displayString2[15];

    // Time Stuff
    char date_string[15];
    char time_string[15];

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

    // Toggle between 5 possible screen
    switch (screen_num)
    {
    case 0:

        // Check button 2 (toggles selected field on a screen)
        if (button2_state)
        {
            select_num = !select_num;
            button2_state = !button2_state;
        }
        switch (select_num)
        {
        case 0:
            // Check button 3 (toggles settings that's currently selected)
            if (button3_state)
            {
                tracking_toggle=!tracking_toggle;
                button3_state = !button3_state;
            }
            if(tracking_toggle) {
                write_text(1, 0, "STOP TRACKING<");
            }
            else {
                write_text(1, 0, "START TRACKING<");
            }
            write_text(1, 8, "SET ALGORITHM");
            break;

        case 1:
            if (button3_state)
            {
                algorithm_toggle++;
                if (algorithm_toggle > 9)
                {
                    algorithm_toggle = 0;
                }
                button3_state = !button3_state;
            }
            if(tracking_toggle) {
                write_text(1, 0, "STOP TRACKING");
            }
            else {
                write_text(1, 0, "START TRACKING");
            }
            write_text(1, 8, "SET ALGORITHM<");
            break;    
        }
        write_text(1, 16, algorithms[algorithm_toggle]);
        refresh_screen();
        break;

    case 1:
        //if(screenUpdateFlag){
            sprintf(displayString1, "%0.2f*C", sensorBuffer[BufferCounter].temperature);
            sprintf(displayString2, "%0.2f W/m^2", sensorBuffer[BufferCounter].irradiance);
            //screenUpdateFlag = !screenUpdateFlag;
        //}
        write_text(20, 0, "TEMPERATURE:");
        write_text(40, 8, displayString1);
        write_text(20, 16, "IRRADIANCE:");
        write_text(20, 24, displayString2);
        refresh_screen();
        break;

    case 2:
        //if (screenUpdateFlag)
        //{
            sprintf(displayString1, "%0.2fV, %0.2fA", sensorBuffer[BufferCounter].PM1voltage, sensorBuffer[BufferCounter].PM1current);
            sprintf(displayString2, "%0.2fV, %0.2fA", sensorBuffer[BufferCounter].PM2voltage, sensorBuffer[BufferCounter].PM2current);
            screenUpdateFlag = !screenUpdateFlag;
       // }
        write_text(20, 0, "PM1 PV-Buck:");
        write_text(1, 8, displayString1);
        write_text(20, 16, "PM2 Buck-CC:");
        write_text(1, 24, displayString2);
        refresh_screen();
        break;

    case 3:
        //if (screenUpdateFlag)
        //{
            float batterySOC;
            if (sensorBuffer[BufferCounter-1].PM3voltage == 0) {
                batterySOC = 0.00;
            } else {
                batterySOC = 44.328 * (sensorBuffer[BufferCounter].PM3voltage) - 475.61;
            }
            sprintf(displayString1, "%0.2fV, %0.2fA", sensorBuffer[BufferCounter].PM3voltage, sensorBuffer[BufferCounter].PM3current);
            sprintf(displayString2, "%0.2f%", batterySOC);
            screenUpdateFlag = !screenUpdateFlag;
       // }
        write_text(10, 0, "PM3 CC-Battery:");
        write_text(1, 8, displayString1);
        write_text(20, 16, "BATTERY SOC:");
        write_text(40, 24, displayString2);
        refresh_screen();
        break;

    case 4:
        // Print current time to buffer to be displayed on screen
        aon_timer_get_time_calendar(&PicoTime);
        sprintf(date_string, "%d-%d-%d", PicoTime.tm_year, PicoTime.tm_mon, PicoTime.tm_mday);
        sprintf(time_string, "%d:%d:%d", PicoTime.tm_hour, PicoTime.tm_min, PicoTime.tm_sec);

        if (button2_state)
        {
            select_num++;
            if (select_num > 6)
            {
                select_num = 0;
            }
            button2_state = !button2_state;
        }
        switch (select_num)
        {
        case 0:
            if (button3_state)
            {
                sd_card_toggle = !sd_card_toggle;
                button3_state = !button3_state;
            }
            if(sd_card_toggle) {
                write_text(1, 0, "SD CARD: ON<");
            }
            else {
                write_text(1, 0, "SD CARD: OFF<");
            }
            write_text(1, 8, "DATE & TIME");
            break;

        default:
            if(sd_card_toggle) {
                write_text(1, 0, "SD CARD: ON");
            }
            else {
                write_text(1, 0, "SD CARD: OFF");
            }
            switch (select_num-1) {
                case 0:
                    write_text(1, 8, "DATE & TIME MO");
                break;
                case 1:
                     write_text(1, 8, "DATE & TIME D");
                break;
                case 2:
                    write_text(1, 8, "DATE & TIME Y");
                break;
                case 3:
                    write_text(1, 8, "DATE & TIME H");
                break;
                case 4:
                    write_text(1, 8, "DATE & TIME MI");
                break;
                case 5:
                    write_text(1, 8, "DATE & TIME S");
            }
            // Check button 3 (increments currently selected date value)
            if (button3_state)
            {
                switch (select_num - 1)
                {
                case 0:
                    (PicoTime.tm_mon)++;
                    break;
                case 1:
                    (PicoTime.tm_mday)++;
                    break;
                case 2:
                    (PicoTime.tm_year)++;
                    break;
                case 3:
                    (PicoTime.tm_hour)++;
                    break;
                case 4:
                    (PicoTime.tm_min)++;
                case 5:
                    (PicoTime.tm_sec)++;
                    break;
                }
                button3_state = !button3_state;
            }
            // Check button 4 (decrements currently selected date value)
            if (button4_state)
            {
                switch (select_num - 1)
                {
                case 0:
                    (PicoTime.tm_mon)--;
                    break;
                case 1:
                    (PicoTime.tm_mday)--;
                    break;
                case 2:
                    (PicoTime.tm_year)--;
                    break;
                case 3:
                    (PicoTime.tm_hour)--;
                    break;
                case 4:
                    (PicoTime.tm_min)--;
                case 5:
                    (PicoTime.tm_sec)--;
                    break;
                }
                button4_state = !button4_state;
            } 
            break;
        }
        write_text(1, 16, date_string);
        write_text(1, 24, time_string);
        refresh_screen();
        break;
    }
}