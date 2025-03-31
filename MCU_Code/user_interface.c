#include "user_interface.h"
#include "buttons.h"
#include "oled_screen.h"
#include <string.h>
#include <time.h>
#include "sdCard.h"
#include "sensors.h"

// Time Stuff
char date_string[15];
char time_string[15];

/* Variables to keep track of which screen and setting is selected*/
int screen_num = 0;
int select_num = 0;

/* Initializing Text to Display on Screen*/
char current_tracking[15] = "START TRACKING";
char tracking_status[2][15] = {"START TRACKING", "STOP TRACKING"};
char current_algorithm[5] = "CV";
char current_mode[15] = "MODE: BUCK";
char mode[3][15] = {"MODE: BUCK", "MODE: CC", "MODE: BOTH"};
char current_sd_card[15] = "SD CARD: ON";
char sd_card[2][15] = {"SD CARD: OFF", "SD CARD: ON"};
char current_time_select[15] = "DATE & TIME";

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
    char screen1[4][15] = {" ", "WPI MPPT MQP", "2024-2025", " "};
    int x_distances1[4] = {1, 15, 30, 1};
    print_text(screen1, x_distances1);
}

// Main user interface loop
void run_main_screens()
{
    // Array for displaying sensor data
    char displayString1[15];
    char displayString2[15];

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
            select_num++;
            if (select_num > 2)
            {
                select_num = 0;
            }
            button2_state = !button2_state;
        }
        switch (select_num)
        {
        case 0:
            // Check button 3 (toggles settings that's currently selected)
            if (button3_state)
            {
                tracking_toggle++;
                if (tracking_toggle > 1)
                {
                    tracking_toggle = 0;
                }
                button3_state = !button3_state;
            }
            char tracking_status_selected[2][15] = {"START TRACKING<", "STOP TRACKING<"};
            sprintf(current_tracking, "%s", tracking_status_selected[tracking_toggle]);
            sprintf(current_algorithm, "%s", algorithms[algorithm_toggle]);
            sprintf(current_mode, "%s", mode[mode_toggle]);
            char screen0[4][15];
            sprintf(screen0[0], "%s", current_tracking);
            sprintf(screen0[1], "%s", "SET ALGORITHM");
            sprintf(screen0[2], "%s", current_algorithm);
            sprintf(screen0[3], "%s", current_mode);
            //= {current_tracking, "SET ALGORITHM", current_algorithm, current_mode};
            int x_distances0[4] = {1, 1, 1, 1};
            print_text(screen0, x_distances0);
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
            sprintf(current_tracking, "%s", tracking_status[tracking_toggle]);
            sprintf(current_algorithm, "%s", algorithms[algorithm_toggle]);
            sprintf(current_mode, "%s", mode[mode_toggle]);
            char screen01[4][15];
            sprintf(screen01[0], "%s", current_tracking);
            sprintf(screen01[1], "%s", "SET ALGORITHM<");
            sprintf(screen01[2], "%s", current_algorithm);
            sprintf(screen01[3], "%s", current_mode);
            // = {current_tracking, "SET ALGORITHM<", current_algorithm, current_mode};
            int x_distances01[4] = {1, 1, 1, 1};
            print_text(screen01, x_distances01);

            break;

        case 2:
            if (button3_state)
            {
                mode_toggle++;
                if (mode_toggle > 2)
                {
                    mode_toggle = 0;
                }
                button3_state = !button3_state;
            }
            char mode_selected[3][15] = {"MODE: BUCK<", "MODE: CC<", "MODE: BOTH<"};
            sprintf(current_tracking, "%s", tracking_status_selected[tracking_toggle]);
            sprintf(current_algorithm, "%s", algorithms[algorithm_toggle]);
            sprintf(current_mode, "%s", mode_selected[mode_toggle]);
            char screen02[4][15];
            sprintf(screen02[0], "%s", current_tracking);
            sprintf(screen02[1], "%s", "SET ALGORITHM");
            sprintf(screen02[2], "%s", current_algorithm);
            sprintf(screen02[3], "%s", current_mode);
            //= {current_tracking, "SET ALGORITHM", current_algorithm, current_mode};
            int x_distances02[4] = {1, 1, 1, 1};
            print_text(screen02, x_distances02);
            break;
        }

        break;

    case 1:
        //if(screenUpdateFlag){
            sprintf(displayString1, "%0.2f*C", sensorBuffer[BufferCounter].temperature);
            sprintf(displayString2, "%0.2f W/m^2", sensorBuffer[BufferCounter].irradiance);
            //screenUpdateFlag = !screenUpdateFlag;
        //}
        char screen1[4][15];
        sprintf(screen1[0], "%s", "TEMPERATURE:");
        sprintf(screen1[1], "%s", displayString1);
        sprintf(screen1[2], "%s", "IRRADIANCE:");
        sprintf(screen1[3], "%s", displayString2);
        //= {"TEMPERATURE:", displayString1, "IRRADIANCE:", displayString2}
        int x_distances1[4] = {20, 40, 20, 20};
        print_text(screen1, x_distances1);
        break;

    case 2:
        //if (screenUpdateFlag)
        //{
            sprintf(displayString1, "%0.2fV, %0.2fA", sensorBuffer[BufferCounter].PM1voltage, sensorBuffer[BufferCounter].PM1current);
            sprintf(displayString2, "%0.2fV, %0.2fA", sensorBuffer[BufferCounter].PM2voltage, sensorBuffer[BufferCounter].PM2current);
            screenUpdateFlag = !screenUpdateFlag;
       // }
        char screen2[4][15];
        sprintf(screen2[0], "%s", "PM1 PV-Buck:");
        sprintf(screen2[1], "%s", displayString1);
        sprintf(screen2[2], "%s", "PM2 Buck-CC:");
        sprintf(screen2[3], "%s", displayString2);
        //= {"PM1 PV-Buck:", displayString1, "PM2 Buck-CC:", displayString2};
        int x_distances2[4] = {20, 1, 20, 1};
        print_text(screen2, x_distances2);
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
        char screen3[4][15];
        sprintf(screen3[0], "%s", "PM3 CC-Battery:");
        sprintf(screen3[1], "%s", displayString1);
        sprintf(screen3[2], "%s", "BATTERY SOC:");
        sprintf(screen3[3], "%s", displayString2);
        // = {"PM3 CC-Battery:", displayString1, "BATTERY SOC:", displayString2};
        int x_distances3[4] = {10, 1, 20, 40};
        print_text(screen3, x_distances3);

        break;

    case 4:
        // Print current time to buffer to be displayed on screen
        aon_timer_get_time_calendar(&PicoTime);
        sprintf(date_string, "%d-%d-%d", PicoTime.tm_year, PicoTime.tm_mon, PicoTime.tm_mday);
        sprintf(time_string, "%d:%d:%d", PicoTime.tm_hour, PicoTime.tm_min, PicoTime.tm_sec);

        if (button2_state)
        {
            select_num++;
            if (select_num > 5)
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
            char sd_card_selected[2][15] = {"SD CARD: OFF<", "SD CARD: ON<"};
            sprintf(current_sd_card, "%s", sd_card_selected[sd_card_toggle]);
            char screen4[4][15];
            sprintf(screen4[0], "%s", current_sd_card);
            sprintf(screen4[1], "%s", "DATE & TIME");
            sprintf(screen4[2], "%s", date_string);
            sprintf(screen4[3], "%s", time_string);
            // = {current_sd_card, "DATE & TIME", date_string, time_string};
            int x_distances4[4] = {1, 1, 1, 1};
            print_text(screen4, x_distances4);
            break;

        default:
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

            char date_time_selected[6][15] = {"DATE & TIME MO", "DATE & TIME D", "DATE & TIME Y", "DATE & TIME H", "DATE & TIME MI", "DATE & TIME S"};
            sprintf(current_time_select, "%s", date_time_selected[select_num - 1]);
            sprintf(current_sd_card, "%s", sd_card[sd_card_toggle]);
            char screen41[4][15];
            sprintf(screen41[0], "%s", current_sd_card);
            sprintf(screen41[1], "%s", current_time_select);
            sprintf(screen41[2], "%s", date_string);
            sprintf(screen41[3], "%s", time_string);
            // = {current_sd_card, current_time_select, date_string, time_string};
            int x_distances41[4] = {1, 1, 1, 1};
            print_text(screen41, x_distances41);

            break;
        }
        break;
    }
}