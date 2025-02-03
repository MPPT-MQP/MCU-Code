#include "user_interface.h"
#include "buttons.h"
#include "oled_screen.h"
#include <string.h>

/* Variables to keep track of which screen and setting is selected*/
int screen_num = 0;
int select_num = 0;

/* Initializing Text to Display on Screen*/
char *current_tracking = "START TRACKING";
char *tracking_status[] = {"START TRACKING", "STOP TRACKING"};
char *current_algorithm = "CV";
char *algorithms[] = {"CV", "BETA", "P&O", "P&O VAR", "INC-COND", "INC-COND VAR", "RCC", "PSO", "A-of-A"};
char *current_mode = "BUCK";
char *mode[] = {"MODE: BUCK", "MODE: CC", "MODE: BOTH"};
char *current_sd_card = "SD CARD: ON";
char *sd_card[] = {"SD CARD: ON", "SD CARD: OFF"};

/*Toggle Variables to Keep Track of Settings*/
// 0 = Tracking Off, 1 = Tracking On 
int tracking_toggle = 0;
// 0 = CV, 1 = Beta, 2= P&O, etc.
int algorithm_toggle = 0;
// 0 = Buck Only, 1 = Charge Controller Only, 2 = Both
int mode_toggle = 0;
// 0 = OFF, 1 = ON
int sd_card_toggle = 0;


void welcome_screen(){
    char *screen1[] = {" ", "WPI MPPT MQP", "2024-2025", " "};
    int x_distances1[] = {1, 15, 30, 1};
    print_text(screen1, count_of(screen1), x_distances1);
}

void run_main_screens() {

    if(button1_state) {
        screen_num++;
        if(screen_num > 4) {
            screen_num = 0;
        }
        button1_state = !button1_state;
    }
    clear_display();

    switch(screen_num) {
        case 0:
            if(button2_state) {                
                select_num++;
                if(select_num > 2) {
                   select_num = 0;
                }
                button2_state = !button2_state;
            }
            switch(select_num) {
                case 0:
                    if(button3_state) {
                            tracking_toggle++;
                        if(tracking_toggle > 1) {
                              tracking_toggle = 0;
                        }
                         button3_state = !button3_state;
                    }
                    char *tracking_status_selected[] = {"START TRACKING<", "STOP TRACKING<"};
                    current_tracking = tracking_status_selected[tracking_toggle];
                    current_algorithm = algorithms[algorithm_toggle];
                    current_mode = mode[mode_toggle];
                    char *screen0[] = {current_tracking, "SET ALGORITHM", current_algorithm, current_mode};
                    int x_distances0[] = {1, 1, 1, 1};
                    print_text(screen0, count_of(screen0), x_distances0);
                break;
                    
                case 1:
                    if(button3_state) {
                            algorithm_toggle++;
                        if(algorithm_toggle > 8) {
                             algorithm_toggle = 0;
                        }
                         button3_state = !button3_state;
                    }    
                    current_tracking = tracking_status[tracking_toggle];
                    current_algorithm = algorithms[algorithm_toggle];
                    current_mode = mode[mode_toggle];
                    char *screen01[] = {current_tracking, "SET ALGORITHM<", current_algorithm, current_mode};
                    int x_distances01[] = {1, 1, 1, 1};
                    print_text(screen01, count_of(screen01), x_distances01);

                break;

                case 2:
                    if(button3_state) {
                            mode_toggle++;
                        if(mode_toggle > 2) {
                             mode_toggle = 0;
                        }
                         button3_state = !button3_state;
                    }
                    char *mode_selected[] = {"MODE: BUCK<", "MODE: CC<", "MODE: BOTH<"};
                    current_tracking = tracking_status[tracking_toggle];
                    current_algorithm = algorithms[algorithm_toggle];
                    current_mode = mode_selected[mode_toggle];
                    char *screen02[] = {current_tracking, "SET ALGORITHM", current_algorithm, current_mode};
                    int x_distances02[] = {1, 1, 1, 1};
                    print_text(screen02, count_of(screen02), x_distances02);
                break;

            }
            
        break;

        case 1:
            char *screen1[] = {"TEMPERATURE:", "00.00*C", "IRRADIANCE:", "00.00 W/m^2"};
            int x_distances1[] = {20, 40, 20, 20};
            print_text(screen1, count_of(screen1), x_distances1);
        break;

        case 2:
            char *screen2[] = {"PM1 PV-Buck:", "00.00 V 00.00 A", "PM2 Buck-CC:", "00.00 V 00.00 A"};
            int x_distances2[] = {20, 1, 20, 1};
            print_text(screen2, count_of(screen2), x_distances2);
        break;

        case 3:
            char *screen3[] = {"PM3 CC-Battery:", "00.00 V 00.00 A", "BATTERY SOC:", "00.00%"};
            int x_distances3[] = {10, 1, 20, 40};
            print_text(screen3, count_of(screen3), x_distances3);
            
        break;

        case 4:
            if(button2_state) {                
                select_num++;
                if(select_num > 2) {
                   select_num = 0;
                }
                button2_state = !button2_state;
            }
            switch(select_num){
                case 0:








            }
            char *screen4[] = {"SD CARD: ON", "Date & Time", "2025-02-02", "00:00:00"};
            int x_distances4[] = {1, 1, 1, 1};
            print_text(screen4, count_of(screen4), x_distances4);
        break;  

    }
}