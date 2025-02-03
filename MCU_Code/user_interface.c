#include "user_interface.h"
#include "buttons.h"
#include "oled_screen.h"
#include <string.h>

int screen_num = 0;
int select_num = 0;
int toggle_num = 0;
char *current_mode = "BUCK";
char *current_algorithm = "CV";
char *current_tracking = "START TRACKING";
char *tracking_status[] = {"START TRACKING", "STOP TRACKING"};
char *mode[] = {"MODE: BUCK", "MODE: CC", "MODE: BOTH"};
char *algorithms[] = {"CV", "Beta", "PO", "PO Var", "IC", "IC Var", "RCC", "PSO", "A-of-A"};
int current_mode_toggle = 0;
int current_tracking_toggle = 0;
int current_algorithm_toggle = 0;

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
                            current_tracking_toggle++;
                        if(current_tracking_toggle > 1) {
                              current_tracking_toggle = 0;
                        }
                         button3_state = !button3_state;
                    }
                    char *tracking_status_selected[] = {"START TRACKING<", "STOP TRACKING<"};
                    current_tracking = tracking_status_selected[current_tracking_toggle];
                    current_algorithm = algorithms[current_algorithm_toggle];
                    current_mode = mode[current_mode_toggle];
                    char *screen0[] = {current_tracking, "SET ALGORITHM", current_algorithm, current_mode};
                    int x_distances0[] = {1, 1, 30, 1};
                    print_text(screen0, count_of(screen0), x_distances0);
                break;
                    
                case 1:
                    if(button3_state) {
                            current_algorithm_toggle++;
                        if(current_algorithm_toggle > 8) {
                             current_algorithm_toggle = 0;
                        }
                         button3_state = !button3_state;
                    }    
                    current_algorithm = algorithms[current_algorithm_toggle];
                    current_tracking = tracking_status[current_tracking_toggle];
                    current_mode = mode[current_mode_toggle];
                    char *screen01[] = {current_tracking, "SET ALGORITHM<", current_algorithm, current_mode};
                    int x_distances01[] = {1, 1, 30, 1};
                    print_text(screen01, count_of(screen01), x_distances01);

                break;

                case 2:
                    if(button3_state) {
                            current_mode_toggle++;
                        if(current_mode_toggle > 2) {
                             current_mode_toggle = 0;
                        }
                         button3_state = !button3_state;
                    }
                    char *mode_selected[] = {"MODE: BUCK<", "MODE: CC<", "MODE: BOTH<"};
                    current_tracking = tracking_status[current_tracking_toggle];
                    current_algorithm = algorithms[current_algorithm_toggle];
                    current_mode = mode_selected[current_mode_toggle];
                    char *screen02[] = {current_tracking, "SET ALGORITHM", current_algorithm, current_mode};
                    int x_distances02[] = {1, 1, 30, 1};
                    print_text(screen02, count_of(screen02), x_distances02);
                break;

            }
            
        break;

        case 1:
            char *screen1[] = {"TEMPERATURE:", "00.00", "IRRADIANCE:", "00.00"};
            int x_distances1[] = {20, 40, 20, 40};
            print_text(screen1, count_of(screen1), x_distances1);
        break;

        case 2:
            char *screen2[] = {"PM1 PV-Buck:", "00.00 V 00.00 A", "PM2 Buck-CC:", "00.00 V 00.00 A"};
            int x_distances2[] = {20, 1, 20, 1};
            print_text(screen2, count_of(screen2), x_distances2);
        break;

        case 3:
            char *screen3[] = {"PM3 CC-Battery:", "00.00 V 00.00 A", "BATTERY SOC:", "00.00"};
            int x_distances3[] = {10, 1, 20, 40};
            print_text(screen3, count_of(screen3), x_distances3);
            
        break;

        case 4:
            char *screen4[] = {"Save SD Card", "Date and Time", "2025-02-02", "00:00"};
            int x_distances4[] = {15, 10, 20, 40};
            print_text(screen4, count_of(screen4), x_distances4);
        break;  

    }
}