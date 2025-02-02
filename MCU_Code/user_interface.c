#include "user_interface.h"
#include "buttons.h"
#include "oled_screen.h"

int screen_num = 0;
int select_num = 0;

void welcome_screen(){
    char *screen1[] = {" ", "WPI MPPT MQP", "2024 2025", " "};
    int x_distances1[] = {1, 15, 30, 1};
    print_text(screen1, count_of(screen1), x_distances1);
}

void run_main_screens() {

    if(button1_state) {
        screen_num++;
        if(screen_num > 3) {
            screen_num = 0;
        }
        button1_state = !button1_state;
    }
    clear_display();

    switch(screen_num) {
        case 0:
            char *screen1[] = {"TEMPERATURE:", "00.00", "IRRADIANCE", "00.00"};
            int x_distances1[] = {20, 40, 20, 40};
            print_text(screen1, count_of(screen1), x_distances1);
        break;

        case 1:
            char *screen2[] = {"POWER MONITOR 1", "00.00 V 00.00 A", "POWER MONITOR 2", "00.00 V, 00.00 A"};
            int x_distances2[] = {1, 1, 1, 1};
            print_text(screen2, count_of(screen2), x_distances2);
        break;

        case 2:
            char *screen3[] = {"POWER MONITOR 3", "00.00 V 00.00 A", "BATTERY SOC", "00.00"};
            int x_distances3[] = {1, 1, 20, 40};
            print_text(screen3, count_of(screen3), x_distances3);
        break;

        case 3:
            if(button2_state) {
                select_num++;
                if(select_num > 2) {
                   select_num = 0;
                }
                button2_state = !button2_state;
            }
            switch(select_num) {
                case 0:
                    char *screen4[] = {"START TRACKING)", "SET ALGORITHM", "Test", "SAVE SD CARD"};
                    int x_distances4[] = {1, 1, 30, 1};
                    print_text(screen4, count_of(screen4), x_distances4);
                break;
                    
                case 1:
                    char *screen41[] = {"START TRACKING", "SET ALGORITHM)", "Test", "SAVE SD CARD"};
                    int x_distances41[] = {1, 1, 30, 1};
                    print_text(screen41, count_of(screen41), x_distances41);
                break;

                case 2:
                    char *screen42[] = {"START TRACKING", "SET ALGORITHM", "Test", "SAVE SD CARD)"};
                    int x_distances42[] = {1, 1, 30, 1};
                    print_text(screen42, count_of(screen42), x_distances42);
                break;

            }
        break;
    }
}