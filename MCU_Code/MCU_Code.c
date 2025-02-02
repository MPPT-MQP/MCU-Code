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

int screen_num = 0;

int main()
{
    stdio_init_all();

    // // Timer example code - This example fires off the callback after 2000ms
    // add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks
    

    /*Start of non example code*/
    //Init both I2C0 and I2C1
    configI2C();

    //Temp Sensor ADC Setup
    TMP_ADC_setup();

    //Init SD Card Setup (hw_config.c sets the SPI pins)
    sd_init_driver();

    //Setup Buttons
    buttonsInit();

    // Initialize OLED Screen
    oled_init();

    while (true) {
        //PM_printManID(0x40);
        //printf("\n\nTEST");
        //printf("\nVoltage: %f", PM_readVoltage(0x40));
        //printf("\nCurrent: %f", PM_readCurrent(0x40));
        //printf("%d", button1_state);
        if(button1_state) {
            screen_num++;
            if(screen_num > 2) {
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
        }
    }
}
