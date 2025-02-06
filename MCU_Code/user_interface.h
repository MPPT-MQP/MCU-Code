#include <stdio.h>
#include "pico/stdlib.h"


// Externed variables to keep track of settings in main
extern int tracking_toggle;
extern int algorithm_toggle;
extern int mode_toggle;
extern int sd_card_toggle;

/*Function Prototypes*/
void run_main_screens();
void welcome_screen();

// //Extern Main Sensor Buffer
// extern struct sensorData sensorBuffer[800];
// extern uint16_t BufferCounter;