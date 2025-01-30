#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define BUTTONPIN_1 6
#define BUTTONPIN_2 7
#define BUTTONPIN_3 8
#define BUTTONPIN_4 9

#define BUTTON_INTERRUPTS 1
//Comment this out to disable interrupts


void buttonsInit();
void buttonISR();