#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define BUTTON1PIN 6
#define BUTTON2PIN 7
#define BUTTON3PIN 8
#define BUTTON4PIN 9
#define BUTTONDELAYTIME 10 // delay for push button in ms
#define BUTTONFIFOSIZE 10 // can store up to 10 button presses

#define BUTTON_INTERRUPTS 1
//Comment this out to disable interrupts

extern volatile bool button1_state;
extern volatile bool button2_state;
extern volatile bool button3_state;
extern volatile bool button4_state;

void buttonsInit();
void buttonCallback(uint gpio, uint32_t events); // ISR for button interrupts

