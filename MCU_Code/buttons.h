#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define BUTTON1PIN 6
#define BUTTON2PIN 7
#define BUTTON3PIN 8
#define BUTTON4PIN 9

#define BUTTON_INTERRUPTS 4
//Comment this out to disable interrupts

// extern int tracking_toggle;

// Make button state variables accessible in main 
extern volatile bool button1_state;
extern volatile bool button2_state;
extern volatile bool button3_state;
extern volatile bool button4_state;

//Function Prototypes
void buttonsInit();
void buttonISR(uint gpio, uint32_t events); // ISR for button interrupts

