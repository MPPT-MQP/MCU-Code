#include "buttons.h"
#include "oled_screen.h"


volatile bool button1_state = 0;
volatile bool button2_state = 0;
volatile bool button3_state = 0;
volatile bool button4_state = 0;

/* Start Debounce Code*/
#define DEBOUNCE_MS 50
bool is_debounceing = false;

int64_t debounce_alarm_callback(alarm_id_t id, void *user_data) {
    is_debounceing = false;
    return 0;
}

// Method to debounce switch input
bool debounce() {
    if (!is_debounceing) {
        add_alarm_in_ms(DEBOUNCE_MS, &debounce_alarm_callback, NULL, false);
        is_debounceing = true;
        return false;
    }
    return true;
}
/* End Debounce Code */

/// @brief Initialize buttons 1-4 using defined pins
void buttonsInit(void) {
    // initialize buttons
    gpio_init(BUTTON1PIN);
    gpio_init(BUTTON2PIN);
    gpio_init(BUTTON3PIN);
    gpio_init(BUTTON4PIN);
    // configure buttons as inputs
    gpio_set_dir(BUTTON1PIN, GPIO_IN);
    gpio_set_dir(BUTTON2PIN, GPIO_IN);
    gpio_set_dir(BUTTON3PIN, GPIO_IN);
    gpio_set_dir(BUTTON4PIN, GPIO_IN);
    // configure pull-up resistors
    gpio_pull_up(BUTTON1PIN);
    gpio_pull_up(BUTTON2PIN);
    gpio_pull_up(BUTTON3PIN);
    gpio_pull_up(BUTTON4PIN);

    #ifdef BUTTON_INTERRUPTS
    // enable interrupts with rising edge -> 0x08
    gpio_set_irq_enabled_with_callback(BUTTON1PIN, GPIO_IRQ_EDGE_FALL, true, &buttonISR);
    gpio_set_irq_enabled(BUTTON2PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON3PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON4PIN, GPIO_IRQ_EDGE_FALL, true);
    #endif

}

// Button Interrupt Service Routine
void buttonISR(uint gpio, uint32_t events) {
    if(debounce()) return; // Debounce button
    switch(gpio) {
        case BUTTON1PIN:
            printf("\nButton 1 pressed\n");
            button1_state = !button1_state;
        break;
            
        case BUTTON2PIN:
            printf("\nButton 2 pressed\n");
            button2_state = !button2_state;
        break;

        case BUTTON3PIN:
            printf("\nButton 3 pressed\n");
            button3_state = !button3_state;
            #ifdef OLED_SCREEN
            tracking_toggle = !tracking_toggle;
            #endif
        break;
        
        case BUTTON4PIN:
            printf("\nButton 4 pressed\n");
            button4_state = !button4_state;
        break;
    }
    
}