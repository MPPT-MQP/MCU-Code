#include "buttons.h"
#include "oled_screen.h"

int screen_num = 0;

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
    // enable interrupts with falling edge -> 0x04
    gpio_set_irq_enabled_with_callback(BUTTON1PIN, GPIO_IRQ_EDGE_RISE, true, &buttonCallback);
    gpio_set_irq_enabled_with_callback(BUTTON2PIN, GPIO_IRQ_EDGE_RISE, true, &buttonCallback);
    gpio_set_irq_enabled_with_callback(BUTTON3PIN, GPIO_IRQ_EDGE_RISE, true, &buttonCallback);
    gpio_set_irq_enabled_with_callback(BUTTON4PIN, GPIO_IRQ_EDGE_RISE, true, &buttonCallback);
}

void buttonCallback(uint gpio, uint32_t events) {
    if(debounce()) return;
    screen_num++;
    // switch(gpio) {
    //     case 6:
            
    //     break;
        
    //     case 7:
    //     break;

    //     case 8:
    //     break;
        
    //     case 9:
    //     break;
    // }
    
}