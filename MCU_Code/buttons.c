#include "buttons.h"


void buttonsInit(){
    
    //These might work, not sure
    gpio_init_mask(0xF);
    gpio_set_dir_in_masked(0xF);
    //0xF sets gpio numbers 6,7,8,9 mask to 1 i think?
    
    // gpio_init(BUTTONPIN_1);
    // gpio_set_dir(BUTTONPIN_1, GPIO_IN);
    gpio_pull_up(BUTTONPIN_1);

    // gpio_init(BUTTONPIN_2);
    // gpio_set_dir(BUTTONPIN_2, GPIO_IN);
    gpio_pull_up(BUTTONPIN_2);

    // gpio_init(BUTTONPIN_3);
    // gpio_set_dir(BUTTONPIN_3, GPIO_IN);
    gpio_pull_up(BUTTONPIN_3);

    // gpio_init(BUTTONPIN_4);
    // gpio_set_dir(BUTTONPIN_4, GPIO_IN);
    gpio_pull_up(BUTTONPIN_4);

    #ifdef BUTTON_INTERRUPTS
    //Set up button interrupts (all share the same ISR fcn: buttonISR)
    gpio_set_irq_enabled_with_callback(BUTTONPIN_1, GPIO_IRQ_EDGE_FALL, BUTTON_INTERRUPTS, buttonISR);
    gpio_set_irq_enabled(BUTTONPIN_2, GPIO_IRQ_EDGE_FALL, BUTTON_INTERRUPTS);
    gpio_set_irq_enabled(BUTTONPIN_3, GPIO_IRQ_EDGE_FALL, BUTTON_INTERRUPTS);
    gpio_set_irq_enabled(BUTTONPIN_4, GPIO_IRQ_EDGE_FALL, BUTTON_INTERRUPTS);
    #endif
}



//Button Interrupt
void buttonISR(uint gpio, uint32_t event_mask){
    //Flag is cleared by the PicoSDK automatically

    //Find which button was pressed
    switch (gpio)
    {
    case BUTTONPIN_1:
        printf("\nButton 1 pressed\n");
        break;
    
    case BUTTONPIN_2:
        printf("\nButton 2 pressed\n");
        break;
    
    case BUTTONPIN_3:
        printf("\nButton 3 pressed\n");
        break;

    case BUTTONPIN_4:
        printf("\nButton 4 pressed\n");
        break;
    }

}


