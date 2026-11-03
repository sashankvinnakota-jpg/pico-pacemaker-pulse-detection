#include <stdio.h>
#include "pico/stdlib.h"


int main()
{ 
    stdio_init_all();
    const uint8_t Onboard_LED_PIN = 25;
    const uint8_t Button_PIN = 22;
    gpio_init(Onboard_LED_PIN);
    gpio_init(Button_PIN);
    gpio_set_dir(Onboard_LED_PIN, GPIO_OUT);
    gpio_set_dir(Button_PIN, GPIO_IN);
    

    while (true) {
        if(gpio_get(Button_PIN)==1){
        gpio_put(Onboard_LED_PIN, 1);
        }
        else{
        gpio_put(Onboard_LED_PIN, 0);
        }
    }
}
