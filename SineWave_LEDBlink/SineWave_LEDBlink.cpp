#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdint.h>
#include <math.h>
   #define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS  5
#define PIN_SCK  2
#define PIN_MOSI 3
#define pi (3.14159265)

#define LED_PIN 25
#define LED_RATE_MS    1000 // LED Blinking Rate

int k=0;
static repeating_timer_t led_timer;
static repeating_timer_t sine_timer;

// Create a repeating timer for the LED ISR.
static bool led_timer_cb(repeating_timer_t *t) {
    gpio_put(LED_PIN, !gpio_get(LED_PIN)); // Change LED state
    return true;
}
static bool sine_timer_cb(repeating_timer_t *t) {
    
            float sinwave_float=(4095.0*sin(k*pi/180.0))/2.0+2047.5;
            int sinwave=roundf(sinwave_float);
            uint16_t config_bits= 0x3000;
            uint16_t c=(uint16_t)(config_bits | (sinwave & 0x0FFF));
            uint8_t data[2];
            data[0]=(c>>8)&0xFF; //MSB
            data[1]=c & 0xFF;     //LSB    
            gpio_put(PIN_CS, 0); //Enable device
            spi_write_blocking(SPI_PORT, data, 2);
            gpio_put(PIN_CS, 1); //Disable device
           
        
        k++;
        if(k>=360){
            k=0;
        }
    
    return true;
}


int main() {
    
 // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
 
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);




    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    add_repeating_timer_ms(LED_RATE_MS, led_timer_cb, NULL, &led_timer);
     add_repeating_timer_us(2778, sine_timer_cb, NULL, &sine_timer);
    while (1) {
       tight_loop_contents();
    }
}
