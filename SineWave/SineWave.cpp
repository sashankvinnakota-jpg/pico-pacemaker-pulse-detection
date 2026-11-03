#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include<stdint.h>
#include<math.h>
#define pi (3.14159265)

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS  5
#define PIN_SCK  2
#define PIN_MOSI 3


int k=0;
int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    while (true) {
        for(k=0;k<360;k++){
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
            sleep_us(2777.8);
        
        }
    }
}
