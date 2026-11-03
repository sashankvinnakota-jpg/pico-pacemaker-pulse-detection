/**
 * @file main.c
 * @author Zerind Spore (zspore@wisc.edu)
 * @brief 
 * @version 0.2
 * @date 2025-10-8
 */

#include "main.h"

/*******************************************************************************
********************************************************************************


               -----------------
              |GPO    USB   VBUS|
    Disp CS<->|GP1          VBUS|
              |GND           GND|
    Err LED<->|GP2        3V3_EN|
   User LED<->|GP3       3V3_OUT|<-->VDD_MCP4728, VREFA_MCP4728
    ESP CS <->|GP4      ADC_VREF|
              |GP5          ADC2|
              |GND           GND|
              |GP6          ADC1|
              |GP7          ADC0|
              |GP8           RUN|
              |GP9          GP22|
              |GND           GND|
    SPI SCK<->|GP10         GP21|
   SPI MOSI<->|GP11         GP21|
   SPI MISO<->|GP12         GP19|
              |GP13         GP18|
              |GND           GND|
       SDA<-->|GP14         GP17|<-> UART0_RX
       SCL<-->|GP15         GP16|<-> UART0_TX
               -----------------
          Raspberry PI PICO 2 GPIO Header
*******************************************************************************/


// SPI Defines
// We are going to use SPI 1, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SEVSEG_SPI   spi1
#define SEVSEG_CS    1
#define SEVSEG_SCK   10
#define SEVSEG_MOSI  11
#define SEVSEG_MISO  12

#define PIN_CS_SEVSEG 1 // Chip Select for Seven Segment Display
#define PIN_CS_ESP 4 // Chip Select for MCP4728 DAC  

// I2C defines
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 16
#define UART_RX_PIN 17

char NAME[] = "BME463 Student"; // Replace with your name

__attribute__((weak)) void app_init_hw(void) { }
__attribute__((weak)) void app_main(void)   { for (;;) tight_loop_contents(); }


int main()
{
    //stdio_init_all();
    //sleep_ms(250);

    //for (;;)
    //{
        stdio_init_all();
        sleep_ms(500);
        cdc_println("Starting application");
        app_init_hw();
        app_main();
        while (1) tight_loop_contents();
    //}

}
