/**
 * @file lab03.c
 * @author Zerind Spore
 * @author "YOUR NAME HERE"
 * @brief Sampling and Signal Conversion Lab
 * @version 0.1
 * @date 2025-10-29
 */

#include "Active_Lab.h"
#include "main.h"


#if defined(lab03)

char APP_DESCRIPTION[] = "BME 463: Lab 03 - Signal Conversion";
// ------------------------------------------------------------------------------------- //
#define SAMPLE_PERIOD_US   2000// Change Sampling period here throughout lab
// Start with 1000 uS, then compute period for 1kHz, 900Hz, 1.1khz, and 5kHz as prompted to
// visualize the importance of aliasing and the Nyquist rate
// ------------------------------------------------------------------------------------- //

// ----- PIN ASSIGNMENT DEFINITIONS -----

// ADC Pin and Channel
#define ADC_GPIO      26
#define ADC_CH        0

// Pin for PICO on board LED
#define LED_PIN PICO_DEFAULT_LED_PIN 

// Pin for interrupt flag
#define FLAG_PIN  22

static repeating_timer_t samp_timer;
static volatile bool sample_flag = false;

#define M_PI 3.14159265358979323846


static bool samp_timer_cb(repeating_timer_t *t)
{
    // Raise the sample flag so main loop knows to grab a sample
    sample_flag = true;

    // Pulse FLAG_PIN high here so you can see it on the oscilliscope if you want
    gpio_put(FLAG_PIN, 1);

    // return true needed to keep the timer repeating
    return true;
}


// Initialize additonal HW pins you wish to use here if needed
void app_init_hw(void){

    // Initialize LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // Initialize interrupt flag pin
    gpio_init(FLAG_PIN);
    gpio_set_dir(FLAG_PIN, GPIO_OUT);
    gpio_put(FLAG_PIN, 0);

    // Initialize DAC
    MCP4728_init(i2c1, 14, 15, 400 * 1000, 0x60);

    // Initialize ADC
    adc_init();
    adc_gpio_init(ADC_GPIO);
    adc_select_input(ADC_CH);

    SevSeg_init(spi1, 1, 10, 11, 12, 100000);
    SevSeg_setBrightness(80);
    Disp_Str("L 03");

    serial_start_boot_banner(APP_DESCRIPTION, NAME);

     add_repeating_timer_us(
        -SAMPLE_PERIOD_US,
        samp_timer_cb,
        NULL,
        &samp_timer
    );
}

// Sampling loop implemented for you, read comments to understand this code as you will
// use sampling extensively in this course and any future mixed-signal MCU work you do
void app_main(void){
    
    while (true)
    {
        if (sample_flag)
        {
            // Read one ADC sample
            // adc_read() returns 12-bit value between 0 and 4095
            uint16_t val_raw = adc_read();

            // Convert to float 0.0 - 1.0
            float val_float = (float)val_raw / 4095.0f;

            // Write same value out to the DAC
            MCP4728_write1(val_float);

            // Lower the FLAG pin
            gpio_put(FLAG_PIN, 0);

            // Clear the flag until next ISR tick
            sample_flag = false;
        }
        tight_loop_contents();
    }
}

#endif