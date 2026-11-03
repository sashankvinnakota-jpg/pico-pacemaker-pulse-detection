/**
 * @file lab06.c
 * @author Gavin Harwood
 * @author Zerind Spore
 * @author "YOUR NAME HERE"
 * @brief 
 * @version 0.2
 * @date 2025-12-17
 */

#include "Active_Lab.h"
#include "main.h"


#if defined(lab06)



char APP_DESCRIPTION[] = "BME 463: Lab 06 - FIR Derivative Filters";

// ----- PIN ASSIGNMENT DEFINITIONS -----

// Pin for PICO on board LED
#define LED_PIN PICO_DEFAULT_LED_PIN 

// Pin for interrupt flag
#define FLAG_PIN  22   

// ADC Pin and Channel
#define ADC_GPIO      26
#define ADC_CH        0

// ---- Timing and Timers ----
#define SAMPLE_PERIOD_US   5000 // Sampling Frequency -> 200 Hz
static repeating_timer_t samp_timer;
static volatile bool sample_flag = false;

static bool samp_timer_cb(repeating_timer_t *t)
{
    sample_flag = true;
    gpio_put(FLAG_PIN, 1);
    return true;
}

// ---------------------------------------------------------------------------------
// Implement values for filter coefficients ----------------------------------------
// ---------------------------------------------------------------------------------

// Selected Derivative Filter 
#define Derivative_N  5
static float c_derivative[Derivative_N] = {0.5,0.25,0,-0.25,-0.5};   // Coefficients
static float a_derivative = 1.0;                // Overall gain           
static float x_derivative[Derivative_N] = {0};   // Input buffer

// 60 Hz Notch
#define NOTCH_N  3
static float c_notch[NOTCH_N] = {1,-1,1};            // Coefficients
static float a_notch = 1.0f;                    // Overall Gain
static float x_notch[NOTCH_N] = {0};            // Input Buffer

// ---------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------

// Enumerated Type to select what part of the lab you are working on
typedef enum{
    PART1 = 0, // Experiment Setup
    PART2 = 1, // Derivative
    PART3 = 2, // Derivative + Notch
} current_t; 

// Initialize additonal HW pins you wish to use here if needed
void app_init_hw(void){
    // Print to serial your name and which lab is running
    printf("%s\n", APP_DESCRIPTION); 
    printf("Name: %s\n", NAME);

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
    
    // ADC init
    adc_init();
    adc_gpio_init(ADC_GPIO); 
    adc_select_input(ADC_CH);

    SevSeg_init(spi1, 1, 10, 11, 12, 100000);
    SevSeg_setBrightness(80);
    Disp_Str("L 06");

    serial_start_boot_banner(APP_DESCRIPTION, NAME);

    // Start periodic sampling timer
    add_repeating_timer_us(
        -SAMPLE_PERIOD_US,
        samp_timer_cb,
        NULL,
        &samp_timer
    );
}

// Implement the behavior described for this lab within main
void app_main(void){

    current_t running = PART3; // CHANGE TO PART YOU ARE ON IN LAB

    while (true){

        if (!sample_flag) { // Wait until new sample
            tight_loop_contents();
            continue;
        }
        sample_flag = false; // Clear flag if high

        // Read ADC and place into input buffer
        uint16_t raw_adc_value = adc_read();
        float new_input_sample = (float)raw_adc_value / 4095.0f; // Center around 0V

        float new_output_sample = 0.0f; // Computed in switch statement

        switch(running)
        {
            case PART1: // Experiment Setup
                // Test input/output by connecting ADC to DAC directly
                // Allows verification of analog filter circuit
                MCP4728_write1(new_input_sample);
                break;
                

            case PART2: // Derivative
                // Implement Derivative Filter here
            shift_right(x_derivative, Derivative_N);
                x_derivative[0] = new_input_sample;

                float y1;
                y1 = filter_FIR(
                    a_derivative,
                    x_derivative,
                    c_derivative,
                    Derivative_N
                );
               
                
                

                // Send result to DAC
                MCP4728_write1(y1+0.5f);
                break;


            case PART3: // Derivative + Notch
                // Implement Derivative + Notch Filter here
                
                shift_right(x_derivative, Derivative_N);
                x_derivative[0] = new_input_sample;

                float y1;
                y1 = filter_FIR(
                    a_derivative,
                    x_derivative,
                    c_derivative,
                    Derivative_N
                );
                MCP4728_write1(y1+0.5f);
                shift_right(x_notch, NOTCH_N);
           x_notch[0] = y1;

              new_output_sample = filter_FIR(
        a_notch,
        x_notch,
        c_notch,
        NOTCH_N
    ) ;

                // Send Derivative filter result to DAC
               

                // Send Derivative + Notch filter result to DAC
                MCP4728_write1(new_output_sample);
                break;

            default :
                new_output_sample = new_input_sample;
                MCP4728_write1(new_output_sample);
                break;
        }

        // Drop HW flag (this allows visibility into the length of time this routine takes to run)
        gpio_put(FLAG_PIN, 0);

    }
}

#endif// Paste Lab 6 Code Here When Provided.
// Derivative Filters (FIR)