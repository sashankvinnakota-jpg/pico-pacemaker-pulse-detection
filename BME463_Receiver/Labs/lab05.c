/**
 * @file lab05.c
 * @author Zerind Spore
 * @author "YOUR NAME HERE"
 * @brief 
 * @version 0.1
 * @date 2025-10-29
 */

#include "Active_Lab.h"
#include "main.h"
#include "pico/stdio.h"


#if defined(lab05)



char APP_DESCRIPTION[] = "BME 463: Lab 05 - FIR Filters";

// ----- PIN ASSIGNMENT DEFINITIONS -----

// Pin for PICO on board LED
#define LED_PIN PICO_DEFAULT_LED_PIN 

// Pin for interrupt flag
#define FLAG_PIN  22   

// ADC Pin and Channel
#define ADC_GPIO      26
#define ADC_CH        0

// ---- Timing and Timers ----
#define SAMPLE_PERIOD_US   5556 // Sampling Frequency -> 200 Hz
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

// Hanning Filter -> 
#define HANNING_N  3
static float c_hanning[HANNING_N] = {1,2,1};   // Coefficients
static float a_hanning = 1.0f/4.0f;              // Overall gain           
static float x_hanning[HANNING_N] = {0};     // Input buffer

// Cascaded Hanning (run hanning multiple times)
static float x2_hanning[HANNING_N] = {0};    // Input buffer
static float x3_hanning[HANNING_N] = {0};    // Input buffer

// Poly4
#define POLY4_N  9
static float c_poly4[POLY4_N] = {-21,14,39,54,59,54,39,14,-21};
static float a_poly4 = 1.0f/231.0f; 
static float x_poly4[POLY4_N] = {0}; 

// Notch
#define NOTCH_N  3
static float c_notch[NOTCH_N] = {1,-1,1};
static float a_notch = 1.0f/3.0f;
static float x_notch[NOTCH_N] = {0}; 

// ---------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------

// Enumerated Type to select what part of the lab you are working on
typedef enum{
    PART1 = 0, // Hanning
    PART2 = 1, // Hanning Cascade
    PART3 = 2, // Poly4
    PART4 = 3, // Notch
} current_t; 

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
    
    // ADC init
    adc_init();
    adc_gpio_init(ADC_GPIO); 
    adc_select_input(ADC_CH);

    SevSeg_init(spi1, 1, 10, 11, 12, 100000);
    SevSeg_setBrightness(80);
    Disp_Str("L 05");

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
// MUST COMPLETE filter_FIR in Filters.c BEFORE doing anything further
void app_main(void){
    
    
    current_t running = PART4; // CHANGE TO PART YOU ARE ON IN LAB

    while (true){

        if (!sample_flag) { // Wait until new sample
            tight_loop_contents();
            continue;
        }
        sample_flag = false; // Clear flag if high

        // Read ADC and place into input buffer
        uint16_t raw_adc_value = adc_read();
        float new_input_sample = (float)raw_adc_value / 4095.0f;

        float new_output_sample = 0.0f; // Computed in switch statement

        switch(running)
        {
            case PART1: // Hanning
                shift_right(x_hanning, HANNING_N);
                x_hanning[0] = new_input_sample;

                new_output_sample = filter_FIR(
                    a_hanning,
                    x_hanning,
                    c_hanning,
                    HANNING_N
                );

                // Send result to DAC
                MCP4728_write1(new_output_sample);
                // Record amplitude vs frequency
                break;
                

            case PART2: // Cascaded Hanning
                shift_right(x_hanning, HANNING_N);
                x_hanning[0] = new_input_sample;

                float y1;
                y1 = filter_FIR(
                    a_hanning,
                    x_hanning,
                    c_hanning,
                    HANNING_N
                );
                MCP4728_write1(y1);
                shift_right(x2_hanning, HANNING_N);
                x2_hanning[0] = y1;

                new_output_sample = filter_FIR(
                    a_hanning,
                    x2_hanning,
                    c_hanning,
                    HANNING_N
                );
                // Send result to DAC
                MCP4728_write2(new_output_sample);
                // Record amplitude vs frequency
                break;


            case PART3: // Poly4
            {
                shift_right(x_poly4, POLY4_N);
                x_poly4[0] = new_input_sample;

                new_output_sample = filter_FIR(
                    a_poly4,
                    x_poly4,
                    c_poly4,
                    POLY4_N
                );

                // Send result to DAC
                MCP4728_write1(new_output_sample);
            }

                break;

            case PART4:
            {
            shift_right(x_notch, NOTCH_N);
    x_notch[0] = new_input_sample;

    new_output_sample = filter_FIR(
        a_notch,
        x_notch,
        c_notch,
        NOTCH_N
    ) ;

    // Send result to DAC
    MCP4728_write1(new_output_sample);
} // Notch
            
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

#endif