/**
 * @file lab04.c
 * @author Zerind Spore
 * @author "YOUR NAME HERE"
 * @brief 
 * @version 0.2
 * @date 2025-10-29
 */

#include "Active_Lab.h"
#include "Drivers/DAC_I2C_Driver.h"
#include "main.h"


#if defined(lab04)



char APP_DESCRIPTION[] = "BME 463: Lab 04 - IIR Filters";

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
// Implement values for filter coefficients from prelab ----------------------------
// ---------------------------------------------------------------------------------
static float x[3] = {0.0f, 0.0f, 0.0f};
static float y[3] = {0.0f, 0.0f, 0.0f};

// Low Pass Filter (LPF) -> Fill in values from prelab
static float cx_LPF[3] = {1.0,2.0,1.0};
static float cy_LPF[3] = {1,0.68,-0.25};
static float gain_LPF = 0.2; 

// Band Reject Filter (BRF, 60Hz) -> Fill in values from prelab
static float cx_BRF[3] = {1.0,0.61830,1};
static float cy_BRF[3] = {1,0.55623,0.81};
static float gain_BRF  = 0.903;   // start at 1.0 for frequency response sanity


// High Pass Filter (HPF) -> Fill in values from prelab
static float cx_HPF[3] = {1.0,-2.0,1.0};
static float cy_HPF[3] = {1.0,-0.49443,0.64};
static float gain_HPF  = 0.2894;

// Band Pass Filter (BPF) -> Fill in values from prelab
static float cx_BPF[3] = {1.0,0.0,-1.0};
static float cy_BPF[3] = {1.0,-1.601,0.8649};
static float gain_BPF  = 0.06753;

// ---------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------

// Enumerated Type to select what part of the lab you are working on
typedef enum{
    PART1 = 0,
    PART2 = 1,
    PART3 = 2,
    PART4 = 3,
    PART5 = 4,
    PART6 = 5,
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
    Disp_Str("L 04");

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
    current_t running = PART6; // CHANGE TO PART YOU ARE ON IN LAB
    
    while (true){

        if (!sample_flag) { // Wait until new sample
            tight_loop_contents();
            continue;
        }
        sample_flag = false; // Clear flag if high

        // shift inputs and outputs
        // *note this will only work if you have completed the Filters.c implementation of the function
        shift_right(x, 3);
        shift_right(y, 3);

        // Read ADC and place into input buffer
        uint16_t raw_adc_value = adc_read();
        float new_input_sample = (float)raw_adc_value / 4095.0f; // Center around 0V
        x[0] = new_input_sample;

        float new_output_sample = 0.0f; // Computed in switch statement

        switch(running)
        {
            case PART1: // Fill out Filters.c
                // For PART 1, get Filters.c checked off by TA
                // Get checked off before continuing
                new_output_sample = new_input_sample;
                break;


            case PART2: // Confirm input/output pin operation
                // For PART 2 connect sine wave from the sender Pico to the ADC pin 
                // (this is done on the PCB, you don't have to physically do it) 
                // and monitor the output from the RC filter connected to DAC
                new_output_sample = new_input_sample;
                break;


            case PART3: // Implement Low Pass Filter
                 new_output_sample = filter_IIR(
                    gain_LPF,
                    x, cx_LPF, 3,
                    y, cy_LPF, 3
                );
                // adjust the LPF gain so that the DAC output won’t overflow
                // Vary the frequency of the sine wave. Find the –3 dB frequency
                // What is the BW of this filter? 
                // What is the 60-Hz gain in dB, and how does it compare with the DigiScope filter? 
                // Connect the ECG simulator with 10% 60-Hz noise. Are the output results what you expect? 
                break;


            case PART4: // Implement Band Reject Filter (60Hz Notch)
                 new_output_sample = filter_IIR(
                    gain_BRF,
                    x, cx_BRF, 3,
                    y, cy_BRF, 3
                );
                // adjust the BRF gain so that the DAC output won’t overflow
                // Connect the sine wave. 
                // Does the filter completely reject 60 Hz noise as expected as compared with the DigiScope filter?
                // Connect the ECG simulator with 30% 60-Hz noise. Are the output results what you expected?
                break;


            case PART5: // Implement High Pass Filter
                 new_output_sample = filter_IIR(
                    gain_HPF,
                    x, cx_HPF, 3,
                    y, cy_HPF, 3
                );
                // adjust the HPF gain so that the DAC output won’t overflow
                // Vary the frequency of the sine wave. Find the –3 dB frequency
                // What is the BW of this filter? 
                // Connect the ECG simulator with 10% 60-Hz noise. Are the output results what you expect? 
                break;


            case PART6: // Implement Band Pass Filter
                 new_output_sample = filter_IIR(
                    gain_BPF,
                    x, cx_BPF, 3,
                    y, cy_BPF, 3
                );
                // adjust the BPF gain so that the DAC output won’t overflow
                // Vary the frequency of the sine wave. Find the –3 dB frequency
                // What is the BW of this filter? 
                // Calculate the Q of this filter. How does it compare with the DigiScope model filter?
                // Connect the ECG simulator with 10% 60-Hz noise. Are the output results what you expect? 
                break;


            default :
                new_output_sample = new_input_sample;
                break;
        }

        // Store calculated value in output buffer
        y[0] = new_output_sample;

        // Send calculated value to DAC
        float dac_out = new_output_sample;
        if (dac_out < 0.0f) dac_out = 0.0f;
        if (dac_out > 1.0f) dac_out = 1.0f;
        MCP4728_write1(dac_out);

        // Drop HW flag (this allows visibility into the length of time this routine takes to run)
        gpio_put(FLAG_PIN, 0);

    }
}

#endif