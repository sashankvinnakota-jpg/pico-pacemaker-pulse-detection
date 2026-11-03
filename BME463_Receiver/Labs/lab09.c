// Paste Lab 9 Code Here When Provided.
/* @file lab09.c
 * @author Gavin Harwood
 * @author Zerind Spore
 * @author "YOUR NAME HERE"
 * @brief
 * @version 0.1
 * @date 2025-12-19
 */


#include "Active_Lab.h"
#include "main.h"




#if defined(lab09)






char APP_DESCRIPTION[] = "BME 463: Lab 09 - Basic QRS Detection";


// ----- PIN ASSIGNMENT DEFINITIONS -----


// SPI PINOUT
#define DAC_SPI       spi0
#define DAC_CS_PIN    17
#define DAC_SCK_PIN   18
#define DAC_MOSI_PIN  19
#define DAC_MISO_PIN  16


// Pin for PICO on board LED
#define LED_PIN PICO_DEFAULT_LED_PIN


// Pin for interrupt flag
#define FLAG_PIN  2  


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
// Implement values for filter coefficients ----------------------------
// ---------------------------------------------------------------------------------


#define DERIVATIVE_N  5
static float c_derivative[DERIVATIVE_N] = {2,1,0,-1,-2};   // Coefficients
static float a_derivative = .25;
static float x_derivative[DERIVATIVE_N] = {0};    




#define MWI_N  32
static float c_mwi[MWI_N] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
static float a_mwi = 0.031;
static float x_mwi[MWI_N] = {0};
// ---------------------------------------------------------------------------------


// Enumerated Type to select what part of the lab you are working on
typedef enum{
    PART1 = 0, // Reconstruction Filter
    PART2 = 1, // Derivative + Squaring + MWI Filter
    PART3 = 2, // Derivative + Squaring + MWI + Threshold Detector


} current_t;


// Initialize additional HW pins you wish to use here if needed
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


    // Start periodic sampling timer
    add_repeating_timer_us(
        -SAMPLE_PERIOD_US,
        samp_timer_cb,
        NULL,
        &samp_timer
    );
}










// ---------------------------------------------------------------------------------
// -------------------------ONLY MODIFY BELOW HERE----------------------------------
// ---------------------------------------------------------------------------------


// Implement the behavior described for this lab within main
// MUST COMPLETE filter_FIR in Filters.c BEFORE doing anything further
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
        float new_input_sample = (float)raw_adc_value / 4095.0f;


        float new_output_sample = 0.0f; // Computed in switch statement






        switch(running)
        {
            case PART1:


                MCP4728_write1(new_input_sample + 0.5f);


                break;


            case PART2: // deriv + squaring + mwi


                float y1 = 0.0;
                float squaring_output = 0.0;




                shift_right(x_derivative, DERIVATIVE_N);


                x_derivative[0] = new_input_sample;


                y1 = filter_FIR(a_derivative,
                    x_derivative,
                    c_derivative,
                    DERIVATIVE_N);
                // Send result to DAC
                MCP4728_write1(y1 + 0.5f);




                //Enter Squaring Parameters


                squaring_output = y1 * y1;
                MCP4728_write2(squaring_output + 0.5f);




                shift_right(x_mwi, MWI_N);


                x_mwi[0] = squaring_output;


                new_output_sample = filter_FIR(a_mwi,
                    x_mwi,
                    c_mwi,
                    MWI_N);


                // Send result to DAC
                MCP4728_write3(new_output_sample + 0.5f);
                // Record amplitude vs frequency
                break;


            case PART3: /// deriv + squaring + mwi + Threshold Detector




                y1 = 0.0;
                squaring_output = 0.0;




                shift_right(x_derivative, DERIVATIVE_N);


                x_derivative[0] = new_input_sample;


                y1 = filter_FIR(a_derivative,
                    x_derivative,
                    c_derivative,
                    DERIVATIVE_N)*20.0f; // Multiplied by 10.0 to increase visibility of output on DAC
                // Send result to DAC
                MCP4728_write1((y1 + 0.5f));




                //Enter Squaring Parameters
                squaring_output = y1 * y1;
                MCP4728_write2(squaring_output + 0.5f);




                shift_right(x_mwi, MWI_N);


                x_mwi[0] = squaring_output;


                new_output_sample = filter_FIR(a_mwi,
                    x_mwi,
                    c_mwi,
                    MWI_N);


                // Send result to DAC
                MCP4728_write3((new_output_sample + 0.5f)); // Multiplied by 6 to increase visibility of output on DAC
                // Record amplitude vs frequency


                float threshold = 0.01f; // Set an amplitude threshold for QRS detection (this may require tuning based on your input signal)


                if (new_output_sample > threshold)
                {
                    MCP4728_write4(1.0f); //Output a 1 for detected QRS complex
                }
                else
                {
                    MCP4728_write4(0.0f); //Output a 0 for no detection
                }
               


                break;


            default :
                new_output_sample = new_input_sample;
                MCP4728_write1(new_output_sample + 0.5f);
                break;
        }


        // Drop HW flag (this allows visibility into the length of time this routine takes to run)
        gpio_put(FLAG_PIN, 0);


    }
}
#endif

// Basic QRS Detector