// Paste Lab 8 Code Here When Provided.
// Paste Lab 8 Code Here When Provided.
// QRS Filters
/**
 * @file lab08.c
 * @author Gavin Harwood
 * @author Zerind Spore
 * @author "YOUR NAME HERE"
 * @brief
 * @version 0.2
 * @date 2025-12-17
 */


#include "Active_Lab.h"
#include "main.h"


#if defined(lab08)


char APP_DESCRIPTION[] = "BME 463: Lab 08 - Integer Filters";


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
// Implement values for filter coefficients ----------------------------
// ---------------------------------------------------------------------------------


// Low Pass Filter
#define LOW_PASS_N  11
static float c_low_pass[LOW_PASS_N] = {1,2,3,4,5,6,5,4,3,2,1};   // Coefficients
static float a_low_pass = 1.0/35.98;          
static float x_low_pass[LOW_PASS_N] = {0};     // Input buffer


// High Pass Filter
#define HIGH_PASS_N  32
static float c_high_pass[HIGH_PASS_N] = {-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,0.96875,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125,-0.03125};
static float a_high_pass =  1.0/1.2;
static float x_high_pass[HIGH_PASS_N] = {0};


// Integer Filter
#define INTEGER_N 21 // You will need to change this based on your filter design
static float c_integer[INTEGER_N] = {1,2,1,-2,-4,-2,3,6,3,-4,-8,-4,3,6,3,-2,-4,-2,1,2,1};
static float a_integer =  0.0207;
static float x_integer[INTEGER_N] = {0};


// ---------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------
// --------------------------------------------------------------------------------




// Enumerated Type to select what part of the lab you are working on
typedef enum{
    PART1 = 0, // Reconstruction Filter
    PART2 = 1, // Low Pass Filter
    PART3 = 2, // High Pass Filter
    PART4 = 3, // Bandpass Filter (Low + High Pass Filters)
    PART5 = 4, // Digiscope Integer Filter


} current_t;


// Initialize additional HW pins you wish to use here if needed
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
    Disp_Str("L 08");
    // Print to serial your name and which lab is running
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
    Disp_Str("LAB8");
    current_t running = PART5; // CHANGE TO PART YOU ARE ON IN LAB


    while (true){


        if (!sample_flag) { // Wait until new sample
            tight_loop_contents();
            continue;
        }
        sample_flag = false; // Clear flag if high


        // Read ADC and place into input buffer
        uint16_t raw_adc_value = adc_read();
        float new_input_sample = (float)raw_adc_value / 4095.0f - 0.5f; // Center around 0V


        float new_output_sample = 0.0f; // Computed in switch statement


        switch(running)
        {
            case PART1:


                MCP4728_write1(new_input_sample + 0.5f);


                break;


            case PART2: // Low Pass Filter


                shift_right(x_low_pass, LOW_PASS_N);
                x_low_pass[0] = new_input_sample;
                float y1;
                y1= filter_FIR(a_low_pass,x_low_pass,c_low_pass, LOW_PASS_N);
                
                

                // Send result to DAC
                MCP4728_write1(y1+0.5f);
                break;
                
               


            case PART3: // High Pass Filter
                // Implement High Pass Filter here


                // Send result to DAC
               
           
                shift_right(x_high_pass, HIGH_PASS_N);
                x_high_pass[0] = new_input_sample;
                float w1;
                w1= filter_FIR(a_high_pass,x_high_pass,c_high_pass, HIGH_PASS_N);
                
                

                // Send result to DAC
                MCP4728_write1(w1+0.5f);
                break;

  case PART4:  // Band Pass Filter (LPF + HPF cascade)
    // Step 1: Low Pass Filter
    shift_right(x_low_pass, LOW_PASS_N);
    x_low_pass[0] = new_input_sample;
    new_output_sample = filter_FIR(a_low_pass, x_low_pass, c_low_pass, LOW_PASS_N);
    
    // Step 2: High Pass Filter (input is LPF output)
    shift_right(x_high_pass, HIGH_PASS_N);
    x_high_pass[0] = new_output_sample; 
    float x; // ← This is the LPF output
    x = filter_FIR(a_high_pass, x_high_pass, c_high_pass, HIGH_PASS_N);
    
    // Step 3: Output cascaded result
    MCP4728_write1(x + 0.5f);
    break;

             // Band Pass Filter (Low + High Pass Filters)
                // Implement Band Pass Filter here
 
                // Send Low Pass filter result to DAC




                // Send Low Pass + High Pass filter result to DAC
  
                break;




            case PART5: // Digiscope Integer Filter



  shift_right(x_integer, INTEGER_N);
                x_integer[0] = new_input_sample;
                float l1;
                l1= filter_FIR(a_integer,x_integer,c_integer, INTEGER_N);
                
                

                // Send result to DAC
                MCP4728_write1(l1+0.5f);
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

// QRS Filters