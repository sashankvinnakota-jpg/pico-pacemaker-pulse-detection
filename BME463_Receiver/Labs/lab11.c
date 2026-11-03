#include "Active_Lab.h"
#include "main.h"
#include "pico/time.h"




#if defined(lab11)


char APP_DESCRIPTION[] = "BME 463: Lab 11 - Signal Averaging";


// ----- PIN ASSIGNMENT DEFINITIONS -----


// Pin for PICO on board LED
#define LED_PIN PICO_DEFAULT_LED_PIN
#define QRS_LED 3


// Pin for interrupt flag
#define FLAG_PIN 22


// ADC Pin and Channel
#define ADC_GPIO 26
#define ADC_CH   0


// ---- Timing and Timers ----
#define SAMPLE_PERIOD_US 5000   // 200 Hz
static repeating_timer_t samp_timer;
static volatile bool sample_flag = false;


static bool samp_timer_cb(repeating_timer_t *t)
{
    sample_flag = true;
    gpio_put(FLAG_PIN, 1);
    return true;
}


// ---------------------------------------------------------------------------------
// Filter coefficients / state
// ---------------------------------------------------------------------------------


// Low Pass Filter (kept in same style as your Pico lab10 code)
#define LOW_PASS_N 11
static float c_low_pass[LOW_PASS_N] = {1, 2, 3, 4, 5, 6, 5, 4, 3, 2, 1};
static float a_low_pass = 1.0f / 36.016f;
static float x_low_pass[LOW_PASS_N] = {0};


// High Pass Filter
#define HIGH_PASS_N 31
static float c_high_pass[HIGH_PASS_N] = {
    -0.03125f, -0.03125f, -0.03125f, -0.03125f, -0.03125f,
    -0.03125f, -0.03125f, -0.03125f, -0.03125f, -0.03125f,
    -0.03125f, -0.03125f, -0.03125f, -0.03125f, -0.03125f,
     0.96875f,  0.96875f,
    -0.03125f, -0.03125f, -0.03125f, -0.03125f, -0.03125f,
    -0.03125f, -0.03125f, -0.03125f, -0.03125f, -0.03125f,
    -0.03125f, -0.03125f, -0.03125f, -0.03125f
};
static float a_high_pass = 1.0f / 1.216f;
static float x_high_pass[HIGH_PASS_N] = {0};


// Derivative Filter
#define DERIVATIVE_N 5
static float c_derivative[DERIVATIVE_N] = {2.0f, 1.0f, 0.0f, -1.0f, -2.0f};
static float a_derivative = 1.0f / 4.0f;
static float x_derivative[DERIVATIVE_N] = {0};


// Moving Window Integrator
#define MWI_N 32
static float c_mwi[MWI_N] = {
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
};
static float a_mwi = 1.0f / 4.0f;
static float x_mwi[MWI_N] = {0};


// Adaptive Threshold Detector history
#define THRESHOLD_N 3
static float x_threshold[THRESHOLD_N] = {0.0f, 0.0f, 0.0f};


// Pan-Tompkins threshold state
static float peaki = 0.0f;
static float peakt = 0.0f;
static float spki = 0.4f;          // match old Nucleo init
static float npki = 0.0f;
static float thresholdi1 = 0.15f;  // match old Nucleo init


// Signal averaging buffer
#define AVG_N 18// Change to number of samples to sample over
#define AVG_BUF_LEN 500
static int max_index = 0;
static int min_index = AVG_BUF_LEN;
static float avg_buf[AVG_BUF_LEN] = {0};
static float running_buf[AVG_BUF_LEN] = {0};
static int run_index = 0;
static int N_index = 0;
static float time1 = 0.0f;


static bool avg_done = false;
static bool qrs_state = false;
static bool last_qrs_state = false;


// ---------------------------------------------------------------------------------


typedef enum{
    PART1 = 0, // Pass-through
    PART2 = 1, // Full PT chain + adaptive threshold
    PART3 = 2, // Signal averaging
} current_t;


// ---------------------------------------------------------------------------------
// Hardware init - changed to match lab06 style
// ---------------------------------------------------------------------------------
void app_init_hw(void){
    printf("%s\n", APP_DESCRIPTION);
    printf("Name: %s\n", NAME);


    // LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);


        // QRS LED
    gpio_init(QRS_LED);
    gpio_set_dir(QRS_LED, GPIO_OUT);
    gpio_put(QRS_LED, 0);


    // Interrupt timing flag
    gpio_init(FLAG_PIN);
    gpio_set_dir(FLAG_PIN, GPIO_OUT);
    gpio_put(FLAG_PIN, 0);


    // DAC (same style as lab06)
    MCP4728_init(i2c1, 14, 15, 400 * 1000, 0x60);


    // ADC
    adc_init();
    adc_gpio_init(ADC_GPIO);
    adc_select_input(ADC_CH);


    // 7-seg display like lab06
    SevSeg_init(spi1, 1, 10, 11, 12, 100000);
    SevSeg_setBrightness(80);
    Disp_Str("L 10");


    serial_start_boot_banner(APP_DESCRIPTION, NAME);


    // Initialize PT threshold state
    peaki = 0.0f;
    peakt = 0.0f;
    thresholdi1 = 0.15f;
    spki = 0.4f;
    npki = 0.0f;
    x_threshold[0] = 0.0f;
    x_threshold[1] = 0.0f;
    x_threshold[2] = 0.0f;


    add_repeating_timer_us(
        -SAMPLE_PERIOD_US,
        samp_timer_cb,
        NULL,
        &samp_timer
    );
}


// ---------------------------------------------------------------------------------
// Adaptive threshold detector
// This now closely matches the old Nucleo PanT_Thresh()
// ---------------------------------------------------------------------------------
static void panT_Thresh(void)
{
    // Track the current local peak
    if ((x_threshold[0] > x_threshold[2]) && (x_threshold[0] > peakt)) {
        peakt = x_threshold[0];
    }


    // QRS indicator is high whenever peakt > threshold
    if (peakt > thresholdi1) {
        gpio_put(QRS_LED, 1);
        MCP4728_write2(1.0f);  // useful scope/debug output
    }


    // Detect end of a candidate peak
    if ((x_threshold[0] <= x_threshold[2]) && (x_threshold[0] < 0.5f * peakt)) {
        peaki = peakt;


        // Turn the QRS indicator off here
        gpio_put(QRS_LED, 0);
        MCP4728_write2(0.0f);


        if (peaki > thresholdi1) {
            spki = 0.125f * peaki + 0.875f * spki;
        } else {
            npki = 0.125f * peaki + 0.875f * npki;
        }


        thresholdi1 = npki + 0.25f * (spki - npki);
        peakt = 0.0f;
    }
    MCP4728_write3(thresholdi1);
    MCP4728_write4(spki);


    // Shift threshold history exactly once
    x_threshold[2] = x_threshold[1];
    x_threshold[1] = x_threshold[0];
}


// ---------------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------------
void app_main(void)
{
    current_t running = PART3; // CHANGE TO PART YOU ARE ON IN LAB


    float new_output_sample = 0.0f;
    float low_pass_output = 0.0f;
    float high_pass_output = 0.0f;
    float derivative_output = 0.0f;
    float squaring_output = 0.0f;
    float mwi_output = 0.0f;


    while (true) {


        if (!sample_flag) {
            tight_loop_contents();
            continue;
        }
        sample_flag = false;


        uint16_t raw_adc_value = adc_read();
        float new_input_sample = (float)raw_adc_value / 4095.0f;


        switch (running)
        {
            case PART1:
                // Pass-through
                MCP4728_write1(new_input_sample);
                break;


            case PART2:
                // Full PT chain + adaptive threshold


                shift_right(x_high_pass, HIGH_PASS_N);
                x_high_pass[0] = new_input_sample;
                high_pass_output = filter_FIR(a_high_pass, x_high_pass, c_high_pass, HIGH_PASS_N);


                shift_right(x_low_pass, LOW_PASS_N);
                x_low_pass[0] = new_input_sample;
                low_pass_output = filter_FIR(a_low_pass, x_low_pass, c_low_pass, LOW_PASS_N);


                shift_right(x_derivative, DERIVATIVE_N);
                x_derivative[0] = low_pass_output;
                derivative_output = filter_FIR(a_derivative, x_derivative, c_derivative, DERIVATIVE_N);


                squaring_output = 100.0f * derivative_output * derivative_output;


                shift_right(x_mwi, MWI_N);
                x_mwi[0] = squaring_output;
                mwi_output = filter_FIR(a_mwi, x_mwi, c_mwi, MWI_N);


                // MWI output to DAC1
                MCP4728_write1(mwi_output);


                // Threshold detector input
                x_threshold[0] = mwi_output;
                panT_Thresh();
                break;


            case PART3:
                    // Full PT chain + adaptive threshold
                if (N_index < AVG_N) {
                    shift_right(x_high_pass, HIGH_PASS_N);
                    x_high_pass[0] = new_input_sample;
                    high_pass_output = filter_FIR(a_high_pass, x_high_pass, c_high_pass, HIGH_PASS_N);


                    shift_right(x_low_pass, LOW_PASS_N);
                    x_low_pass[0] = new_input_sample;
                    low_pass_output = filter_FIR(a_low_pass, x_low_pass, c_low_pass, LOW_PASS_N);


                    shift_right(x_derivative, DERIVATIVE_N);
                    x_derivative[0] = low_pass_output;
                    derivative_output = filter_FIR(a_derivative, x_derivative, c_derivative, DERIVATIVE_N);


                    squaring_output = 60.0f * derivative_output * derivative_output;


                    shift_right(x_mwi, MWI_N);
                    x_mwi[0] = squaring_output;
                    mwi_output = filter_FIR(a_mwi, x_mwi, c_mwi, MWI_N);


                    // MWI output to DAC1
                    MCP4728_write1(mwi_output);


                    // Threshold detector input
                    x_threshold[0] = mwi_output;
                    panT_Thresh();


                    // Save sample to running buffer while we are still averaging, and track the number of samples in the current beat with run_index
                    running_buf[run_index] = new_input_sample;
                    run_index++;


                    //Increment time 1
                    time1 += 0.005f;


                    // Heart rate logic - detect valid peaks and compute HR based on RR intervals
                    // Treat QRS output high as beat detected
                    qrs_state = gpio_get(QRS_LED);


                    // Rising edge only, so we do not count the same beat repeatedly, require minumum time to avoid detections on T waves
                    if (qrs_state && !last_qrs_state && time1 > 0.3f) {
                        if (N_index > 1) { // skip the first two beats since PT algorithm is still stabilizing
                            if (run_index < min_index) { //Update min_index
                                min_index = run_index;
                            }
                            for(int i = 0; i < run_index; i++) { // Update average buffer with new beat, and reset running buffer for next beat
                                avg_buf[i] += running_buf[i];
                                running_buf[i] = 0.0f;
                            }
                            for(int i = run_index; i < max_index; i++) { // Clear any remaining samples in the running buffer that are past the end of the current beat
                                running_buf[i] = 0.0f;
                            }
                        }
                        time1 = 0.0f; // reset time since last beat
                        run_index = 0; // reset running buffer index
                        N_index++; // increment beat count
                    }
                    last_qrs_state = qrs_state; // update last QRS state
                }
                else { // After enough beats have been averaged, output the average beat in a loop
                    if (N_index >= AVG_N && !avg_done) { // Finalize average buffer and prepare to output
                        run_index = 0;
                        avg_done = true;
                        for(int i = 0; i < min_index; i++) { // Finalize average buffer by dividing by N
                            avg_buf[i] /= (float)(AVG_N-2); // We started averaging from the 3rd beat, so divide by (N-2) instead of N
                        }
                    }


                    new_output_sample = avg_buf[run_index];
                    run_index++;
                    if (run_index >= min_index) {
                        run_index = 0;
                    }
                    MCP4728_write1(new_output_sample);
                }
                break;




            default:
                new_output_sample = new_input_sample;
                MCP4728_write1(new_output_sample+0.5f);
                break;
        }


        gpio_put(FLAG_PIN, 0);
    }
}


#endif
