// Paste Lab 10 Code Here when provided.
 //* @author Gavin Harwood
 //* @author Zerind Spore
 //* @author "YOUR NAME HERE"
 //* @brief BME 463 Lab 10 - Basic QRS Detection
 //* @version 0.2
 //* @date 2025-12-19
 //*/


#include "Active_Lab.h"
#include "main.h"


#if defined(lab10)


char APP_DESCRIPTION[] = "BME 463: Lab 10 - Basic QRS Detection";


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
static float a_low_pass = 1.0f / 35.98f;
static float x_low_pass[LOW_PASS_N] = {0};


// High Pass Filter
#define HIGH_PASS_N 32
static float c_high_pass[HIGH_PASS_N] = {
    -0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,
    -0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,
     0.96875f,
    -0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,
    -0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f,-0.03125f
};
static float a_high_pass = 1.0f / 1.2f;
static float x_high_pass[HIGH_PASS_N] = {0};


// Derivative Filter
#define DERIVATIVE_N 5
static float c_derivative[DERIVATIVE_N] = {2, 1, 0, -1, -2};
static float a_derivative = 0.25f;
static float x_derivative[DERIVATIVE_N] = {0};


// Moving Window Integrator
#define MWI_N 32
static float c_mwi[MWI_N] = {
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
};
static float a_mwi = 0.031f;
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


// Heart rate detection / averaging
#define HEART_RATE_N 8
static float x_heart_rate[HEART_RATE_N] = {0.0f};
static float time1 = 0.0f;     // seconds since last valid beat
float rr_avg = 0.0f;
static float hr = 0.0f;
static bool last_qrs_state = false;
bool qrs_state = false;


// ---------------------------------------------------------------------------------
typedef enum{
    PART1 = 0, // Reconstruction / debug
    PART2 = 1, // PT chain without adaptive threshold emphasis
    PART3 = 2, // PT chain + adaptive threshold
    PART4 = 3, // Full PT + threshold + heart rate logic
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


    add_repeating_timer_us(
        -SAMPLE_PERIOD_US,
        samp_timer_cb,
        NULL,
        &samp_timer
    );
}

size_t cc = 0;

// ---------------------------------------------------------------------------------
// Adaptive threshold detector
// ---------------------------------------------------------------------------------
static void panT_Thresh(void)
{
    // Track the current local peak: update peakt when signal is rising
    // (current sample > two samples prior) and exceeds current peak
    if ((x_threshold[0] >= x_threshold[2]) && (x_threshold[0] > peakt)) {
        peakt = x_threshold[0];
    }

    // QRS indicator is high whenever the running peak exceeds the threshold
    if (peakt > thresholdi1) {
        gpio_put(QRS_LED, 1);
        cc = 0;
    }

    // Detect end of candidate peak: signal falling and well below peak
    if ((x_threshold[0] <= x_threshold[2]) && (x_threshold[0] < 0.5f * peakt)) {
        // Turn QRS indicator off
        gpio_put(QRS_LED, 0);

        // Save the peak amplitude for classification
        peaki = peakt;

        // Classify peak as signal (QRS) or noise and update running estimates
        if (peaki > thresholdi1) {
            spki = 0.125f * peaki + 0.875f * spki;   // signal peak estimate
        } else {
            npki = 0.125f * peaki + 0.875f * npki;   // noise peak estimate
        }

        // Update adaptive threshold and reset local peak tracker
        thresholdi1 = npki + 0.25f * (spki - npki);
        peakt = 0.0f;
    }

    // Output current threshold and signal peak estimate for debugging
    MCP4728_write3(thresholdi1);
    MCP4728_write4(spki);

    // Shift threshold history: [2] <- [1] <- [0]
    x_threshold[2] = x_threshold[1];
    x_threshold[1] = x_threshold[0];

    cc++;
    if (cc > 1000) { // Example condition, adjust as needed
        cc = 0;
         peaki = 0.0f;
peakt = 0.0f;
 spki = 0.0f;          // match old Nucleo init
 npki = 0.0f;
thresholdi1 = 0.0f;  // match old Nucleo init
    }

}


// ---------------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------------
void app_main(void)
{
    current_t running = PART4; // CHANGE TO PART YOU ARE ON IN LAB


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
                // --- Stage 1: Low Pass Filter ---
                shift_right(x_low_pass, LOW_PASS_N);
                x_low_pass[0] = new_input_sample;
                low_pass_output = filter_FIR(a_low_pass, x_low_pass, c_low_pass, LOW_PASS_N);

                // --- Stage 2: High Pass Filter (bandpass with LPF) ---
                shift_right(x_high_pass, HIGH_PASS_N);
                x_high_pass[0] = low_pass_output;
                high_pass_output = filter_FIR(a_high_pass, x_high_pass, c_high_pass, HIGH_PASS_N);

                // --- Stage 3: Derivative Filter ---
                shift_right(x_derivative, DERIVATIVE_N);
                x_derivative[0] = high_pass_output;
                derivative_output = filter_FIR(a_derivative, x_derivative, c_derivative, DERIVATIVE_N);

                // --- Stage 4: Squaring (point-wise nonlinearity) ---
                squaring_output = derivative_output * derivative_output * 100.0f;

                // --- Stage 5: Moving Window Integrator ---
                shift_right(x_mwi, MWI_N);
                x_mwi[0] = squaring_output;
                mwi_output = filter_FIR(a_mwi, x_mwi, c_mwi, MWI_N);

                // Output MWI result to DAC channel RA
                MCP4728_write1(mwi_output);
                break;


            case PART3: // Full Pan-Tompkins with Adaptive Thresholding
                // --- Full PT chain (same as PART2) ---
                shift_right(x_low_pass, LOW_PASS_N);
                x_low_pass[0] = new_input_sample;
                low_pass_output = filter_FIR(a_low_pass, x_low_pass, c_low_pass, LOW_PASS_N);

                shift_right(x_high_pass, HIGH_PASS_N);
                x_high_pass[0] = low_pass_output;
                high_pass_output = filter_FIR(a_high_pass, x_high_pass, c_high_pass, HIGH_PASS_N);

                shift_right(x_derivative, DERIVATIVE_N);
                x_derivative[0] = high_pass_output;
                derivative_output = filter_FIR(a_derivative, x_derivative, c_derivative, DERIVATIVE_N);

                squaring_output = derivative_output * derivative_output* 100.0f;

                shift_right(x_mwi, MWI_N);
                x_mwi[0] = squaring_output;
                mwi_output = filter_FIR(a_mwi, x_mwi, c_mwi, MWI_N);

                // MWI output to DAC channel RA
                MCP4728_write1(mwi_output);

                // Threshold detector input is set to MWI output
                x_threshold[0] = mwi_output;

                // Call Pan-Tompkins threshold function
                panT_Thresh();

                break;
               
            case PART4: // PT + threshold + heart rate
                // --- Full PT chain (same as PART3) ---
                shift_right(x_low_pass, LOW_PASS_N);
                x_low_pass[0] = new_input_sample;
                low_pass_output = filter_FIR(a_low_pass, x_low_pass, c_low_pass, LOW_PASS_N);

                shift_right(x_high_pass, HIGH_PASS_N);
                x_high_pass[0] = low_pass_output;
                high_pass_output = filter_FIR(a_high_pass, x_high_pass, c_high_pass, HIGH_PASS_N);

                shift_right(x_derivative, DERIVATIVE_N);
                x_derivative[0] = high_pass_output;
                derivative_output = filter_FIR(a_derivative, x_derivative, c_derivative, DERIVATIVE_N);

                squaring_output = derivative_output * derivative_output * 200.0f;

                shift_right(x_mwi, MWI_N);
                x_mwi[0] = squaring_output;
                mwi_output = filter_FIR(a_mwi, x_mwi, c_mwi, MWI_N);

                MCP4728_write1(mwi_output);

                x_threshold[0] = mwi_output;
                panT_Thresh();

                // Advance RR interval timer by one sample period (200 Hz -> 0.005 s)
                time1 += (SAMPLE_PERIOD_US / 1000000.0f);

                // Determine current QRS state from the LED pin
                qrs_state = (bool)gpio_get(QRS_LED);

                // Rising edge only: new beat detected this sample
                if (qrs_state && !last_qrs_state) {
                    // Shift RR interval buffer and store latest interval
                    shift_right(x_heart_rate, HEART_RATE_N);
                    x_heart_rate[0] = time1;

                    // Average RR interval over the last 8 beats
                    rr_avg = 0.0f;
                    for (int i = 0; i < HEART_RATE_N; i++) {
                        rr_avg += x_heart_rate[i];
                    }
                    rr_avg /= (float)HEART_RATE_N;

                    // Heart rate in BPM
                    hr = 60.0f / rr_avg;

                    printf("HR: %3.1f bpm\r\n", hr);

                    // Reset inter-beat timer
                    time1 = 0.0f;
                }

                // Update last QRS state for next iteration
                last_qrs_state = qrs_state;
                break;


            default:
                new_output_sample = new_input_sample;
                MCP4728_write1(new_output_sample);
                break;
        }


        gpio_put(FLAG_PIN, 0);
    }
}


#endif


// Complete QRS Detector