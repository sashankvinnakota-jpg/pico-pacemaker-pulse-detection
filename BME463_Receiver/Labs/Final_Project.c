/**
 * @file Final_Project.c
 * @author Sashank Vinnakota
 * @brief BME 463 Final Project - Detection of Pacemaker Pulses in ECG Signals
 *        Modified Pan-Tompkins pipeline adapted for high-frequency pacemaker spike
 *        detection using: HPF -> Derivative -> Squaring -> MWI -> Adaptive Threshold
 *        Records tested: MIT-BIH 102, 104, 107, 113, 217
 * @version 1.0
 * @date 2026-04-19
 */

#include "Active_Lab.h"
#include "main.h"

#if defined(project)

char APP_DESCRIPTION[] = "BME 463: Final Project - Pacemaker Pulse Detection";

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define LED_PIN         PICO_DEFAULT_LED_PIN  // Onboard LED
#define PACE_LED        3                     // GPIO 3 - HIGH when pacemaker pulse detected
#define FLAG_PIN        22                    // Oscilloscope timing flag

// ADC
#define ADC_GPIO        26
#define ADC_CH          0

// ============================================================
// SAMPLING TIMER  (200 Hz -> 5000 us period)
// ============================================================

#define SAMPLE_PERIOD_US    5000
static repeating_timer_t    samp_timer;
static volatile bool        sample_flag = false;

static bool samp_timer_cb(repeating_timer_t *t)
{
    sample_flag = true;
    gpio_put(FLAG_PIN, 1);   // Rising edge marks start of sample processing
    return true;
}

// ============================================================
// FILTER COEFFICIENTS & STATE BUFFERS
// ============================================================

// -----------------------------------------------------------
// Stage 1: High-Pass Filter
//   Purpose: Remove baseline wander and slow ECG components
//            (P wave, T wave, QRS body) while preserving the
//            sharp high-frequency pacemaker spike edges.
//   Coefficients: DC-null HPF centered at sample 16
//   Normalization: a_high_pass scales output to ~unity gain
// -----------------------------------------------------------
#define HIGH_PASS_N     31
static const float c_high_pass[HIGH_PASS_N] = {
    -0.03125f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
     1.0f,    -1.0f,
     0.0f,     0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
     0.03125f,
     0.0f,     0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
     0.0f,     0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
     0.0f,     0.0f, 0.0f
};
// Gain normalization: sum of positive coefficients ~ 1.03125
static const float a_high_pass = 1.0f / 1.0f;   // tuned empirically
static float x_high_pass[HIGH_PASS_N] = {0};

// -----------------------------------------------------------
// Stage 2: Derivative Filter
//   Purpose: Emphasize slopes (edges) in the signal. Pacemaker
//            spikes have extremely steep slopes compared to the
//            QRS complex. This is the core feature extractor.
//   H(z) = (2 + z^-1 - z^-3 - 2z^-4) / 8
//   Dividing by 8 (a_derivative = 0.125) prevents integer
//   overflow before the squaring stage.
// -----------------------------------------------------------
#define DERIVATIVE_N    5
static const float c_derivative[DERIVATIVE_N] = {2.0f, 1.0f, 0.0f, -1.0f, -2.0f};
static const float a_derivative = 0.125f;        // 1/8 — prevents overflow before squaring
static float x_derivative[DERIVATIVE_N] = {0};

// -----------------------------------------------------------
// Stage 3: Squaring  (applied inline, not a filter buffer)
//   y[n] = x[n]^2
//   Makes all values positive and disproportionately amplifies
//   large slope changes (pacemaker spikes) vs smaller QRS slopes.
//   Scale factor 100.0 brings output into DAC-visible range.
// -----------------------------------------------------------
#define SQUARING_SCALE  100.0f

// -----------------------------------------------------------
// Stage 4: Moving Window Integrator (MWI)
//   Purpose: Smooth the squared derivative into a detectable
//            bump while suppressing inter-sample noise.
//   Window N=10 (50ms @ 200 sps) — increased from N=6 to
//   reduce noise visible on oscilloscope output while still
//   being narrow enough to preserve pacemaker spike energy.
//   A large window (like Pan-Tompkins N=32, 150ms) would
//   spread and bury the spike; N=10 is a practical compromise.
//   m[n] = (1/N) * sum(e[n-i], i=0..N-1)
// -----------------------------------------------------------
#define MWI_N           10
static const float c_mwi[MWI_N] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};
static const float a_mwi = 1.0f / (float)MWI_N;  // 1/10 normalization
static float x_mwi[MWI_N] = {0};

// ============================================================
// ADAPTIVE THRESHOLD STATE
// ============================================================

// -----------------------------------------------------------
// Three-sample history window for local peak detection
//   Peak condition: x_thresh[1] > x_thresh[2] AND
//                   x_thresh[1] > x_thresh[0]
//   (previous sample is greater than both neighbors)
// -----------------------------------------------------------
#define THRESH_HIST_N   3
static float x_thresh[THRESH_HIST_N] = {0.0f, 0.0f, 0.0f};

// Running signal and noise level estimates (exponential moving average)
//   spki  = signal peak estimate  (updated when peak > threshold)
//   npki  = noise peak estimate   (updated when peak < threshold)
//   Update formula: new = 0.125 * peak + 0.875 * old
//   0.125 = 1/8 (right shift 3),  0.875 = 7/8 (value - value>>3)
static float spki         = 0.3f;    // Initial signal estimate (tuned from DigiScope)
static float npki         = 0.0f;    // Initial noise estimate
static float thresholdi1  = 0.10f;   // Initial threshold = npki + 0.25*(spki-npki)
static float peakt        = 0.0f;    // Running local peak tracker
static float peaki        = 0.0f;    // Saved peak at end of candidate event

// -----------------------------------------------------------
// Threshold floor and ceiling
//
//   MIN_THRESHOLD: Prevents threshold collapsing to zero in
//   quiet segments. Set ~10% above typical noise floor seen
//   in DigiScope for record 102 MWI output in inter-beat regions.
//
//   MAX_THRESHOLD: Prevents threshold drifting so high that
//   real pacemaker pulses get missed. If spki overestimates
//   signal level (e.g. due to a motion artifact early on),
//   the threshold would otherwise become unreachably high.
//   Set to 80% of DAC full scale as a safe upper ceiling.
// -----------------------------------------------------------
#define MIN_THRESHOLD   0.02f
#define MAX_THRESHOLD   0.80f

// ============================================================
// BLANKING INTERVAL
//   After a pacemaker pulse is detected, ignore new detections
//   for BLANK_SAMPLES samples to prevent re-detecting the same
//   spike's ring-down artifact.
//   10 ms @ 200 sps = 2 samples (minimum meaningful blanking)
//   Using 4 samples (20ms) for safety margin.
// ============================================================
#define BLANK_SAMPLES   4
static int  blank_count  = 0;        // Counts down from BLANK_SAMPLES to 0
static bool in_blank     = false;    // True while blanking is active

// ============================================================
// INITIALIZATION WINDOW
//   During the first INIT_SAMPLES samples (2 seconds @ 200sps),
//   scan the MWI output to set initial sigLvl and noiseLvl
//   before attempting detection.
// ============================================================
#define INIT_SAMPLES    400
static int   init_count  = 0;        // Sample counter during init
static bool  init_done   = false;    // True after initialization complete
static float init_max    = 0.0f;     // Max MWI value seen during init
static float init_sum    = 0.0f;     // Sum for mean calculation during init

// ============================================================
// DETECTION COUNTER (for serial output / debugging)
// ============================================================
static int detection_count = 0;

// ============================================================
// HARDWARE INITIALIZATION
// ============================================================
void app_init_hw(void)
{
    printf("%s\n", APP_DESCRIPTION);
    printf("Authors: Henry Ahl & Sashank Vinnakota\n");
    printf("Project: Pacemaker Pulse Detection\n");

    // Onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // Pacemaker detection indicator LED
    gpio_init(PACE_LED);
    gpio_set_dir(PACE_LED, GPIO_OUT);
    gpio_put(PACE_LED, 0);

    // Oscilloscope timing flag
    gpio_init(FLAG_PIN);
    gpio_set_dir(FLAG_PIN, GPIO_OUT);
    gpio_put(FLAG_PIN, 0);

    // DAC: 4-channel I2C DAC (MCP4728)
    //   CH1: MWI output (filtered signal for oscilloscope)
    //   CH2: Detection pulse (HIGH when pacemaker detected)
    //   CH3: Adaptive threshold level (for oscilloscope visualization)
    //   CH4: Signal peak estimate spki (debugging)
    MCP4728_init(i2c1, 14, 15, 400 * 1000, 0x60);

    // ADC
    adc_init();
    adc_gpio_init(ADC_GPIO);
    adc_select_input(ADC_CH);

    // 7-segment display
    SevSeg_init(spi1, 1, 10, 11, 12, 100000);
    SevSeg_setBrightness(80);
    Disp_Str("PaCE");

    serial_start_boot_banner(APP_DESCRIPTION, NAME);

    // Initialize threshold state
    spki         = 0.3f;
    npki         = 0.0f;
    thresholdi1  = 0.10f;
    peakt        = 0.0f;
    peaki        = 0.0f;
    blank_count  = 0;
    in_blank     = false;
    init_count   = 0;
    init_done    = false;
    init_max     = 0.0f;
    init_sum     = 0.0f;
    detection_count = 0;

    // Start 200 Hz sampling timer
    add_repeating_timer_us(
        -SAMPLE_PERIOD_US,
        samp_timer_cb,
        NULL,
        &samp_timer
    );
}

// ============================================================
// ADAPTIVE THRESHOLD DETECTOR
//   Called once per sample with the current MWI output.
//   Implements Pan-Tompkins style adaptive thresholding adapted
//   for pacemaker spike detection.
//
//   Returns: true  if a new pacemaker pulse is detected this sample
//            false otherwise
// ============================================================
static bool pace_thresh_detect(float mwi_sample)
{
    bool detected = false;

    // --- Blanking interval management ---
    // Decrement blank counter each sample; clear flag when done
    if (in_blank) {
        blank_count--;
        if (blank_count <= 0) {
            in_blank    = false;
            blank_count = 0;
        }
        // Output threshold and spki to DAC for oscilloscope monitoring
        MCP4728_write3(thresholdi1);
        MCP4728_write4(spki);

        // Shift history
        x_thresh[2] = x_thresh[1];
        x_thresh[1] = x_thresh[0];
        x_thresh[0] = mwi_sample;
        return false;   // No detection possible during blanking
    }

    // --- Shift sample history ---
    // x_thresh[1] = previous sample (candidate peak center)
    // x_thresh[0] = current sample
    // x_thresh[2] = two samples ago
    x_thresh[2] = x_thresh[1];
    x_thresh[1] = x_thresh[0];
    x_thresh[0] = mwi_sample;

    // --- Local peak tracking ---
    // Update running peak if signal is rising and exceeds current peak
    if ((x_thresh[1] >= x_thresh[2]) && (x_thresh[1] > peakt)) {
        peakt = x_thresh[1];
    }

    // --- Peak end detection ---
    // Signal is now falling AND well below the tracked peak:
    // the previous sample was the peak - evaluate it now
    if ((x_thresh[1] <= x_thresh[2]) && (x_thresh[1] < 0.5f * peakt) && (peakt > 0.0f)) {
        peaki = peakt;   // Save the peak amplitude for classification

        if (peaki > thresholdi1) {
            // ---- SIGNAL PEAK: pacemaker pulse detected ----
            // Update signal level estimate:  spki = 0.125*peak + 0.875*spki
            spki    = 0.125f * peaki + 0.875f * spki;

            // Update threshold and clamp between floor and ceiling
            thresholdi1 = npki + 0.25f * (spki - npki);
            if (thresholdi1 < MIN_THRESHOLD) thresholdi1 = MIN_THRESHOLD;
            if (thresholdi1 > MAX_THRESHOLD) thresholdi1 = MAX_THRESHOLD;

            // Assert detection LED and DAC output
            gpio_put(PACE_LED, 1);
            MCP4728_write2(1.0f);   // CH2: detection pulse HIGH

            // Start blanking interval
            in_blank    = true;
            blank_count = BLANK_SAMPLES;

            detected = true;
            detection_count++;

            printf("Pacemaker pulse #%d detected\n", detection_count);

        } else {
            // ---- NOISE PEAK: update noise level estimate ----
            // npki = 0.125*peak + 0.875*npki
            npki    = 0.125f * peaki + 0.875f * npki;

            // Update threshold and clamp between floor and ceiling
            thresholdi1 = npki + 0.25f * (spki - npki);
            if (thresholdi1 < MIN_THRESHOLD) thresholdi1 = MIN_THRESHOLD;
            if (thresholdi1 > MAX_THRESHOLD) thresholdi1 = MAX_THRESHOLD;

            // Clear detection output
            gpio_put(PACE_LED, 0);
            MCP4728_write2(0.0f);   // CH2: detection pulse LOW
        }

        peakt = 0.0f;   // Reset local peak tracker for next event
    }

    // --- Output threshold and signal level to DAC for oscilloscope ---
    MCP4728_write3(thresholdi1);  // CH3: adaptive threshold level
    MCP4728_write4(spki);         // CH4: signal peak estimate

    return detected;
}

// ============================================================
// MAIN APPLICATION LOOP
// ============================================================
void app_main(void)
{
    // Filter stage intermediate values
    float high_pass_output  = 0.0f;
    float derivative_output = 0.0f;
    float squaring_output   = 0.0f;
    float mwi_output        = 0.0f;

    while (true) {

        // Wait for 200 Hz sample timer
        if (!sample_flag) {
            tight_loop_contents();
            continue;
        }
        sample_flag = false;

        // Read ADC and normalize to [0, 1]
        uint16_t raw_adc = adc_read();
        float new_sample = (float)raw_adc / 4095.0f;

        // ====================================================
        // FILTER CHAIN
        // ====================================================

        // --- Stage 1: High-Pass Filter ---
        // Eliminates baseline wander, P/T waves, and QRS body.
        // Passes only sharp high-frequency pacemaker spike edges.
        shift_right(x_high_pass, HIGH_PASS_N);
        x_high_pass[0]  = new_sample;
        high_pass_output = filter_FIR(a_high_pass,
                                      x_high_pass,
                                      c_high_pass,
                                      HIGH_PASS_N);

        // --- Stage 2: Derivative Filter ---
        // Enhances slope features. Pacemaker spikes have much
        // steeper slopes than QRS complexes after HPF.
        // Divided by 8 (a_derivative=0.125) to prevent overflow
        // before the squaring stage.
        shift_right(x_derivative, DERIVATIVE_N);
        x_derivative[0]  = high_pass_output;
        derivative_output = (filter_FIR(a_derivative,
                                       x_derivative,
                                       c_derivative,
                                       DERIVATIVE_N)); // Additional empirical scaling to bring derivative output into visible range on DAC

        // --- Stage 3: Squaring ---
        // y[n] = (x[n])^2 * scale
        // Makes all values positive. Disproportionately amplifies
        // large pacemaker slope values vs smaller QRS residuals.
        squaring_output = derivative_output * derivative_output * SQUARING_SCALE;

        // --- Stage 4: Moving Window Integrator (N=6, 30ms) ---
        // Smooths spike energy into a detectable bump.
        // Small window preserves narrow pacemaker pulse energy.
        shift_right(x_mwi, MWI_N);
        x_mwi[0]   = squaring_output;
        mwi_output = (filter_FIR(a_mwi, x_mwi, c_mwi, MWI_N))*25.0f; // Additional empirical scaling to bring MWI output into visible range on DAC

        // Output MWI signal to DAC CH1 (oscilloscope channel 1)
        MCP4728_write1(mwi_output);

        // ====================================================
        // INITIALIZATION WINDOW (first 2 seconds = 400 samples)
        // Scans MWI output to set initial threshold estimates
        // before attempting any detection.
        // ====================================================
        if (!init_done) {
            if (mwi_output > init_max) {
                init_max = mwi_output;
            }
            init_sum += mwi_output;
            init_count++;

            if (init_count >= INIT_SAMPLES) {
                float init_mean = init_sum / (float)INIT_SAMPLES;
                // Signal estimate: largest peak seen * 0.25
                spki = init_max * 0.25f;
                // Noise estimate: mean * 0.1 (baseline floor)
                npki = init_mean * 0.1f;
                // Initial threshold: 25% between noise and signal
                thresholdi1 = npki + 0.25f * (spki - npki);
                if (thresholdi1 < MIN_THRESHOLD) {
                    thresholdi1 = MIN_THRESHOLD;
                }
                init_done = true;
                printf("Init complete: spki=%.4f npki=%.4f thr=%.4f\n",
                       spki, npki, thresholdi1);
            }

            // Still output threshold markers during init for scope visibility
            MCP4728_write2(0.0f);
            MCP4728_write3(thresholdi1);
            MCP4728_write4(spki);
            gpio_put(FLAG_PIN, 0);
            continue;   // Skip detection until initialized
        }

        // ====================================================
        // STAGE 5: ADAPTIVE THRESHOLD DETECTION
        // Pass MWI output to detector. Returns true on detection.
        // ====================================================
        pace_thresh_detect(mwi_output);

        // Drop oscilloscope timing flag (marks end of processing)
        gpio_put(FLAG_PIN, 0);
    }
}

#endif