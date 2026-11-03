// Paste Lab 12 Code Here when provided.
#include "Active_Lab.h"
#include "main.h"


#if defined(lab12)


char APP_DESCRIPTION[] = "BME 463: Lab 12 - MiniRPNet Learning Model";


#include "mini_rpnet_weights.h"
#include <string.h>
#include <math.h>


// ---- Pin definitions ----
#define LED_PIN          PICO_DEFAULT_LED_PIN
#define QRS_LED          3
#define FLAG_PIN         22
#define ADC_GPIO         26
#define ADC_CH           0
#define SAMPLE_PERIOD_US 5000
#define SAMPLE_PERIOD_S  0.005f


static repeating_timer_t samp_timer;
static volatile bool sample_flag = false;


static bool samp_timer_cb(repeating_timer_t *t) {
    sample_flag = true;
    gpio_put(FLAG_PIN, 1);
    return true;
}


// =========================================================
// Window
// Inference runs every INFER_STRIDE samples.
// Output is the CENTER sample of the DT window so it
// corresponds to the sample that entered WINDOW_SIZE/2
// samples ago — delay buffer matches this exactly.
// =========================================================
#define WINDOW_SIZE   200
#define INFER_STRIDE  20    // run inference every 20 samples
                            // = 10x per second = 100ms cadence
                            // fast enough to look continuous
                            // but not so fast it can't keep up


// ---- Sliding window ----
static float win[WINDOW_SIZE] = {0.0f};
static int   win_count = 0;   // counts up to INFER_STRIDE


// ---- DT output — ring buffer of center values ----
// Each inference produces one center DT value.
// We store them and stream them out one per sample tick
// by linearly interpolating between consecutive values
// so the DAC output is smooth between inferences.
#define DT_RING_LEN  16
static float dt_ring[DT_RING_LEN] = {0.5f};
static int   dt_ring_write = 0;
static int   dt_ring_read  = 0;
static float dt_interp_from = 0.5f;
static float dt_interp_to   = 0.5f;
static int   dt_interp_pos  = 0;   // 0..INFER_STRIDE-1


// ---- Delay buffer ----
// Delay = WINDOW_SIZE/2 samples so the ECG output lines up
// with the CENTER of the inference window
#define DELAY_LEN (WINDOW_SIZE / 2)
static float delay_buf[DELAY_LEN];
static int   delay_pos = 0;


// ---- Heart rate ----
#define HEART_RATE_N 8
static float x_heart_rate[HEART_RATE_N] = {0.0f};
static float time1  = 0.0f;
static float rr_avg = 0.0f;
static float hr     = 0.0f;


// ---- QRS pulse ----
#define QRS_PULSE_DURATION 20
static int qrs_pulse_counter = 0;


// ---- DT smoothing ----
#define DT_SMOOTH_N 5
static float dt_smooth_hist[DT_SMOOTH_N] = {0.5f,0.5f,0.5f,0.5f,0.5f};
static int   dt_smooth_idx = 0;


static float dt_smooth_update(float v) {
    dt_smooth_hist[dt_smooth_idx] = v;
    dt_smooth_idx = (dt_smooth_idx + 1) % DT_SMOOTH_N;
    float s = 0.0f;
    for (int i = 0; i < DT_SMOOTH_N; i++) s += dt_smooth_hist[i];
    return s / DT_SMOOTH_N;
}


// =========================================================
// Valley detector
// =========================================================
#define VD_LP_ALPHA    0.15f
#define VD_RISE_RATIO  1.3f
#define VD_REFRACTORY  40


static float vd_lp      = 0.5f;
static float vd_min     = 1.0f;
static float vd_spk     = 0.25f;
static float vd_npk     = 0.75f;
static float vd_thresh1 = 0.5f;
static float vd_thresh2 = 0.375f;
static int   vd_refract = 0;
static int   vd_since   = 0;
static float vd_rr_avg  = 0.0f;
static bool  vd_falling = false;


#define VD_HIST 4
static float vd_hist[VD_HIST] = {0.5f,0.5f,0.5f,0.5f};


static bool vd_update(float raw) {
    vd_lp = VD_LP_ALPHA * raw + (1.0f - VD_LP_ALPHA) * vd_lp;
    float s = vd_lp;
    vd_hist[3]=vd_hist[2]; vd_hist[2]=vd_hist[1];
    vd_hist[1]=vd_hist[0]; vd_hist[0]=s;
    vd_since++;
    if (vd_refract > 0) { vd_refract--; return false; }
    bool falling = (vd_hist[0] < vd_hist[1]);
    if (falling && !vd_falling) vd_min = s;
    if (falling && s < vd_min)  vd_min = s;
    vd_falling = falling;
    bool rose = (s > vd_min * VD_RISE_RATIO)
             && (vd_hist[1] > vd_hist[2]);
    if (!rose) return false;
    float thr = vd_thresh1;
    if (vd_rr_avg > 0.0f) {
        int timeout = (int)(vd_rr_avg / SAMPLE_PERIOD_S * 1.66f);
        if (vd_since > timeout) thr = vd_thresh2;
    }
    bool sig = (vd_min < thr);
    if (sig) {
        vd_spk    = 0.125f * vd_min + 0.875f * vd_spk;
        vd_refract    = VD_REFRACTORY;
        vd_rr_avg = 0.875f * vd_rr_avg
                  + 0.125f * (float)vd_since * SAMPLE_PERIOD_S;
        vd_since  = 0;
    } else {
        vd_npk = 0.125f * vd_min + 0.875f * vd_npk;
    }
    vd_min     = 1.0f;
    vd_falling = false;
    vd_thresh1 = vd_npk - 0.25f * (vd_npk - vd_spk);
    vd_thresh2 = 0.75f * vd_thresh1;
    return sig;
}


// =========================================================
// Normalization
// =========================================================
static float norm_mean = 0.0f;
static float norm_m2   = 0.0f;
static float norm_var  = 1.0f;
static int   norm_n    = 0;


static float normalize_for_model(float x) {
    norm_n++;
    float d  = x - norm_mean;
    norm_mean += d / (float)norm_n;
    norm_m2   += d * (x - norm_mean);
    norm_var   = (norm_n > 1) ? norm_m2/(float)(norm_n-1) : 1.0f;
    if (norm_n > 4000) { norm_n=2000; norm_m2=norm_var*norm_n; }
    float std = sqrtf(norm_var);
    if (std < 1e-6f) std = 1.0f;
    return (x - norm_mean) / std;
}


// =========================================================
// Neural net primitives
// =========================================================
static void leaky_relu(float *x, int n) {
    for (int i = 0; i < n; i++)
        x[i] = x[i] >= 0.0f ? x[i] : 0.2f * x[i];
}
static void sigmoid_inplace(float *x, int n) {
    for (int i = 0; i < n; i++)
        x[i] = 1.0f / (1.0f + expf(-x[i]));
}
static void batch_norm(float *x, int ch, int len,
                       const float *g, const float *b,
                       const float *m, const float *v) {
    for (int c = 0; c < ch; c++) {
        float sc = g[c] / sqrtf(v[c] + BN_EPSILON);
        float bi = b[c] - sc * m[c];
        for (int i = 0; i < len; i++)
            x[c*len+i] = sc * x[c*len+i] + bi;
    }
}
static void conv1d(const float *in, int ic, int il,
                   const float *w, int oc, int k,
                   const float *b,
                   float *out, int ol, int st, int pad) {
    for (int o = 0; o < oc; o++)
        for (int i = 0; i < ol; i++) {
            float acc = b[o];
            for (int c = 0; c < ic; c++)
                for (int kk = 0; kk < k; kk++) {
                    int ip = i*st - pad + kk;
                    if (ip >= 0 && ip < il)
                        acc += in[c*il+ip] * w[o*ic*k+c*k+kk];
                }
            out[o*ol+i] = acc;
        }
}
static void conv1d_s2(const float *in, int ic, int il,
                      const float *w, int oc,
                      const float *b, float *out) {
    conv1d(in,ic,il,w,oc,4,b,out,il/2,2,1);
}
static void convT(const float *in, int ic, int il,
                  const float *w, int oc,
                  const float *b, float *out, int ol) {
    memset(out,0,oc*ol*sizeof(float));
    for (int c=0;c<ic;c++) for (int i=0;i<il;i++)
        for (int o=0;o<oc;o++) for (int k=0;k<4;k++) {
            int op=i*2-1+k;
            if(op>=0&&op<ol)
                out[o*ol+op]+=in[c*il+i]*w[c*oc*4+o*4+k];
        }
    for(int o=0;o<oc;o++) for(int i=0;i<ol;i++) out[o*ol+i]+=b[o];
}
static void pw(const float *in, int ic, int l,
               const float *w, int oc,
               const float *b, float *out) {
    conv1d(in,ic,l,w,oc,1,b,out,l,1,0);
}
static void cat(const float *a, int ac,
                const float *b, int bc, int l, float *out) {
    memcpy(out,a,ac*l*sizeof(float));
    memcpy(out+ac*l,b,bc*l*sizeof(float));
}
static void addi(float *a, const float *b, int n) {
    for(int i=0;i<n;i++) a[i]+=b[i];
}
static void irb(const float *in, int ch, int l,
                const float *b1w,const float *b1b,
                const float *b1cw,const float *b1cb,
                const float *b2w,const float *b2b,
                const float *b2cw,const float *b2cb,
                const float *b3w,const float *b3b,
                const float *b3cw,const float *b3cb,
                const float *b4w,const float *b4b,
                const float *bng,const float *bnb,
                const float *bnm,const float *bnv,
                const float *rw,const float *rb,
                float *tm,float *to,float *tr) {
    int m=ch/4;
    pw(in,ch,l,b1w,m,b1b,tm);
    conv1d(tm,m,l,b1cw,m,5,b1cb,tr,      l,1,2);
    pw(in,ch,l,b2w,m,b2b,tm);
    conv1d(tm,m,l,b2cw,m,7,b2cb,tr+m*l,  l,1,3);
    pw(in,ch,l,b3w,m,b3b,tm);
    conv1d(tm,m,l,b3cw,m,9,b3cb,tr+2*m*l,l,1,4);
    pw(in,ch,l,b4w,m,b4b,tr+3*m*l);
    batch_norm(tr,ch,l,bng,bnb,bnm,bnv);
    leaky_relu(tr,ch*l);
    pw(in,ch,l,rw,ch,rb,to);
    addi(tr,to,ch*l);
    memcpy(to,tr,ch*l*sizeof(float));
}


// =========================================================
// Activation buffers
// =========================================================
#define F 8
static float e1[F*100]={0};    static float e2[F*2*50]={0};
static float e3[F*4*25]={0};   static float e4[F*8*12]={0};
static float d1u[F*4*25]={0};  static float d1c[F*8*25]={0};
static float d1o[F*8*25]={0};
static float d2u[F*2*50]={0};  static float d2c[F*4*50]={0};
static float d2o[F*4*50]={0};
static float d3u[F*100]={0};   static float d3c[F*2*100]={0};
static float d3o[F*2*100]={0};
static float d4o[F*200]={0};   static float ob[200]={0};
static float sm[F*8*25]={0};
static float so[F*8*25]={0};
static float sr[F*8*25]={0};


// =========================================================
// Forward pass — returns center sample of DT output
// =========================================================
static float run_inference_center(void) {
    conv1d_s2(win,1,200,enc1_0_weight,F,enc1_0_bias,e1);
    batch_norm(e1,F,100,enc1_1_weight,enc1_1_bias,
               enc1_1_running_mean,enc1_1_running_var);
    leaky_relu(e1,F*100);
    irb(e1,F,100,
        enc1_3_branch1_0_weight,enc1_3_branch1_0_bias,
        enc1_3_branch1_1_weight,enc1_3_branch1_1_bias,
        enc1_3_branch2_0_weight,enc1_3_branch2_0_bias,
        enc1_3_branch2_1_weight,enc1_3_branch2_1_bias,
        enc1_3_branch3_0_weight,enc1_3_branch3_0_bias,
        enc1_3_branch3_1_weight,enc1_3_branch3_1_bias,
        enc1_3_branch4_weight,enc1_3_branch4_bias,
        enc1_3_bn_weight,enc1_3_bn_bias,
        enc1_3_bn_running_mean,enc1_3_bn_running_var,
        enc1_3_residual_proj_weight,enc1_3_residual_proj_bias,
        sm,so,sr);
    memcpy(e1,so,F*100*sizeof(float));


    conv1d_s2(e1,F,100,enc2_0_weight,F*2,enc2_0_bias,e2);
    batch_norm(e2,F*2,50,enc2_1_weight,enc2_1_bias,
               enc2_1_running_mean,enc2_1_running_var);
    leaky_relu(e2,F*2*50);
    irb(e2,F*2,50,
        enc2_3_branch1_0_weight,enc2_3_branch1_0_bias,
        enc2_3_branch1_1_weight,enc2_3_branch1_1_bias,
        enc2_3_branch2_0_weight,enc2_3_branch2_0_bias,
        enc2_3_branch2_1_weight,enc2_3_branch2_1_bias,
        enc2_3_branch3_0_weight,enc2_3_branch3_0_bias,
        enc2_3_branch3_1_weight,enc2_3_branch3_1_bias,
        enc2_3_branch4_weight,enc2_3_branch4_bias,
        enc2_3_bn_weight,enc2_3_bn_bias,
        enc2_3_bn_running_mean,enc2_3_bn_running_var,
        enc2_3_residual_proj_weight,enc2_3_residual_proj_bias,
        sm,so,sr);
    memcpy(e2,so,F*2*50*sizeof(float));


    conv1d_s2(e2,F*2,50,enc3_0_weight,F*4,enc3_0_bias,e3);
    batch_norm(e3,F*4,25,enc3_1_weight,enc3_1_bias,
               enc3_1_running_mean,enc3_1_running_var);
    leaky_relu(e3,F*4*25);
    irb(e3,F*4,25,
        enc3_3_branch1_0_weight,enc3_3_branch1_0_bias,
        enc3_3_branch1_1_weight,enc3_3_branch1_1_bias,
        enc3_3_branch2_0_weight,enc3_3_branch2_0_bias,
        enc3_3_branch2_1_weight,enc3_3_branch2_1_bias,
        enc3_3_branch3_0_weight,enc3_3_branch3_0_bias,
        enc3_3_branch3_1_weight,enc3_3_branch3_1_bias,
        enc3_3_branch4_weight,enc3_3_branch4_bias,
        enc3_3_bn_weight,enc3_3_bn_bias,
        enc3_3_bn_running_mean,enc3_3_bn_running_var,
        enc3_3_residual_proj_weight,enc3_3_residual_proj_bias,
        sm,so,sr);
    memcpy(e3,so,F*4*25*sizeof(float));


    conv1d_s2(e3,F*4,25,enc4_0_weight,F*8,enc4_0_bias,e4);
    batch_norm(e4,F*8,12,enc4_1_weight,enc4_1_bias,
               enc4_1_running_mean,enc4_1_running_var);
    leaky_relu(e4,F*8*12);
    irb(e4,F*8,12,
        enc4_3_branch1_0_weight,enc4_3_branch1_0_bias,
        enc4_3_branch1_1_weight,enc4_3_branch1_1_bias,
        enc4_3_branch2_0_weight,enc4_3_branch2_0_bias,
        enc4_3_branch2_1_weight,enc4_3_branch2_1_bias,
        enc4_3_branch3_0_weight,enc4_3_branch3_0_bias,
        enc4_3_branch3_1_weight,enc4_3_branch3_1_bias,
        enc4_3_branch4_weight,enc4_3_branch4_bias,
        enc4_3_bn_weight,enc4_3_bn_bias,
        enc4_3_bn_running_mean,enc4_3_bn_running_var,
        enc4_3_residual_proj_weight,enc4_3_residual_proj_bias,
        sm,so,sr);
    memcpy(e4,so,F*8*12*sizeof(float));


    convT(e4,F*8,12,dec1_0_weight,F*4,dec1_0_bias,d1u,24);
    for(int c=0;c<F*4;c++) d1u[c*25+24]=d1u[c*24+23];
    batch_norm(d1u,F*4,25,dec1_1_weight,dec1_1_bias,
               dec1_1_running_mean,dec1_1_running_var);
    leaky_relu(d1u,F*4*25);
    cat(d1u,F*4,e3,F*4,25,d1c);
    irb(d1c,F*8,25,
        dec1_inc_branch1_0_weight,dec1_inc_branch1_0_bias,
        dec1_inc_branch1_1_weight,dec1_inc_branch1_1_bias,
        dec1_inc_branch2_0_weight,dec1_inc_branch2_0_bias,
        dec1_inc_branch2_1_weight,dec1_inc_branch2_1_bias,
        dec1_inc_branch3_0_weight,dec1_inc_branch3_0_bias,
        dec1_inc_branch3_1_weight,dec1_inc_branch3_1_bias,
        dec1_inc_branch4_weight,dec1_inc_branch4_bias,
        dec1_inc_bn_weight,dec1_inc_bn_bias,
        dec1_inc_bn_running_mean,dec1_inc_bn_running_var,
        dec1_inc_residual_proj_weight,dec1_inc_residual_proj_bias,
        sm,d1o,sr);


    convT(d1o,F*8,25,dec2_0_weight,F*2,dec2_0_bias,d2u,50);
    batch_norm(d2u,F*2,50,dec2_1_weight,dec2_1_bias,
               dec2_1_running_mean,dec2_1_running_var);
    leaky_relu(d2u,F*2*50);
    cat(d2u,F*2,e2,F*2,50,d2c);
    irb(d2c,F*4,50,
        dec2_inc_branch1_0_weight,dec2_inc_branch1_0_bias,
        dec2_inc_branch1_1_weight,dec2_inc_branch1_1_bias,
        dec2_inc_branch2_0_weight,dec2_inc_branch2_0_bias,
        dec2_inc_branch2_1_weight,dec2_inc_branch2_1_bias,
        dec2_inc_branch3_0_weight,dec2_inc_branch3_0_bias,
        dec2_inc_branch3_1_weight,dec2_inc_branch3_1_bias,
        dec2_inc_branch4_weight,dec2_inc_branch4_bias,
        dec2_inc_bn_weight,dec2_inc_bn_bias,
        dec2_inc_bn_running_mean,dec2_inc_bn_running_var,
        dec2_inc_residual_proj_weight,dec2_inc_residual_proj_bias,
        sm,d2o,sr);


    convT(d2o,F*4,50,dec3_0_weight,F,dec3_0_bias,d3u,100);
    batch_norm(d3u,F,100,dec3_1_weight,dec3_1_bias,
               dec3_1_running_mean,dec3_1_running_var);
    leaky_relu(d3u,F*100);
    cat(d3u,F,e1,F,100,d3c);
    irb(d3c,F*2,100,
        dec3_inc_branch1_0_weight,dec3_inc_branch1_0_bias,
        dec3_inc_branch1_1_weight,dec3_inc_branch1_1_bias,
        dec3_inc_branch2_0_weight,dec3_inc_branch2_0_bias,
        dec3_inc_branch2_1_weight,dec3_inc_branch2_1_bias,
        dec3_inc_branch3_0_weight,dec3_inc_branch3_0_bias,
        dec3_inc_branch3_1_weight,dec3_inc_branch3_1_bias,
        dec3_inc_branch4_weight,dec3_inc_branch4_bias,
        dec3_inc_bn_weight,dec3_inc_bn_bias,
        dec3_inc_bn_running_mean,dec3_inc_bn_running_var,
        dec3_inc_residual_proj_weight,dec3_inc_residual_proj_bias,
        sm,d3o,sr);


    convT(d3o,F*2,100,dec4_0_weight,F,dec4_0_bias,d4o,200);
    batch_norm(d4o,F,200,dec4_1_weight,dec4_1_bias,
               dec4_1_running_mean,dec4_1_running_var);
    leaky_relu(d4o,F*200);
    pw(d4o,F,200,out_weight,1,out_bias,ob);
    sigmoid_inplace(ob,200);


    // Normalize window output to [0,1]
    float mn=ob[0],mx=ob[0];
    for(int i=1;i<WINDOW_SIZE;i++){
        if(ob[i]<mn) mn=ob[i];
        if(ob[i]>mx) mx=ob[i];
    }
    float rng=mx-mn; if(rng<1e-4f) rng=1.0f;


    // Return center sample — corresponds to the sample
    // that is WINDOW_SIZE/2 steps old in the delay buffer
    return (ob[WINDOW_SIZE/2] - mn) / rng;
}


// =========================================================
// Hardware init
// =========================================================
void app_init_hw(void) {
    printf("%s\n", APP_DESCRIPTION);
    gpio_init(LED_PIN);  gpio_set_dir(LED_PIN,  GPIO_OUT);
    gpio_init(QRS_LED);  gpio_set_dir(QRS_LED,  GPIO_OUT);
    gpio_init(FLAG_PIN); gpio_set_dir(FLAG_PIN, GPIO_OUT);
    MCP4728_init(i2c1,14,15,400*1000,0x60);
    adc_init(); adc_gpio_init(ADC_GPIO); adc_select_input(ADC_CH);
    SevSeg_init(spi1,1,10,11,12,100000);
    SevSeg_setBrightness(80);
    Disp_Str("RPnt");
    serial_start_boot_banner(APP_DESCRIPTION, NAME);


    // Fill delay buffer with mid-rail
    for (int i = 0; i < DELAY_LEN; i++)
        delay_buf[i] = 0.5f;
    delay_pos = 0;


    // Fill inference window with zeros
    memset(win, 0, sizeof(win));
    win_count = 0;


    add_repeating_timer_us(
        -SAMPLE_PERIOD_US, samp_timer_cb, NULL, &samp_timer
    );
}


// =========================================================
// Main loop
// =========================================================
void app_main(void) {


    // Current interpolated DT value output to DAC
    float dt_current = 0.5f;
    // Target DT value from last inference
    float dt_target  = 0.5f;
    // Previous DT value for interpolation
    float dt_prev    = 0.5f;


    int dbg_count = 0;


    while (true) {
        if (!sample_flag) {
            tight_loop_contents();
            continue;
        }
        sample_flag = false;


        // Read ADC
        uint16_t raw_adc = adc_read();
        float new_sample  = (float)raw_adc / 4095.0f;


        // QRS pulse countdown
        if (qrs_pulse_counter > 0) {
            qrs_pulse_counter--;
            if (qrs_pulse_counter == 0) {
                gpio_put(QRS_LED, 0);
                MCP4728_write2(0.0f);
            }
        }


        // ---- Delay buffer ----
        // Read oldest sample, write new sample, advance
        float dac_ecg     = delay_buf[delay_pos];
        delay_buf[delay_pos] = new_sample;
        delay_pos = (delay_pos + 1) % DELAY_LEN;
        MCP4728_write1(dac_ecg);


        // ---- Slide inference window by 1 ----
        memmove(win, win + 1, (WINDOW_SIZE - 1) * sizeof(float));
        win[WINDOW_SIZE - 1] = normalize_for_model(new_sample);
        win_count++;


        // ---- Run inference every INFER_STRIDE samples ----
        if (win_count >= INFER_STRIDE) {
            win_count = 0;
            dt_prev   = dt_target;
            dt_target = run_inference_center();
        }


        // ---- Interpolate DT between inference updates ----
        // alpha goes 0->1 over INFER_STRIDE samples
        float alpha  = 1.0f - (float)win_count / (float)INFER_STRIDE;
        dt_current   = alpha * dt_prev + (1.0f - alpha) * dt_target;


        // ---- Smooth and output DT ----
        float smooth = dt_smooth_update(dt_current);
        MCP4728_write3(smooth * 0.8f + 0.1f);


        // ---- Valley detector ----
        if (vd_update(smooth)) {
            gpio_put(QRS_LED, 1);
            MCP4728_write2(1.0f);
            qrs_pulse_counter = QRS_PULSE_DURATION;


            shift_right(x_heart_rate, HEART_RATE_N);
            x_heart_rate[0] = time1;
            rr_avg = 0.0f;
            for (int k = 0; k < HEART_RATE_N; k++)
                rr_avg += x_heart_rate[k];
            rr_avg /= (float)HEART_RATE_N;
            if (rr_avg > 0.33f && rr_avg < 1.5f) {
                hr = 60.0f / rr_avg;
                printf("HR: %3.1f bpm\r\n", hr);
                char buf[5];
                snprintf(buf, sizeof(buf), "%3.0f", hr);
                Disp_Str(buf);
            }
            time1 = 0.0f;
        }


        time1 += SAMPLE_PERIOD_S;


        if (++dbg_count >= 400) {
            printf("spk:%.3f npk:%.3f thr:%.3f HR:%.1f\r\n",
                   vd_spk, vd_npk, vd_thresh1, hr);
            dbg_count = 0;
        }


        gpio_put(FLAG_PIN, 0);
    }
}


#endif

// ECG Machine Learning