/**
 * @file SevSeg_Driver.h
 * @author Zerind Spore
 * @brief Prototype file for driver for on board Seven Segment Display
 * @version 1.0
 * @date 10/29/2025
 * 
*/
#include "SevSeg_Driver.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <math.h>
#include <string.h>

// SparkFun command bytes
#define CMD_CLEAR_DISPLAY  0x76
#define CMD_DECIMAL_CTRL   0x77
#define CMD_BRIGHTNESS     0x7A

static spi_inst_t *g_spi = NULL;
static uint g_cs_pin = 0;

static inline void cs_select(void)   { gpio_put(g_cs_pin, 0); }
static inline void cs_deselect(void) { gpio_put(g_cs_pin, 1); }

// One full transaction (CS low once)
static void sevseg_tx(const uint8_t *buf, size_t n)
{
    if (!g_spi || !buf || n == 0) return;
    cs_select();
    spi_write_blocking(g_spi, buf, n);
    cs_deselect();
}

void SevSeg_init(spi_inst_t *spi,
                 uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin,
                 uint32_t spi_baud_hz)
{
    g_spi = spi;
    g_cs_pin = cs_pin;

    spi_init(g_spi, spi_baud_hz ? spi_baud_hz : SEVSEG_DEFAULT_SPI_BAUD_HZ);
    spi_set_format(g_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(sck_pin,  GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);

    gpio_init(g_cs_pin);
    gpio_set_dir(g_cs_pin, GPIO_OUT);
    gpio_put(g_cs_pin, 1);
    sleep_ms(10);

    SevSeg_clear();
    SevSeg_setDecimals(0);
}

void SevSeg_clear(void)
{
    uint8_t b = CMD_CLEAR_DISPLAY;
    sevseg_tx(&b, 1);
}

void SevSeg_setBrightness(uint8_t percent_0_100)
{
    if (percent_0_100 > 100) percent_0_100 = 100;
    uint8_t buf[2] = { CMD_BRIGHTNESS, percent_0_100 };
    sevseg_tx(buf, 2);
}

void SevSeg_setDecimals(uint8_t mask_0_63)
{
    uint8_t buf[2] = { CMD_DECIMAL_CTRL, (uint8_t)(mask_0_63 & 0x3Fu) };
    sevseg_tx(buf, 2);
}

// Helper: write 4 ASCII characters + decimals in ONE transaction
static void write4_with_decimals(const char d[4], uint8_t decmask)
{
    uint8_t pkt[1 + 2 + 4];
    pkt[0] = CMD_CLEAR_DISPLAY;
    pkt[1] = CMD_DECIMAL_CTRL;
    pkt[2] = (uint8_t)(decmask & 0x3Fu);
    pkt[3] = (uint8_t)d[0];
    pkt[4] = (uint8_t)d[1];
    pkt[5] = (uint8_t)d[2];
    pkt[6] = (uint8_t)d[3];
    sevseg_tx(pkt, sizeof(pkt));
}

void Disp_Str(const char *s)
{
    char d[4] = { ' ', ' ', ' ', ' ' };
    if (s) {
        for (int i = 0; i < 4 && s[i]; i++) {
            char c = s[i];
            if ((unsigned char)c < 32 || (unsigned char)c > 126) c = ' ';
            d[i] = c;
        }
    }
    write4_with_decimals(d, 0);
}

static uint8_t decmask_for_decimals(int decimals)
{
    // decimals = digits after decimal point:
    // 1 -> ###.# (decimal after digit 3)
    // 2 -> ##.## (decimal after digit 2)
    // 3 -> #.### (decimal after digit 1)
    switch (decimals) {
        case 1: return SEVSEG_DEC_D3;
        case 2: return SEVSEG_DEC_D2;
        case 3: return SEVSEG_DEC_D1;
        default: return 0;
    }
}

// Format an integer 0..9999 into 4 ASCII digits (with optional leading spaces)
static void u16_to_4digits(uint16_t v, char out4[4], bool leading_spaces)
{
    out4[0] = (char)('0' + (v / 1000u) % 10u);
    out4[1] = (char)('0' + (v / 100u)  % 10u);
    out4[2] = (char)('0' + (v / 10u)   % 10u);
    out4[3] = (char)('0' + (v / 1u)    % 10u);

    if (leading_spaces) {
        // Replace leading zeros with spaces, keep at least last digit
        if (out4[0] == '0') { out4[0] = ' '; }
        if (out4[0] == ' ' && out4[1] == '0') { out4[1] = ' '; }
        if (out4[0] == ' ' && out4[1] == ' ' && out4[2] == '0') { out4[2] = ' '; }
    }
}

bool Disp_Int(uint16_t v)
{
    if (v > 9999u) return false;

    char d[4];
    u16_to_4digits(v, d, true);
    write4_with_decimals(d, 0);
    return true;
}

bool Disp_Fixed(float v, int decimals)
{
    // Fixed decimal places, fits in 4 digits total
    if (decimals < 0) decimals = 0;
    if (decimals > 3) decimals = 3;

    if (!(v >= 0.0f)) return false; // reject NaN/neg

    // Clamp to what fits
    float vmax;
    if (decimals == 0) vmax = 9999.0f;
    else if (decimals == 1) vmax = 999.9f;
    else if (decimals == 2) vmax = 99.99f;
    else vmax = 9.999f;

    if (v > vmax) v = vmax;

    float scale = 1.0f;
    for (int i = 0; i < decimals; i++) scale *= 10.0f;

    uint32_t code = (uint32_t)lroundf(v * scale);
    if (code > 9999u) code = 9999u;

    char d[4];
    u16_to_4digits((uint16_t)code, d, false); // don't blank leading zeros when decimals used
    write4_with_decimals(d, decmask_for_decimals(decimals));
    return true;
}

bool Disp_Out(float v)
{
    // Your desired ranges: 0.001 .. 999.9 (but we'll handle 0 too)
    if (!(v >= 0.0f)) return false;

    if (v > 999.9f) v = 999.9f;
    if (v > 0.0f && v < 0.001f) v = 0.001f;

    int decimals;
    if (v < 1.0f) decimals = 3;      // 0.abc
    else if (v < 10.0f) decimals = 2; // a.bc
    else decimals = 1;               // ab.c or abc.d

    return Disp_Fixed(v, decimals);
}

bool Disp_Time(uint8_t mm, uint8_t ss)
{
    if (mm > 99 || ss > 99) return false;

    char d[4];
    d[0] = (char)('0' + (mm / 10));
    d[1] = (char)('0' + (mm % 10));
    d[2] = (char)('0' + (ss / 10));
    d[3] = (char)('0' + (ss % 10));

    write4_with_decimals(d, SEVSEG_DEC_COLON);
    return true;
}
