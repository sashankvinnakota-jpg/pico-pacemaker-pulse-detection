/**
 * @file SevSeg_Driver.h
 * @author Zerind Spore
 * @brief Prototype file for driver for on board Seven Segment Display
 * @version 2.0
 * @date 10/29/2025
 * 
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SEVSEG_DEFAULT_SPI_BAUD_HZ (500u * 1000u) // 500 kHz proven-safe

// Decimal mask bits (SparkFun Serial 7-Seg):
// bit0 = Decimal0 (far-right, after digit 4)
// bit1 = Decimal1 (after digit 3)
// bit2 = Decimal2 (after digit 2)
// bit3 = Decimal3 (after digit 1)
// bit4 = Colon
// bit5 = Apostrophe
#define SEVSEG_DEC_D4     (1u << 0)
#define SEVSEG_DEC_D3     (1u << 1)
#define SEVSEG_DEC_D2     (1u << 2)
#define SEVSEG_DEC_D1     (1u << 3)
#define SEVSEG_DEC_COLON  (1u << 4)
#define SEVSEG_DEC_APOS   (1u << 5)

// Initialize SPI + pins + CS, then clear display
void SevSeg_init(spi_inst_t *spi,
                 uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin,
                 uint32_t spi_baud_hz);

// Core commands
void SevSeg_clear(void);
void SevSeg_setBrightness(uint8_t percent_0_100);
void SevSeg_setDecimals(uint8_t mask_0_63);

// High-level display functions
bool Disp_Int(uint16_t v);                  // 0..9999
bool Disp_Fixed(float v, int decimals);     // fixed decimals (0..3), auto clamp to fit
bool Disp_Out(float v);                     // 0.001..999.9 style auto formatting
void Disp_Str(const char *s);               // prints first 4 chars, pads with spaces
bool Disp_Time(uint8_t mm, uint8_t ss);     // "MM:SS" (00..99 each), colon on

#ifdef __cplusplus
}
#endif
