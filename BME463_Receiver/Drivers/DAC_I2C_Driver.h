/**
 * @file DAC_I2C_Driver.h
 * @author Zerind Spore
 * @brief MCP4728 I2C DAC Driver for Raspberry Pi Pico 2
 * @version 1.1
 * @date 01/17/2026
 * 
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Defaults
#define MCP4728_DEFAULT_I2C_BAUD_HZ   (400u * 1000u) // 400 kHz
#define MCP4728_DEFAULT_ADDR          (0x60u)        // 7-bit base addr when A2..A0 = 000 :contentReference[oaicite:1]{index=1}

// Channel IDs
typedef enum {
    MCP4728_CH_A = 0,
    MCP4728_CH_B = 1,
    MCP4728_CH_C = 2,
    MCP4728_CH_D = 3,
} mcp4728_channel_t;

// VREF selection (per-channel)
typedef enum {
    MCP4728_VREF_VDD      = 0, // VREF = VDD
    MCP4728_VREF_INTERNAL = 1, // VREF = internal 2.048V :contentReference[oaicite:2]{index=2}
} mcp4728_vref_t;

// Gain selection (per-channel)
typedef enum {
    MCP4728_GAIN_1X = 0,
    MCP4728_GAIN_2X = 1,
} mcp4728_gain_t;

// Power-down modes (PD1:PD0) 
typedef enum {
    MCP4728_PD_NORMAL = 0, // 00 = normal
    MCP4728_PD_1K     = 1, // 01 = 1k to GND (typ)
    MCP4728_PD_100K   = 2, // 10 = 100k to GND (typ)
    MCP4728_PD_500K   = 3, // 11 = 500k to GND (typ)
} mcp4728_pd_t;

// Initialize I2C pins + store bus/address.
// Pass addr7 = 0 to use MCP4728_DEFAULT_ADDR.
// Pass baud_hz = 0 to use MCP4728_DEFAULT_I2C_BAUD_HZ.
void MCP4728_init(i2c_inst_t *i2c,
                  uint sda_pin, uint scl_pin,
                  uint32_t baud_hz,
                  uint8_t addr7);

// Raw 12-bit write to any channel (volatile DAC input register).
// udac = false -> update VOUT immediately after the last ACK 
// udac = true  -> do not update until LDAC low (Not used here, used if LDAC pin is connected)
bool MCP4728_write12(mcp4728_channel_t ch,
                     uint16_t code12,
                     mcp4728_vref_t vref,
                     mcp4728_gain_t gain,
                     mcp4728_pd_t pd,
                     bool udac);

// write normalized [0..1] to any channel, using VDD ref, 1x gain, normal, udac=false
bool MCP4728_write01(mcp4728_channel_t ch, float v01);


static inline bool MCP4728_writeA01(float v01) { return MCP4728_write01(MCP4728_CH_A, v01); }
static inline bool MCP4728_writeB01(float v01) { return MCP4728_write01(MCP4728_CH_B, v01); }
static inline bool MCP4728_writeC01(float v01) { return MCP4728_write01(MCP4728_CH_C, v01); }
static inline bool MCP4728_writeD01(float v01) { return MCP4728_write01(MCP4728_CH_D, v01); }

// Channel Specific write functions
static inline bool MCP4728_write1(float v01) { return MCP4728_writeD01(v01); } // Writes to RA
static inline bool MCP4728_write2(float v01) { return MCP4728_writeC01(v01); } // Writes to RB
static inline bool MCP4728_write3(float v01) { return MCP4728_writeB01(v01); } // Writes to RC
static inline bool MCP4728_write4(float v01) { return MCP4728_writeA01(v01); } // Writes to RD 

#ifdef __cplusplus
}
#endif
