/**
 * @file DAC_I2C_Driver.c
 * @author Zerind Spore
 * @brief MCP4728 I2C DAC Driver for Raspberry Pi Pico 2
 * @version 1.1
 * @date 01/17/2026
 * 
*/
#include "hardware/i2c.h"

#include "DAC_I2C_Driver.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Internal state
static i2c_inst_t *g_i2c = NULL;
static uint8_t     g_addr7 = MCP4728_DEFAULT_ADDR;

// MCP4728 Multi-Write command (volatile DAC input register)
// 2nd byte bitfield (MSB->LSB):
// [C2 C1 C0 W1 W0 DAC1 DAC0 UDAC]
// For Multi-Write: C2 C1 C0 W1 W0 = 0 1 0 0 0  -> base 0b01000xxx = 0x40 :contentReference[oaicite:3]{index=3}
#define MCP4728_CMD_MULTIWRITE_BASE   (0x40u)

void MCP4728_init(i2c_inst_t *i2c,
                  uint sda_pin, uint scl_pin,
                  uint32_t baud_hz,
                  uint8_t addr7)
{
    g_i2c   = i2c;
    g_addr7 = (addr7 == 0) ? MCP4728_DEFAULT_ADDR : (addr7 & 0x7Fu);

    // I2C setup
    i2c_init(g_i2c, baud_hz ? baud_hz : MCP4728_DEFAULT_I2C_BAUD_HZ);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // set all channels to 0V (normalized) on init
    MCP4728_writeA01(0.0f);
    MCP4728_writeB01(0.0f);
    MCP4728_writeC01(0.0f);
    MCP4728_writeD01(0.0f);
}

bool MCP4728_write12(mcp4728_channel_t ch,
                     uint16_t code12,
                     mcp4728_vref_t vref,
                     mcp4728_gain_t gain,
                     mcp4728_pd_t pd,
                     bool udac)
{
    if (!g_i2c) return false;
    if ((unsigned)ch > 3u) return false;

    code12 &= 0x0FFFu;

    // Multi-write packet = 3 bytes after address:
    // 2nd byte: command + channel + UDAC
    // 3rd byte: VREF PD1 PD0 Gx D11..D8
    // 4th byte: D7..D0
    // From Figure 5-8 in data sheet (Multi-Write) byte layout
    uint8_t b2 = (uint8_t)(MCP4728_CMD_MULTIWRITE_BASE |
                           (((uint8_t)ch & 0x03u) << 1) |
                           (udac ? 1u : 0u));

    uint8_t b3 = (uint8_t)(((uint8_t)vref & 0x01u) << 7) |
                 (uint8_t)(((uint8_t)pd   & 0x03u) << 5) |
                 (uint8_t)(((uint8_t)gain & 0x01u) << 4) |
                 (uint8_t)((code12 >> 8) & 0x0Fu);

    uint8_t b4 = (uint8_t)(code12 & 0xFFu);

    uint8_t buf[3] = { b2, b3, b4 };

    // Pico SDK returns bytes written or PICO_ERROR_GENERIC (<0)
    int rc = i2c_write_blocking(g_i2c, g_addr7, buf, (size_t)3, false);
    return (rc == 3);
}

bool MCP4728_write01(mcp4728_channel_t ch, float v01)
{
    if (v01 < 0.0f) v01 = 0.0f;
    if (v01 > 1.0f) v01 = 1.0f;

    uint16_t code12 = (uint16_t)(v01 * 4095.0f + 0.5f);

    return MCP4728_write12(ch,
                           code12,
                           MCP4728_VREF_VDD,
                           MCP4728_GAIN_1X,
                           MCP4728_PD_NORMAL,
                           false);
}
