/**
 * @file i2c_pos.c
 * @brief I2C slave register map for the absolute position (see i2c_pos.h).
 *
 * The exported snapshot lives in one 32-bit word (turns | angle, both
 * little-endian on the wire) plus one status byte.  The main loop writes it
 * with a single aligned store and the transport copies it with a single
 * aligned load at address-match time, so a host read can never observe a
 * torn position even though the sample updates at 1 kHz.
 */
#include "i2c_pos.h"

static uint32_t s_frame; /* regs 0x00..0x03: turns | (angle << 16) */
static uint8_t s_status; /* reg 0x04 */
static uint8_t s_cursor; /* register pointer (host-writable), 0..REG_COUNT */

void i2c_pos_init(void)
{
    s_frame = 0u;
    s_status = 0u;
    s_cursor = 0u;
}

void i2c_pos_update(const gear_pos_t *p)
{
    s_frame = (uint32_t)p->turns | ((uint32_t)p->angle << 16u);
    s_status = p->valid ? I2C_POS_STATUS_VALID : 0u;
}

void i2c_pos_select(uint8_t reg)
{
    s_cursor = (reg < I2C_POS_REG_COUNT) ? reg : I2C_POS_REG_COUNT;
}

uint8_t i2c_pos_read(uint8_t *dst, uint8_t max)
{
    uint32_t frame = s_frame; /* one 32-bit load: consistent snapshot */
    uint8_t n = 0u;
    while (n < max)
    {
        if (s_cursor < I2C_POS_REG_COUNT)
        {
            uint8_t reg = s_cursor++;
            dst[n] = (reg == I2C_POS_REG_STATUS)
                         ? s_status
                         : (uint8_t)(frame >> (8u * reg));
        }
        else
        {
            dst[n] = 0u; /* past the map: zero-fill, never wraps */
        }
        n++;
    }
    return n;
}