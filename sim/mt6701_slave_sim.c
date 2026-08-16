/**
 * @file mt6701_slave_sim.c
 * @brief Simulated MT6701 SSI slave.  Mirrors the frame construction from
 *        the datasheet: 24 bits MSB first, angle in bits 23..10, status
 *        bits 9..6, CRC-6 in bits 5..0.  Uses the same CRC function as
 *        the firmware driver (App/mt6701.c).
 */
#include "mt6701_slave_sim.h"

#include "mt6701.h"

static uint16_t s_angle;
static bool s_button;
static bool s_track_loss;
static bool s_crc_broken;
static bool s_stuck;
static uint8_t s_field_status;

static uint32_t s_frame; /* 24-bit frame, MSB first */
static int s_pos;        /* 0..2, -1 while CS is high */

void mt6701_slave_init(void)
{
    s_angle = 0u;
    s_button = false;
    s_track_loss = false;
    s_crc_broken = false;
    s_stuck = false;
    s_field_status = 0u;
    s_frame = 0u;
    s_pos = -1;
}

static void build_frame(void)
{
    uint32_t frame = (uint32_t)(s_angle & MT6701_ANGLE_MAX) << 10;
    if (s_button)
    {
        frame |= 1u << 8; /* frame bit 8 = Z / push state */
    }
    if (s_track_loss)
    {
        frame |= 1u << 9; /* frame bit 9 = track loss */
    }
    frame |= (uint32_t)(s_field_status & 0x03u) << 6;

    uint8_t bytes[3] = {(uint8_t)(frame >> 16), (uint8_t)(frame >> 8), (uint8_t)frame};
    uint8_t crc = mt6701_crc6_compute(bytes);
    if (s_crc_broken)
    {
        crc ^= 0x01u;
    }

    s_frame = frame | (uint32_t)(crc & 0x3Fu);
}

void mt6701_slave_cs(bool asserted)
{
    if (asserted)
    {
        build_frame();
        s_pos = 0;
    }
    else
    {
        s_pos = -1;
    }
}

uint8_t mt6701_slave_transfer(uint8_t tx)
{
    (void)tx; /* SSI is unidirectional; MOSI is ignored by the chip */

    if (s_stuck)
    {
        return 0xFFu;
    }

    uint8_t rx = 0u;
    if (s_pos >= 0 && s_pos < 3)
    {
        rx = (uint8_t)(s_frame >> (16 - 8 * s_pos));
    }
    s_pos++;
    return rx;
}

void mt6701_slave_set_angle(uint16_t angle)
{
    s_angle = angle & MT6701_ANGLE_MAX;
}

void mt6701_slave_set_button(bool on)
{
    s_button = on;
}

void mt6701_slave_set_track_loss(bool on)
{
    s_track_loss = on;
}

void mt6701_slave_set_field_status(uint8_t status)
{
    s_field_status = status & 0x03u;
}

void mt6701_slave_set_crc_broken(bool on)
{
    s_crc_broken = on;
}

void mt6701_slave_set_stuck(bool on)
{
    s_stuck = on;
}
