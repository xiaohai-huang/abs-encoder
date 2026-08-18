/**
 * @file mt6701_slave_sim.c
 * @brief Simulated MT6701 SSI slaves.  Mirror the frame construction from
 *        the datasheet: 24 bits MSB first, angle in bits 23..10, status
 *        bits 9..6, CRC-6 in bits 5..0.  Use the same CRC function as the
 *        firmware driver (App/mt6701.c).  One chip per encoder index.
 */
#include "mt6701_slave_sim.h"

#include "mt6701.h"

typedef struct
{
    uint16_t angle;
    bool     button;
    bool     track_loss;
    bool     crc_broken;
    bool     stuck;
    uint8_t  field_status;
    uint32_t frame; /* 24-bit frame, MSB first */
    int      pos;   /* 0..2, -1 while CS is high */
} chip_t;

static chip_t s_chips[MT6701_ENC_COUNT];

void mt6701_slave_init(void)
{
    for (uint8_t i = 0; i < MT6701_ENC_COUNT; i++)
    {
        s_chips[i].angle = 0u;
        s_chips[i].button = false;
        s_chips[i].track_loss = false;
        s_chips[i].crc_broken = false;
        s_chips[i].stuck = false;
        s_chips[i].field_status = 0u;
        s_chips[i].frame = 0u;
        s_chips[i].pos = -1;
    }
}

static void build_frame(chip_t *chip)
{
    uint32_t frame = (uint32_t)(chip->angle & MT6701_ANGLE_MAX) << 10;
    if (chip->button)
    {
        frame |= 1u << 8; /* frame bit 8 = Z / push state */
    }
    if (chip->track_loss)
    {
        frame |= 1u << 9; /* frame bit 9 = track loss */
    }
    frame |= (uint32_t)(chip->field_status & 0x03u) << 6;

    uint8_t bytes[3] = {(uint8_t)(frame >> 16), (uint8_t)(frame >> 8), (uint8_t)frame};
    uint8_t crc = mt6701_crc6_compute(bytes);
    if (chip->crc_broken)
    {
        crc ^= 0x01u;
    }

    chip->frame = frame | (uint32_t)(crc & 0x3Fu);
}

void mt6701_slave_cs(uint8_t enc, bool asserted)
{
    if (enc >= MT6701_ENC_COUNT)
    {
        return;
    }

    chip_t *chip = &s_chips[enc];
    if (asserted)
    {
        build_frame(chip);
        chip->pos = 0;
    }
    else
    {
        chip->pos = -1;
    }
}

uint8_t mt6701_slave_transfer(uint8_t enc, uint8_t tx)
{
    (void)tx; /* SSI is unidirectional; MOSI is ignored by the chip */

    if (enc >= MT6701_ENC_COUNT)
    {
        return 0xFFu;
    }

    chip_t *chip = &s_chips[enc];
    if (chip->stuck)
    {
        return 0xFFu;
    }

    uint8_t rx = 0u;
    if (chip->pos >= 0 && chip->pos < 3)
    {
        rx = (uint8_t)(chip->frame >> (16 - 8 * chip->pos));
    }
    chip->pos++;
    return rx;
}

void mt6701_slave_set_angle(uint8_t enc, uint16_t angle)
{
    if (enc < MT6701_ENC_COUNT)
    {
        s_chips[enc].angle = angle & MT6701_ANGLE_MAX;
    }
}

void mt6701_slave_set_button(uint8_t enc, bool on)
{
    if (enc < MT6701_ENC_COUNT)
    {
        s_chips[enc].button = on;
    }
}

void mt6701_slave_set_track_loss(uint8_t enc, bool on)
{
    if (enc < MT6701_ENC_COUNT)
    {
        s_chips[enc].track_loss = on;
    }
}

void mt6701_slave_set_field_status(uint8_t enc, uint8_t status)
{
    if (enc < MT6701_ENC_COUNT)
    {
        s_chips[enc].field_status = status & 0x03u;
    }
}

void mt6701_slave_set_crc_broken(uint8_t enc, bool on)
{
    if (enc < MT6701_ENC_COUNT)
    {
        s_chips[enc].crc_broken = on;
    }
}

void mt6701_slave_set_stuck(uint8_t enc, bool on)
{
    if (enc < MT6701_ENC_COUNT)
    {
        s_chips[enc].stuck = on;
    }
}