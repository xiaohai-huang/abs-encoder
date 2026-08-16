/**
 * @file mt6701.c
 * @brief MT6701 SSI frame exchange and validation (platform independent).
 */
#include "mt6701.h"

#include "hal.h"

#define MT6701_RETRIES        3u
#define MT6701_RETRY_DELAY_US 10u

/* CRC-6 polynomial X^6+X+1, MSB first, over the 18 data bits
 * (D[13:0] + Mg[3:0]) per datasheet Rev 1.8, SSI section (7.8.2).
 * Initial value 0, no final XOR (not specified in the datasheet). */
#define MT6701_CRC6_POLY 0x43u

static uint8_t crc6_update(uint8_t crc, uint8_t byte, int bits)
{
    for (int i = bits - 1; i >= 0; i--)
    {
        uint8_t mix = (uint8_t)((crc >> 5) ^ ((byte >> i) & 1u));
        crc = (uint8_t)((crc << 1) & 0x3Fu);
        if (mix != 0u)
        {
            crc = (uint8_t)((crc ^ MT6701_CRC6_POLY) & 0x3Fu);
        }
    }
    return crc;
}

uint8_t mt6701_crc6_compute(const uint8_t frame[3])
{
    uint8_t crc = crc6_update(0u, frame[0], 8);
    crc = crc6_update(crc, frame[1], 8);
    return crc6_update(crc, (uint8_t)(frame[2] >> 6), 2);
}

int mt6701_read_frame(uint8_t frame[3])
{
    /* SSI is unidirectional: MOSI is don't-care while the chip shifts out. */
    app_hal.spi_cs(true); /* assert (pull low): active-low chip select */
    for (uint32_t i = 0; i < 3u; i++)
    {
        frame[i] = app_hal.spi_transfer(0u);
    }
    app_hal.spi_cs(false);
    return 0;
}

int mt6701_read_sample(mt6701_sample_t *out)
{
    bool saw_fault = false;

    for (uint32_t i = 0; i < MT6701_RETRIES; i++)
    {
        uint8_t frame[3];
        mt6701_read_frame(frame);

#if MT6701_CRC6_ENABLED
        if (mt6701_crc6_compute(frame) != (frame[2] & 0x3Fu))
        {
            app_hal.delay_us(MT6701_RETRY_DELAY_US);
            continue;
        }
#endif

        uint16_t angle = (uint16_t)(((uint16_t)frame[0] << 6) | ((uint16_t)frame[1] >> 2));
        uint8_t status = (uint8_t)(((frame[1] & 0x03u) << 2) | (frame[2] >> 6));

        /* status: bit3 = track loss, bit2 = button, bits1..0 = field status. */
        if ((status & 0x08u) != 0u || (status & 0x03u) != 0u)
        {
            saw_fault = true;
            app_hal.delay_us(MT6701_RETRY_DELAY_US);
            continue;
        }

        out->angle = angle;
        out->button = (status & 0x04u) != 0u;
        return 0;
    }

    return saw_fault ? -1 : -2;
}
