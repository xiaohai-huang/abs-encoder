/**
 * @file mt6701.h
 * @brief MT6701 (MagnTek) SSI protocol layer.
 *
 * The sensor speaks a 24-bit frame (3 bytes, MSB first, SPI mode 1):
 *   bits 23..10  14-bit angle (0..16383 = 0..360 deg)
 *   bit  9       track loss
 *   bit  8       push-magnet (Z) state
 *   bits 7..6    field status (0 = normal)
 *   bits 5..0    CRC-6 (see mt6701_crc6_compute)
 *
 * Only this file and the simulated chip (sim/mt6701_slave_sim.c) know the
 * frame format; everything else consumes a resolved 14-bit angle.
 */
#ifndef APP_MT6701_H
#define APP_MT6701_H

#include <stdbool.h>
#include <stdint.h>

#define MT6701_ANGLE_MAX 16383u

/* The reference drivers ignore the CRC; enable only after verifying the
 * polynomial in mt6701.c against the MT6701 datasheet (SSI section). */
#ifndef MT6701_CRC6_ENABLED
#define MT6701_CRC6_ENABLED 0
#endif

typedef struct
{
    uint16_t angle; /* 0..16383 */
    bool     button; /* push-magnet state */
} mt6701_sample_t;

/**
 * Read one angle frame with status validation and retries.
 * @return 0 on success, -1 if the chip reports a fault (track loss / bad
 *         field status), -2 if no valid frame arrived within the retries.
 */
int mt6701_read_sample(mt6701_sample_t *out);

/** Raw 24-bit frame access without validation (diagnostics, tests). */
int mt6701_read_frame(uint8_t frame[3]);

/**
 * CRC-6 over the 18 data bits (angle + status), returned in the low 6 bits.
 * Polynomial X^6+X^5+X^3+X^2+X+1 (0x5B), init 0, MSB first.
 * NOTE: only used when MT6701_CRC6_ENABLED is defined; verify the
 * polynomial against the MT6701 datasheet (Rev 1.8, SSI section) first.
 */
uint8_t mt6701_crc6_compute(const uint8_t frame[3]);

#endif /* APP_MT6701_H */
