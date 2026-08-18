/**
 * @file mt6701.h
 * @brief MT6701 (MagnTek) SSI protocol layer.
 *
 * The sensor speaks a 24-bit frame (3 bytes, MSB first; the datasheet shows
 * CLK idling high, but mode 1 -- CPOL=0, CPHA=1 -- is what the working
 * reference drivers use and matches the CubeMX SPI1/SPI2 config):
 *   bits 23..10  14-bit angle (0..16383 = 0..360 deg)
 *   bit  9       track loss (Mg[3])
 *   bit  8       push-magnet (Mg[2])
 *   bits 7..6    field status (Mg[1:0], 0 = normal)
 *   bits 5..0    CRC-6 (see mt6701_crc6_compute)
 *
 * Every call is indexed by encoder role (encoder_role_t in
 * gear_config.h): ENC_SUN is the input shaft, ENC_GEAR_1..3 the driven
 * gears.  On this board ENC_GEAR_1/2 share SPI1 (CSN1/CSN2) and
 * ENC_SUN/ENC_GEAR_3 share SPI2 (CSN4/CSN3); the role maps to bus and CS
 * in the HAL implementations (s_enc table in hal_stm32.c).
 * Only this file and the simulated chips (sim/mt6701_slave_sim.c) know
 * the frame format; everything else consumes a resolved 14-bit angle.
 */
#ifndef APP_MT6701_H
#define APP_MT6701_H

#include <stdbool.h>
#include <stdint.h>

#include "gear_config.h" /* encoder_role_t: which wheel each MT6701 reads */

#define MT6701_ANGLE_MAX 16383u
#define MT6701_ENC_COUNT 4u /* encoders: 0,1 on SPI1; 2,3 on SPI2 */

/* CRC-6 (X^6+X+1) validation per datasheet Rev 1.8 SSI section;
 * set to 0 to accept frames without the CRC check. */
#ifndef MT6701_CRC6_ENABLED
#define MT6701_CRC6_ENABLED 1
#endif

typedef struct
{
    uint16_t angle; /* 0..16383 */
    bool     button; /* push-magnet state */
} mt6701_sample_t;

/**
 * Read one angle frame with status validation and retries.
 * @param enc encoder role (ENC_SUN, ENC_GEAR_1..3)
 * @return 0 on success, -1 if the chip reports a fault (track loss / bad
 *         field status), -2 if no valid frame arrived within the retries.
 */
int mt6701_read_sample(encoder_role_t enc, mt6701_sample_t *out);

/** Raw 24-bit frame access without validation (diagnostics, tests). */
int mt6701_read_frame(encoder_role_t enc, uint8_t frame[3]);

/**
 * CRC-6 over the 18 data bits (angle + status), returned in the low 6 bits.
 * Polynomial X^6+X+1 (0x43), initial value 0, no final XOR, MSB first --
 * per datasheet Rev 1.8, SSI section.  Used when MT6701_CRC6_ENABLED is 1.
 */
uint8_t mt6701_crc6_compute(const uint8_t frame[3]);

#endif /* APP_MT6701_H */