/**
 * @file i2c_pos.h
 * @brief I2C slave host port: the decoded absolute position, as a small
 *        register map another device on the I2C bus can poll at any time.
 *
 * Pure logic (no HAL dependency, like gear_decode.c) - the transport lives
 * behind it: hal_stm32.c runs the STM32 slave (own address, listen-IT,
 * EV/ER callbacks), and the firmware main loop pushes one snapshot per
 * sample via i2c_pos_update().  Wire protocol and register table:
 * docs/i2c.md.
 */
#ifndef APP_I2C_POS_H
#define APP_I2C_POS_H

#include <stdbool.h>
#include <stdint.h>

#include "gear_decode.h" /* gear_pos_t */

/** 7-bit slave address of the encoder on the host bus.  Applied to OAR1 by
 *  the backend at init (hal_stm32.c) - the CubeMX-generated OwnAddress1=0
 *  in i2c.c is never used. */
#define I2C_POS_ADDR 0x50u

/* Register map (read-only, little-endian on the wire):
 *   0x00..0x03  absolute position count, uint32 LE, 0 ..
 *               121,716,735  (turns * I2C_POS_COUNTS_PER_TURN + angle;
 *               the turn field 0..7428 is 13 bits and the 14-bit fine
 *               angle makes the combined value a 27-bit integer, so it
 *               fits one uint32 and counts continuously across the
 *               turn boundary)
 *   0x04        status: bit0 = 1 when the sample passed the slew guard */
#define I2C_POS_REG_STATUS   0x04u
#define I2C_POS_REG_COUNT    5u

/** Fine-angle steps per whole turn (MT6701_ANGLE_MAX + 1).  The combined
 *  count the map exports is turns * this + angle. */
#define I2C_POS_COUNTS_PER_TURN (MT6701_ANGLE_MAX + 1u)

#define I2C_POS_STATUS_VALID 0x01u /* p->valid: slew guard accepted */

/** Start from an empty snapshot (all zeros, cursor at register 0). */
void i2c_pos_init(void);

/**
 * Refresh the exported snapshot from the latest decode.  Called once per
 * sample from the main loop.  The position is packed into a single 32-bit
 * word before publishing, so a concurrent reader (address-match ISR) sees
 * either the old or the new snapshot, never a mix.
 */
void i2c_pos_update(const gear_pos_t *p);

/** Set the register pointer (the byte a host writes before reading); values
 *  past the map clamp to it so reads stay zero-filled. */
void i2c_pos_select(uint8_t reg);

/**
 * Copy bytes to the reader starting at the register pointer, advancing it
 * (auto-increment, EEPROM style); past the map every byte reads 0x00.
 * Reads the packed snapshot a single time per call.
 * @param dst  destination (host TX buffer)
 * @param max  bytes requested / room available
 * @return     bytes copied (== max unless the snapshot is smaller)
 */
uint8_t i2c_pos_read(uint8_t *dst, uint8_t max);

#endif /* APP_I2C_POS_H */