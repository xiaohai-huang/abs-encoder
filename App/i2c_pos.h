/**
 * @file i2c_pos.h
 * @brief I2C slave host port: the decoded absolute position, as a small
 *        register map another device on the I2C bus can poll at any time.
 *
 * Pure logic (no HAL dependency, like gear_decode) - the transport lives
 * behind it: hal_stm32.cpp runs the STM32 slave (own address, listen-IT,
 * EV/ER callbacks), and the firmware sample loop pushes one snapshot per
 * sample via positionRegister.Update().  Wire protocol and register
 * table: docs/i2c.md.
 */
#ifndef APP_I2C_POS_H
#define APP_I2C_POS_H

#include <cstdint>

#include "mt6701.h" /* Mt6701::AngleMax */
#include "gear_decode.h" /* GearPosition */

/** The I2C slave register map of the absolute position. */
class PositionRegister
{
public:
    /** 7-bit slave address of the encoder on the host bus.  Applied to OAR1
     *  by the backend at init (hal_stm32.cpp) - the CubeMX-generated
     *  OwnAddress1=0 in i2c.c is never used. */
    static constexpr uint8_t Address = 0x50u;

    /* Register map (read-only, little-endian on the wire):
     *   0x00..0x03  absolute position count, uint32 LE, 0 .. 121,716,735
     *               (turns * CountsPerTurn + angle; the turn field 0..7428
     *               is 13 bits and the 14-bit fine angle makes the combined
     *               value a 27-bit integer, so it fits one uint32 and
     *               counts continuously across the turn boundary)
     *   0x04        status: bit0 = 1 when the sample passed the slew guard */
    static constexpr uint8_t RegisterStatus = 0x04u;
    static constexpr uint8_t RegisterCount  = 5u;

    /** Fine-angle steps per whole turn (Mt6701::AngleMax + 1).  The
     *  combined count the map exports is turns * this + angle. */
    static constexpr uint32_t CountsPerTurn = Mt6701::AngleMax + 1u;

    static constexpr uint8_t StatusValid = 0x01u; /* slew guard accepted */

    /** Start from an empty snapshot (all zeros, cursor at register 0). */
    PositionRegister();

    /** Clear the snapshot and the register pointer (idempotent). */
    void Init();

    /**
     * Refresh the exported snapshot from the latest decode.  Called once
     * per sample from the sample loop.  The position is packed into a
     * single 32-bit word before publishing, so a concurrent reader
     * (address-match ISR) sees either the old or the new snapshot, never
     * a mix.
     */
    void Update(const GearPosition& position);

    /** Set the register pointer (the byte a host writes before reading);
     *  values past the map clamp to it so reads stay zero-filled. */
    void Select(uint8_t registerIndex);

    /**
     * Copy bytes to the reader starting at the register pointer, advancing
     * it (auto-increment, EEPROM style); past the map every byte reads
     * 0x00.  Reads the packed snapshot a single time per call.
     * @param destination  destination (host TX buffer)
     * @param maxBytes     bytes requested / room available
     * @return             bytes copied (== maxBytes unless the snapshot is
     *                     smaller)
     */
    uint8_t Read(uint8_t* destination, uint8_t maxBytes);

private:
    uint32_t _positionSnapshot; /* regs 0x00..0x03: position count */
    uint8_t  _statusByte;       /* reg 0x04 */
    uint8_t  _registerCursor;   /* register pointer (host-writable),
                                   clamped to RegisterCount */
};

/** The firmware's register map instance; the I2C callbacks in
 *  hal_stm32.cpp serve host reads from it, the sample loop updates it. */
extern PositionRegister positionRegister;

#endif /* APP_I2C_POS_H */