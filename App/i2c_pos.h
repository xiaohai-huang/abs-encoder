/**
 * @file i2c_pos.h
 * @brief I2C slave host port: the decoded absolute position and each gear's
 *        own angle, as a small register map another device on the I2C bus
 *        can poll at any time.
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
#include "gear_config.h" /* EncoderRole, GearConfig::TurnRange */
#include "gear_decode.h" /* GearPosition, GearAngles */

/** The I2C slave register map of the absolute position. */
class PositionRegister
{
public:
    /** 7-bit slave address of the encoder on the host bus.  Applied to OAR1
     *  by the backend at init (hal_stm32.cpp) - the CubeMX-generated
     *  OwnAddress1=0 in i2c.c is never used. */
    static constexpr uint8_t Address = 0x50u;

    /* Register map (read-only, little-endian on the wire):
     *   0x00..0x03  absolute position count, uint32 LE, 0 .. MaxPosition
     *               (turns * CountsPerTurn + angle; the turn field 0..7428
     *               is 13 bits and the 14-bit fine angle makes the combined
     *               value a 27-bit integer, so it fits one uint32 and
     *               counts continuously across the turn boundary)
     *   0x04        status: one byte carrying the whole sample verdict --
     *               bit0 = 1 when the last sample is fresh and accepted
     *               (all four encoder reads succeeded AND the slew guard
     *               passed, i.e. the register reads exactly StatusValid);
     *               bits 1..4 name any encoder whose read failed on the
     *               last sample (bit1 = Sun .. bit4 = Gear3)
     *   0x05..0x06  sample counter, uint16 LE: +1 per sample (wraps); a
     *               polling host sees sampling is alive when it changes
     *   0x07..0x0A  full-scale count, uint32 LE: the constant MaxPosition,
     *               so a host scales positions against the range this
     *               build actually has instead of a copied number
     *   0x0B..0x12  gear angles: the 14-bit fine angle each wheel's own
     *               MT6701 reported on the latest sample, uint16 LE, one
     *               register per EncoderRole in role order (Sun at 0x0B,
     *               Gear1 0x0D, Gear2 0x0F, Gear3 0x11).  The raw
     *               per-wheel readings, for bring-up: the Sun register
     *               repeats the low 14 bits of the position count, and the
     *               driven registers show what those three gears see,
     *               which the decoded position on its own cannot.
     */
    static constexpr uint8_t RegisterStatus      = 0x04u;
    static constexpr uint8_t RegisterCounter     = 0x05u;
    static constexpr uint8_t RegisterMaxPosition = 0x07u;
    /** First gear angle (the Sun's); role `r`'s register is at
     *  RegisterGearAngle + r * GearAngleStride. */
    static constexpr uint8_t RegisterGearAngle = 0x0Bu;
    /** Bytes per gear angle: one uint16 per wheel. */
    static constexpr uint8_t GearAngleStride = 2u;
    /** Gear-angle registers: one per EncoderRole. */
    static constexpr uint8_t GearAngleRows =
        static_cast<uint8_t>(EncoderRole::RoleCount);
    /** One past the last byte of the map (the last gear angle is 2 bytes). */
    static constexpr uint8_t RegisterCount =
        RegisterGearAngle + GearAngleStride * GearAngleRows;

    /** Register pointer of `role`'s gear angle. */
    static constexpr uint8_t GearAngleRegister(EncoderRole role)
    {
        return static_cast<uint8_t>(
            RegisterGearAngle + GearAngleStride * static_cast<uint8_t>(role));
    }

    /* The map tiles 0x00..RegisterCount-1 with no gap and no overlap: the
     * four fixed registers, then one gear angle per role.  Read() walks the
     * rows in that order and serves each byte from the one row that covers
     * it, so this is what makes every valid register pointer land in
     * exactly one row. */
    static_assert(RegisterStatus == 4u &&
                      RegisterCounter == RegisterStatus + 1u &&
                      RegisterMaxPosition == RegisterCounter + 2u &&
                      RegisterGearAngle == RegisterMaxPosition + 4u &&
                      RegisterCount ==
                          RegisterGearAngle + GearAngleStride * GearAngleRows,
                  "register map must tile: pos 0x00..0x03, status 0x04, "
                  "seq 0x05..0x06, max 0x07..0x0A, gear angles 0x0B..0x12");

    /** Fine-angle steps per whole turn (Mt6701::AngleMax + 1).  The
     *  combined count the map exports is turns * this + angle. */
    static constexpr uint32_t CountsPerTurn = Mt6701::AngleMax + 1u;

    /** Largest count the map can report: the whole turn range at full
     *  fine-angle resolution, i.e. TurnRange turns of 0 .. AngleMax
     *  (7429 * 16384 - 1 = 121,716,735).  Derived from the geometry, so
     *  a gear_config.h change moves the exported range with it. */
    static constexpr uint32_t MaxPosition =
        static_cast<uint32_t>(GearConfig::TurnRange) * CountsPerTurn - 1u;

    /** The status register reads exactly this when the sample is
     *  certified; anything else means a read failed (bits 1..4) or the
     *  slew guard rejected the decode (0x00). */
    static constexpr uint8_t StatusValid = 0x01u;

    /** Status bit of a role, set when that encoder's read failed on the
     *  last sample.  Bits 1..4, numbering follows EncoderRole order
     *  (Sun = bit1 .. Gear3 = bit4); bit0 of the register is StatusValid. */
    static constexpr uint8_t HealthBit(EncoderRole role)
    {
        return static_cast<uint8_t>(1u << (static_cast<uint8_t>(role) + 1u));
    }

    /** Start from an empty snapshot (all zeros, cursor at register 0). */
    PositionRegister();

    /** Clear the snapshot and the register pointer (idempotent). */
    void Init();

    /**
     * Refresh the exported snapshot from the latest sample.  Called once
     * per sample from the sample loop.  The position is packed into a
     * single 32-bit word before publishing, so a concurrent reader
     * (address-match ISR) sees either the old or the new count, never a
     * mix.
     * @param position   decoded position (Turns/Angle always exported)
     * @param angles     the four angle samples this position was decoded
     *                   from; the gear-angle registers export each wheel's
     *                   own measured angle from the same set
     * @param readHealth one HealthBit per encoder whose read failed on
     *                   this sample; any set bit also clears the status
     *                   valid bit, so the status byte stays a single
     *                   self-consistent verdict.  The counter and status
     *                   bytes always reflect this Update.
     */
    void Update(const GearPosition& position, const GearAngles& angles,
                uint8_t readHealth);

    /** Set the register pointer (the byte a host writes before reading);
     *  values past the map clamp to it so reads stay zero-filled. */
    void Select(uint8_t registerIndex);

    /**
     * Copy bytes to the reader starting at the register pointer, advancing
     * it (auto-increment, EEPROM style); past the map every byte reads
     * 0x00.  Reads the packed snapshot a single time per call.
     * @param destination  destination (host TX buffer)
     * @param maxBytes     bytes requested / room available
     * @return             bytes written - always maxBytes, never fewer,
     *                     so destination must have room for maxBytes
     */
    uint8_t Read(uint8_t* destination, uint8_t maxBytes);

private:
    uint32_t _positionSnapshot;       /* regs 0x00..0x03: position count */
    uint8_t  _statusByte;             /* reg 0x04: bit0 valid, 1..4 health */
    uint16_t _sampleCounter;          /* regs 0x05..0x06: liveness, +1/sample */
    uint16_t _gearAngle[GearAngleRows]; /* regs 0x0B..: measured angle per role */
    uint8_t  _registerCursor;         /* register pointer (host-writable),
                                         clamped to RegisterCount */
};

/** The firmware's register map instance; the I2C callbacks in
 *  hal_stm32.cpp serve host reads from it, the sample loop updates it. */
extern PositionRegister positionRegister;

#endif /* APP_I2C_POS_H */
