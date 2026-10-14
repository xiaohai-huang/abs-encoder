/**
 * @file i2c_pos.h
 * @brief I2C slave host port: the decoded absolute position, each gear's
 *        own angle and per-wheel read diagnostics (why a read failed,
 *        how often, and when), as a small register map another device on
 *        the I2C bus can poll at any time.
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
     *   0x13..0x32  read diagnostics: one 8-byte block per EncoderRole in
     *               role order (Sun at 0x13, Gear1 0x1B, Gear2 0x23, Gear3
     *               0x2B; DiagnosticsRegister(role) computes these), why
     *               the wheel's reads end the way they do:
     *                 +0        cause: CauseOk, CauseMagnetic (frame
     *                           passed CRC-6 but the chip reported track
     *                           loss / bad field) or CauseNoFrame (no
     *                           frame passed CRC-6 in the retries)
     *                 +1        mg: the raw MT6701 status Mg[3:0] of the
     *                           frame that decided the read
     *                           (MgStatusUnknown when none did)
     *                 +2..+3    frameFails, uint16 LE: samples that ended
     *                           CauseNoFrame since power-up, saturating
     *                 +4..+5    statusFails, uint16 LE: samples that ended
     *                           CauseMagnetic since power-up, saturating
     *                 +6..+7    lastFailSeq, uint16 LE: the counter value
     *                           of the sample on which the wheel last
     *                           failed (0 = never)
     */
    static constexpr uint8_t RegisterStatus      = 0x04u;
    static constexpr uint8_t RegisterCounter     = 0x05u;
    static constexpr uint8_t RegisterMaxPosition = 0x07u;
    /** First gear angle (the Sun's); role `r`'s register is at
     *  RegisterGearAngle + r * BytesPerGearAngle. */
    static constexpr uint8_t RegisterGearAngle = 0x0Bu;
    /** Bytes one gear angle occupies on the wire: a uint16 per wheel. */
    static constexpr uint8_t BytesPerGearAngle = 2u;
    /** First diagnostics block (the Sun's); role `r`'s block is at
     *  RegisterDiagnostics + r * BytesPerDiagnostics. */
    static constexpr uint8_t RegisterDiagnostics = 0x13u;
    /** Bytes one wheel's diagnostics block occupies on the wire. */
    static constexpr uint8_t BytesPerDiagnostics = 8u;
    /** Wheels the map describes, one entry per EncoderRole. */
    static constexpr uint8_t WheelCount =
        static_cast<uint8_t>(EncoderRole::RoleCount);
    /** Gear angles in the map: one per EncoderRole. */
    static constexpr uint8_t GearAngleCount = WheelCount;
    /** One past the last byte of the map (the last diagnostics block is
     *  8 bytes). */
    static constexpr uint8_t RegisterCount =
        RegisterDiagnostics + BytesPerDiagnostics * WheelCount;

    /** Register pointer of `role`'s gear angle. */
    static constexpr uint8_t GearAngleRegister(EncoderRole role)
    {
        return static_cast<uint8_t>(
            RegisterGearAngle + BytesPerGearAngle * static_cast<uint8_t>(role));
    }

    /** Register pointer of the first byte of `role`'s diagnostics block. */
    static constexpr uint8_t DiagnosticsRegister(EncoderRole role)
    {
        return static_cast<uint8_t>(
            RegisterDiagnostics + BytesPerDiagnostics * static_cast<uint8_t>(role));
    }

    /* The map tiles 0x00..RegisterCount-1 with no gap and no overlap: the
     * four fixed registers, then one gear angle per role, then one
     * diagnostics block per role.  Read() walks the fields in that order
     * and serves each byte from the one field that covers it, so this is
     * what makes every valid register pointer land in exactly one field. */
    static_assert(RegisterStatus == 4u &&
                      RegisterCounter == RegisterStatus + 1u &&
                      RegisterMaxPosition == RegisterCounter + 2u &&
                      RegisterGearAngle == RegisterMaxPosition + 4u &&
                      RegisterDiagnostics ==
                          RegisterGearAngle + BytesPerGearAngle * GearAngleCount &&
                      RegisterCount ==
                          RegisterDiagnostics + BytesPerDiagnostics * WheelCount,
                  "register map must tile: pos 0x00..0x03, status 0x04, "
                  "seq 0x05..0x06, max 0x07..0x0A, gear angles 0x0B..0x12, "
                  "diagnostics 0x13..0x32");

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

    /* cause byte values of the diagnostics block (docs/i2c.md): what the
     * last read of that wheel did, the two failure causes kept apart
     * because they point at different hardware. */
    static constexpr uint8_t CauseOk       = 0u; /* read succeeded */
    static constexpr uint8_t CauseMagnetic = 1u; /* CRC-valid frame, chip
                                                    reported track loss /
                                                    bad field (magnet) */
    static constexpr uint8_t CauseNoFrame  = 2u; /* no frame passed CRC-6
                                                    (wiring / SPI / chip) */

    /** mg byte value when the sample produced no CRC-valid frame, so the
     *  raw status it would export is unknown (Mt6701::FrameStatusUnknown;
     *  a real Mg[3:0] nibble never exceeds 0x0F). */
    static constexpr uint8_t MgStatusUnknown = Mt6701::FrameStatusUnknown;

    /** One wheel's read outcome on one sample, as Update consumes it and
     *  the diagnostics block exports it (docs/i2c.md). */
    struct ReadResult
    {
        uint8_t Cause;    /* CauseOk / CauseMagnetic / CauseNoFrame */
        uint8_t MgStatus; /* Mg[3:0] of the deciding frame, or
                           * MgStatusUnknown */
    };

    /** Start from an empty snapshot (all zeros, register pointer at 0). */
    PositionRegister();

    /** Clear the snapshot and the register pointer (idempotent). */
    void Init();

    /**
     * Refresh the exported snapshot from the latest sample.  Called once
     * per sample from the sample loop.  The position is packed into a
     * single 32-bit word before publishing, so a concurrent reader
     * (address-match ISR) sees either the old or the new count, never a
     * mix.
     * @param position  decoded position (Turns/Angle always exported)
     * @param angles    the four angle samples this position was decoded
     *                  from; the gear-angle registers export each wheel's
     *                  own measured angle from the same set
     * @param results   one ReadResult per wheel, in EncoderRole order:
     *                  the cause and raw chip status of that wheel's read
     *                  on this sample.  The diagnostics block stores them
     *                  as-is and bumps the matching failure counter and
     *                  last-failure seq; the status byte's health bits are
     *                  derived from the same causes (any non-CauseOk also
     *                  clears the valid bit), so the verdict and the
     *                  detail can never disagree.
     */
    void Update(const GearPosition& position, const GearAngles& angles,
                const ReadResult (&results)[WheelCount]);

    /** Set the register pointer (the byte a host writes before reading);
     *  values past the map clamp to it so reads stay zero-filled. */
    void Select(uint8_t registerIndex);

    /**
     * Compose the whole register map for the reader, starting at the
     * register pointer: destination[0] is the byte at the pointer, the map
     * bytes before it are skipped, and the tail past the map end reads
     * 0x00.  Reads the packed snapshot a single time per call, so every
     * byte of one call describes one sample.
     *
     * The width is the whole map because the host's read length is not
     * knowable here: the I2C shift register takes a prefix of what was
     * composed and the master's NACK decides where it ends.  The pointer
     * is consumed by the read (it parks past the map), so a read without a
     * preceding Select returns zeros rather than stale map bytes.
     * @param destination  host TX buffer; the array reference makes the
     *                     RegisterCount-byte requirement compile-time
     */
    void Read(uint8_t (&destination)[RegisterCount]);

private:
    /* One wheel's diagnostics block as stored (reg 0x13 + 8 * role): the
     * members mirror the wire layout docs/i2c.md defines, though Read()
     * serves each one as its own field rather than memcpy-ing the struct. */
    struct Diagnostics
    {
        uint8_t  Cause;       /* CauseOk / CauseMagnetic / CauseNoFrame */
        uint8_t  MgStatus;    /* Mg[3:0] of the deciding frame, or unknown */
        uint16_t FrameFails;  /* CauseNoFrame samples since power-up */
        uint16_t StatusFails; /* CauseMagnetic samples since power-up */
        uint16_t LastFailSeq; /* counter value of the last failed sample */
    };

    uint32_t _position;                 /* regs 0x00..0x03: position count --
                                           one aligned word, so a concurrent
                                           reader never sees a torn mix */
    uint8_t  _status;                   /* reg 0x04: bit0 valid, 1..4 health */
    uint16_t _sampleCounter;            /* regs 0x05..0x06: liveness, +1/sample */
    uint16_t _gearAngles[GearAngleCount]; /* regs 0x0B..: measured angle per role */
    Diagnostics _diagnostics[WheelCount]; /* regs 0x13..: per-wheel read causes,
                                             counters and last-failure seq */
    uint8_t  _registerPointer;          /* register pointer (host-writable),
                                           clamped to RegisterCount */
};

/** The firmware's register map instance; the I2C callbacks in
 *  hal_stm32.cpp serve host reads from it, the sample loop updates it. */
extern PositionRegister positionRegister;

#endif /* APP_I2C_POS_H */
