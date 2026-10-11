/**
 * @file i2c_pos.cpp
 * @brief I2C slave register map for the absolute position (see i2c_pos.h).
 *
 * The exported snapshot lives in one 32-bit word: the absolute position
 * count (turns * PositionRegister::CountsPerTurn + angle, little-endian
 * on the wire).  Around it sits one self-consistent status byte (bit0:
 * sample fresh and slew-accepted; bits 1..4: which encoder(s) failed the
 * last read) and a 16-bit sample counter (liveness for the host).  The
 * sample loop writes the snapshot with a single aligned store and the
 * transport copies it with a single aligned load at address-match time,
 * so a host read can never observe a torn position even though the
 * sample updates at 1 kHz.  After the constant full-scale count
 * (MaxPosition, register 0x07) the map ends with each gear's own measured
 * angle (register 0x0B): the raw per-wheel readings next to the decoded
 * result, so a host can see what the four MT6701s actually report.
 */
#include "i2c_pos.h"

PositionRegister positionRegister;

PositionRegister::PositionRegister()
    : _position(0u)
    , _status(0u)
    , _sampleCounter(0u)
    , _gearAngles{}
    , _registerPointer(0u)
{
}

void PositionRegister::Init()
{
    _position = 0u;
    _status = 0u;
    _sampleCounter = 0u;
    for (uint8_t role = 0u; role < GearAngleCount; role++)
    {
        _gearAngles[role] = 0u;
    }
    _registerPointer = 0u;
}

void PositionRegister::Update(const GearPosition& position,
                              const GearAngles& angles, uint8_t readHealth)
{
    /* one aligned 32-bit store: a concurrent reader sees either the old or
       the new count, never a torn mix of turn and angle bytes */
    _position = static_cast<uint32_t>(position.Turns) * CountsPerTurn +
                position.Angle;
    _sampleCounter++; /* liveness: +1 per sample, wraps (docs/i2c.md) */

    /* Each wheel's own angle, in EncoderRole order (Sun, Gear1..Gear3), so
       the raw readings sit next to the decoded position from the same
       sample. */
    const uint16_t measuredAngles[] = {angles.Sun, angles.Gear1, angles.Gear2,
                                       angles.Gear3};
    static_assert(sizeof(measuredAngles) / sizeof(measuredAngles[0]) ==
                      GearAngleCount,
                  "one measured angle per gear register, in EncoderRole order");
    for (uint8_t role = 0u; role < GearAngleCount; role++)
    {
        _gearAngles[role] = measuredAngles[role];
    }

    /* one byte carries both verdicts, so they can never disagree:
       the valid bit only when every read succeeded and the slew guard
       accepted, plus one bit per failed encoder read (bits 1..4) */
    _status = static_cast<uint8_t>(
        readHealth | ((position.IsValid && readHealth == 0u) ? StatusValid
                                                             : 0u));
}

void PositionRegister::Select(uint8_t registerIndex)
{
    _registerPointer =
        (registerIndex < RegisterCount) ? registerIndex : RegisterCount;
}

/* One field of the register map: the register it starts at, how many bytes
 * wide it is, and the value it serves.  The fields *are* the map: Read()
 * walks them in order and asks each for whatever part of it falls inside
 * the host's read window, so there is no per-register branch to keep in
 * step with the layout (the static_assert in i2c_pos.h guarantees the
 * fields tile the whole map). */
struct RegisterField
{
    uint8_t  FirstRegister;
    uint8_t  ByteWidth;
    uint32_t Value;

    /* Copy the bytes of this field that fall inside [readStart, readEnd)
     * to `destination`, returning how many were written (0 when the window
     * misses the field).  Overlap at either end may be partial: a host is
     * allowed to start reading in the middle of a value. */
    uint8_t Serve(uint16_t readStart, uint16_t readEnd,
                  uint8_t* destination) const
    {
        uint8_t        bytesWritten = 0u;
        const uint16_t fieldEnd =
            static_cast<uint16_t>(FirstRegister + ByteWidth);
        for (uint16_t reg = FirstRegister; reg < fieldEnd; reg++)
        {
            if (reg < readStart || reg >= readEnd)
            {
                continue; /* this register of mine is outside the window */
            }
            const uint8_t byteIndex =
                static_cast<uint8_t>(reg - FirstRegister); /* 0 = low byte */
            destination[bytesWritten] =
                static_cast<uint8_t>(Value >> (8u * byteIndex));
            bytesWritten++;
        }
        return bytesWritten;
    }
};

uint8_t PositionRegister::Read(uint8_t* destination, uint8_t maxBytes)
{
    /* One capture per transaction: every byte below is served from these
       locals, never from the live fields, so position is read as a single
       32-bit load and its four bytes can never tear even though the sample
       loop refreshes it at 1 kHz.  The gear angles are captured the same
       way, so one frame describes one sample, never a mix of two. */
    const uint32_t position = _position;
    const uint8_t  status   = _status;
    const uint16_t counter  = _sampleCounter;
    uint16_t       gearAngles[GearAngleCount];
    for (uint8_t role = 0u; role < GearAngleCount; role++)
    {
        gearAngles[role] = _gearAngles[role];
    }

    /* The register window the host asked for; the start pointer is clamped
       to the map, so everything past RegisterCount is zero-filled. */
    const uint16_t readStart = _registerPointer;
    const uint16_t readEnd = static_cast<uint16_t>(readStart + maxBytes);
    uint8_t        bytesWritten = 0u;

    /* The whole map as one list of fields: the four fixed registers, then
       one gear angle per role.  Adding a register = adding one line here.
       The fields tile 0x00..RegisterCount (static_assert in i2c_pos.h), so
       inside the window they serve exactly the bytes asked for. */
    const RegisterField fixedFields[] = {
        {0x00u, 4u, position},
        {RegisterStatus, 1u, status},
        {RegisterCounter, 2u, counter},
        {RegisterMaxPosition, 4u, MaxPosition},
    };
    static constexpr uint8_t fixedFieldCount = static_cast<uint8_t>(
        sizeof(fixedFields) / sizeof(fixedFields[0]));

    RegisterField fields[fixedFieldCount + GearAngleCount];
    uint8_t       fieldCount = 0u;
    for (uint8_t field = 0u; field < fixedFieldCount; field++)
    {
        fields[fieldCount++] = fixedFields[field];
    }
    for (uint8_t role = 0u; role < GearAngleCount; role++)
    {
        fields[fieldCount++] = {GearAngleRegister(static_cast<EncoderRole>(role)),
                                BytesPerGearAngle, gearAngles[role]};
    }

    for (uint8_t field = 0u; field < fieldCount; field++)
    {
        bytesWritten += fields[field].Serve(readStart, readEnd,
                                            destination + bytesWritten);
    }
    while (bytesWritten < maxBytes)
    {
        destination[bytesWritten] = 0u; /* past the map: zero-fill, never wraps */
        bytesWritten++;
    }

    /* Auto-increment, stopping at the map end. */
    const uint16_t nextPointer = static_cast<uint16_t>(readStart + bytesWritten);
    _registerPointer = (nextPointer < RegisterCount)
                           ? static_cast<uint8_t>(nextPointer)
                           : RegisterCount;
    return bytesWritten;
}
