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
 * sample updates at 1 kHz.  The map ends with the constant full-scale
 * count (MaxPosition, register 0x07), which a host reads once to scale
 * positions against the range this build actually has.
 */
#include "i2c_pos.h"

PositionRegister positionRegister;

PositionRegister::PositionRegister()
    : _positionSnapshot(0u)
    , _statusByte(0u)
    , _sampleCounter(0u)
    , _registerCursor(0u)
{
}

void PositionRegister::Init()
{
    _positionSnapshot = 0u;
    _statusByte = 0u;
    _sampleCounter = 0u;
    _registerCursor = 0u;
}

void PositionRegister::Update(const GearPosition& position, uint8_t readHealth)
{
    /* one aligned 32-bit store: a concurrent reader sees either the old or
       the new count, never a torn mix of turn and angle bytes */
    _positionSnapshot =
        static_cast<uint32_t>(position.Turns) * CountsPerTurn + position.Angle;
    _sampleCounter++; /* liveness: +1 per sample, wraps (docs/i2c.md) */
    /* one byte carries both verdicts, so they can never disagree:
       the valid bit only when every read succeeded and the slew guard
       accepted, plus one bit per failed encoder read (bits 1..4) */
    _statusByte =
        static_cast<uint8_t>(readHealth |
                             ((position.IsValid && readHealth == 0u)
                                  ? StatusValid
                                  : 0u));
}

void PositionRegister::Select(uint8_t registerIndex)
{
    _registerCursor =
        (registerIndex < RegisterCount) ? registerIndex : RegisterCount;
}

/* One row of the register map: the register pointer it starts at, its
 * width in bytes, and the value it serves.  This table *is* the map --
 * Read() matches the cursor against the rows, so there is no per-register
 * branch to keep in step with the layout (the static_assert in i2c_pos.h
 * guarantees the rows tile the whole map). */
struct RegisterRow
{
    uint8_t  base;
    uint8_t  size;
    uint32_t value;
};

/* Byte `offset` of a little-endian register value. */
static uint8_t ByteOf(uint32_t value, uint8_t offset)
{
    return static_cast<uint8_t>(value >> (8u * offset));
}

uint8_t PositionRegister::Read(uint8_t* destination, uint8_t maxBytes)
{
    /* One capture per transaction: every byte below is served from these
       locals, never from the live fields, so position is read as a single
       32-bit load and its four bytes can never tear even though the sample
       loop refreshes it at 1 kHz. */
    const uint32_t position = _positionSnapshot;
    const uint8_t  status   = _statusByte;
    const uint16_t counter  = _sampleCounter;

    const RegisterRow rows[] = {
        {0x00u, 4u, position},
        {RegisterStatus, 1u, status},
        {RegisterCounter, 2u, counter},
        {RegisterMaxPosition, 4u, MaxPosition},
    };
    const uint8_t rowCount =
        static_cast<uint8_t>(sizeof(rows) / sizeof(rows[0]));

    uint8_t index = 0u;
    while (index < maxBytes)
    {
        uint8_t byte = 0u; /* past the map: zero-fill, never wraps */
        if (_registerCursor < RegisterCount)
        {
            /* serve the byte of the row whose range covers the cursor */
            for (uint8_t row = 0u; row < rowCount; row++)
            {
                const uint8_t base = rows[row].base;
                if (_registerCursor >= base &&
                    _registerCursor < (uint8_t)(base + rows[row].size))
                {
                    byte = ByteOf(rows[row].value, _registerCursor - base);
                    break;
                }
            }
            _registerCursor++;
        }
        destination[index] = byte;
        index++;
    }
    return index;
}