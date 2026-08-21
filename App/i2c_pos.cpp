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
 * sample updates at 1 kHz.
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

uint8_t PositionRegister::Read(uint8_t* destination, uint8_t maxBytes)
{
    /* one load per field: every byte of a transaction comes from the same
       snapshot and status state (position is a single 32-bit load, so its
       four bytes can never tear) */
    uint32_t snapshot = _positionSnapshot;
    uint8_t  status   = _statusByte;
    uint16_t counter  = _sampleCounter;

    uint8_t index = 0u;
    while (index < maxBytes)
    {
        if (_registerCursor < RegisterCount)
        {
            uint8_t registerIndex = _registerCursor++;
            if (registerIndex == RegisterStatus)
            {
                destination[index] = status;
            }
            else if (registerIndex >= RegisterCounter)
            {
                destination[index] =
                    static_cast<uint8_t>((counter >> (8u * (registerIndex -
                                                            RegisterCounter))) &
                                         0xFFu);
            }
            else
            {
                destination[index] =
                    static_cast<uint8_t>(snapshot >> (8u * registerIndex));
            }
        }
        else
        {
            destination[index] = 0u; /* past the map: zero-fill, never wraps */
        }
        index++;
    }
    return index;
}