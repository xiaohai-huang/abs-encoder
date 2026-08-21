/**
 * @file i2c_pos.cpp
 * @brief I2C slave register map for the absolute position (see i2c_pos.h).
 *
 * The exported snapshot lives in one 32-bit word: the absolute position
 * count (turns * PositionRegister::CountsPerTurn + angle, little-endian
 * on the wire) plus one status byte.  The sample loop writes it with a
 * single aligned store and the transport copies it with a single aligned
 * load at address-match time, so a host read can never observe a torn
 * position even though the sample updates at 1 kHz.
 */
#include "i2c_pos.h"

PositionRegister positionRegister;

PositionRegister::PositionRegister()
    : _positionSnapshot(0u)
    , _statusByte(0u)
    , _registerCursor(0u)
{
}

void PositionRegister::Init()
{
    _positionSnapshot = 0u;
    _statusByte = 0u;
    _registerCursor = 0u;
}

void PositionRegister::Update(const GearPosition& position)
{
    /* one aligned 32-bit store: a concurrent reader sees either the old or
       the new count, never a torn mix of turn and angle bytes */
    _positionSnapshot =
        static_cast<uint32_t>(position.Turns) * CountsPerTurn + position.Angle;
    _statusByte = position.IsValid ? StatusValid : 0u;
}

void PositionRegister::Select(uint8_t registerIndex)
{
    _registerCursor =
        (registerIndex < RegisterCount) ? registerIndex : RegisterCount;
}

uint8_t PositionRegister::Read(uint8_t* destination, uint8_t maxBytes)
{
    uint32_t snapshot = _positionSnapshot; /* one 32-bit load: atomic */
    uint8_t index = 0u;
    while (index < maxBytes)
    {
        if (_registerCursor < RegisterCount)
        {
            uint8_t registerIndex = _registerCursor++;
            destination[index] = (registerIndex == RegisterStatus)
                                     ? _statusByte
                                     : static_cast<uint8_t>(snapshot >>
                                                            (8u * registerIndex));
        }
        else
        {
            destination[index] = 0u; /* past the map: zero-fill, never wraps */
        }
        index++;
    }
    return index;
}