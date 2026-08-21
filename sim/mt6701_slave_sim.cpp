/**
 * @file mt6701_slave_sim.cpp
 * @brief Simulated MT6701 SSI slaves.  Mirror the frame construction from
 *        the datasheet: 24 bits MSB first, angle in bits 23..10, status
 *        bits 9..6, CRC-6 in bits 5..0.  Use the same CRC function as the
 *        firmware driver (App/mt6701.cpp).  One chip per encoder role.
 */
#include "mt6701_slave_sim.h"

#include "mt6701.h"

namespace
{
size_t RoleIndex(EncoderRole encoder)
{
    return static_cast<size_t>(encoder);
}
} // namespace

Mt6701SlaveSim::SimulatedChip
    Mt6701SlaveSim::_chips[Mt6701::EncoderCount];

void Mt6701SlaveSim::BuildFrame(SimulatedChip& chip)
{
    uint32_t frame = static_cast<uint32_t>(chip.Angle & Mt6701::AngleMax) << 10;
    if (chip.IsButtonPressed)
    {
        frame |= 1u << 8; /* frame bit 8 = Z / push state */
    }
    if (chip.HasTrackLoss)
    {
        frame |= 1u << 9; /* frame bit 9 = track loss */
    }
    frame |= static_cast<uint32_t>(chip.FieldStatus & 0x03u) << 6;

    uint8_t frameBytes[3] = {static_cast<uint8_t>(frame >> 16),
                             static_cast<uint8_t>(frame >> 8),
                             static_cast<uint8_t>(frame)};
    uint8_t crc = Mt6701::ComputeCrc6(frameBytes);
    if (chip.IsCrcBroken)
    {
        crc ^= 0x01u;
    }

    chip.Frame = frame | static_cast<uint32_t>(crc & 0x3Fu);
}

void Mt6701SlaveSim::Init()
{
    for (size_t i = 0u; i < Mt6701::EncoderCount; i++)
    {
        _chips[i].Angle = 0u;
        _chips[i].IsButtonPressed = false;
        _chips[i].HasTrackLoss = false;
        _chips[i].IsCrcBroken = false;
        _chips[i].IsStuck = false;
        _chips[i].FieldStatus = 0u;
        _chips[i].Frame = 0u;
        _chips[i].ByteIndex = -1;
    }
}

void Mt6701SlaveSim::SelectChip(EncoderRole encoder, bool asserted)
{
    if (encoder >= EncoderRole::RoleCount)
    {
        return;
    }

    SimulatedChip& chip = _chips[RoleIndex(encoder)];
    if (asserted)
    {
        BuildFrame(chip);
        chip.ByteIndex = 0;
    }
    else
    {
        chip.ByteIndex = -1;
    }
}

uint8_t Mt6701SlaveSim::Transfer(EncoderRole encoder, uint8_t txByte)
{
    (void)txByte; /* SSI is unidirectional; MOSI is ignored by the chip */

    if (encoder >= EncoderRole::RoleCount)
    {
        return 0xFFu;
    }

    SimulatedChip& chip = _chips[RoleIndex(encoder)];
    if (chip.IsStuck)
    {
        return 0xFFu;
    }

    uint8_t rxByte = 0u;
    if (chip.ByteIndex >= 0 && chip.ByteIndex < 3)
    {
        rxByte = static_cast<uint8_t>(chip.Frame >> (16 - 8 * chip.ByteIndex));
    }
    chip.ByteIndex++;
    return rxByte;
}

void Mt6701SlaveSim::SetAngle(EncoderRole encoder, uint16_t angle)
{
    if (encoder < EncoderRole::RoleCount)
    {
        _chips[RoleIndex(encoder)].Angle = angle & Mt6701::AngleMax;
    }
}

void Mt6701SlaveSim::SetButton(EncoderRole encoder, bool pressed)
{
    if (encoder < EncoderRole::RoleCount)
    {
        _chips[RoleIndex(encoder)].IsButtonPressed = pressed;
    }
}

void Mt6701SlaveSim::SetTrackLoss(EncoderRole encoder, bool enabled)
{
    if (encoder < EncoderRole::RoleCount)
    {
        _chips[RoleIndex(encoder)].HasTrackLoss = enabled;
    }
}

void Mt6701SlaveSim::SetFieldStatus(EncoderRole encoder, uint8_t status)
{
    if (encoder < EncoderRole::RoleCount)
    {
        _chips[RoleIndex(encoder)].FieldStatus = status & 0x03u;
    }
}

void Mt6701SlaveSim::SetCrcBroken(EncoderRole encoder, bool broken)
{
    if (encoder < EncoderRole::RoleCount)
    {
        _chips[RoleIndex(encoder)].IsCrcBroken = broken;
    }
}

void Mt6701SlaveSim::SetStuck(EncoderRole encoder, bool stuck)
{
    if (encoder < EncoderRole::RoleCount)
    {
        _chips[RoleIndex(encoder)].IsStuck = stuck;
    }
}