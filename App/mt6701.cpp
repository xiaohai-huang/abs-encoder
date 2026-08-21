/**
 * @file mt6701.cpp
 * @brief MT6701 SSI frame exchange and validation (platform independent).
 */
#include "mt6701.h"

#include "hal.h"

uint8_t Mt6701::UpdateCrc6(uint8_t crc, uint8_t frameByte, int bitCount)
{
    for (int bitIndex = bitCount - 1; bitIndex >= 0; bitIndex--)
    {
        uint8_t xorBit = static_cast<uint8_t>((crc >> 5) ^
                                              ((frameByte >> bitIndex) & 1u));
        crc = static_cast<uint8_t>((crc << 1) & 0x3Fu);
        if (xorBit != 0u)
        {
            crc = static_cast<uint8_t>((crc ^ Crc6Polynomial) & 0x3Fu);
        }
    }
    return crc;
}

uint8_t Mt6701::ComputeCrc6(const uint8_t frame[3])
{
    uint8_t crc = UpdateCrc6(0u, frame[0], 8);
    crc = UpdateCrc6(crc, frame[1], 8);
    return UpdateCrc6(crc, static_cast<uint8_t>(frame[2] >> 6), 2);
}

int Mt6701::ReadFrame(Hal& hal, EncoderRole encoder, uint8_t frame[3])
{
    /* SSI is unidirectional: MOSI is don't-care while the chip shifts out. */
    hal.SelectChip(encoder, true); /* assert (pull low): active-low chip select */
    for (uint32_t byteIndex = 0u; byteIndex < 3u; byteIndex++)
    {
        frame[byteIndex] = hal.SpiTransfer(encoder, 0u);
    }
    hal.SelectChip(encoder, false);
    return 0;
}

int Mt6701::ReadSample(Hal& hal, EncoderRole encoder, Mt6701Sample& sample)
{
    if (encoder >= EncoderRole::RoleCount)
    {
        return -2;
    }
    bool sawFault = false;

    for (int attempt = 0; attempt < MaxReadRetries; attempt++)
    {
        uint8_t frame[3];
        ReadFrame(hal, encoder, frame);

#if Mt6701Crc6Enabled
        if (ComputeCrc6(frame) != (frame[2] & 0x3Fu))
        {
            hal.DelayMicroseconds(RetryDelayMicroseconds);
            continue;
        }
#endif

        uint16_t angle = static_cast<uint16_t>(
            (static_cast<uint16_t>(frame[0]) << 6) |
            (static_cast<uint16_t>(frame[1]) >> 2));
        uint8_t status = static_cast<uint8_t>(
            ((frame[1] & 0x03u) << 2) | (frame[2] >> 6));

        /* status: bit3 = track loss, bit2 = button, bits1..0 = field status. */
        if ((status & 0x08u) != 0u || (status & 0x03u) != 0u)
        {
            sawFault = true;
            hal.DelayMicroseconds(RetryDelayMicroseconds);
            continue;
        }

        sample.Angle = angle;
        sample.IsButtonPressed = (status & 0x04u) != 0u;
        return 0;
    }

    return sawFault ? -1 : -2;
}