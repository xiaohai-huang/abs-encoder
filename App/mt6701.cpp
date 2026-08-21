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
        uint8_t xorBit = static_cast<uint8_t>((crc >> CrcTopBitShift) ^
                                              ((frameByte >> bitIndex) & 1u));
        crc = static_cast<uint8_t>((crc << 1) & CrcFieldMask);
        if (xorBit != 0u)
        {
            crc = static_cast<uint8_t>((crc ^ Crc6Polynomial) & CrcFieldMask);
        }
    }
    return crc;
}

uint8_t Mt6701::ComputeCrc6(const uint8_t frame[BytesInFrame])
{
    uint8_t crc = UpdateCrc6(0u, frame[0], BitsInByte);
    crc = UpdateCrc6(crc, frame[1], BitsInByte);
    return UpdateCrc6(crc, static_cast<uint8_t>(frame[2] >> CrcFieldBits),
                      BitsInByte - CrcFieldBits);
}

int Mt6701::ReadFrame(Hal& hal, EncoderRole encoder, uint8_t frame[BytesInFrame])
{
    /* SSI is unidirectional: MOSI is don't-care while the chip shifts out. */
    hal.SelectChip(encoder, true); /* assert (pull low): active-low chip select */
    for (uint32_t byteIndex = 0u; byteIndex < BytesInFrame; byteIndex++)
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
        return ErrNoValidFrame;
    }
    bool sawFault = false;

    for (int attempt = 0; attempt < MaxReadRetries; attempt++)
    {
        uint8_t frame[BytesInFrame];
        ReadFrame(hal, encoder, frame);

        uint32_t raw = (static_cast<uint32_t>(frame[0]) << (2 * BitsInByte)) |
                       (static_cast<uint32_t>(frame[1]) << BitsInByte) |
                       static_cast<uint32_t>(frame[2]);

#if Mt6701Crc6Enabled
        if (ComputeCrc6(frame) != static_cast<uint8_t>(raw & CrcFieldMask))
        {
            hal.DelayMicroseconds(RetryDelayMicroseconds);
            continue;
        }
#endif

        uint16_t angle = static_cast<uint16_t>(
            (raw >> AngleFieldShift) & AngleFieldMask);
        uint8_t status = static_cast<uint8_t>(
            (raw >> StatusFieldShift) & RawStatusMask);

        if ((status & StatusTrackLossMask) != 0u ||
            (status & StatusFieldMask) != 0u)
        {
            sawFault = true;
            hal.DelayMicroseconds(RetryDelayMicroseconds);
            continue;
        }

        sample.Angle = angle;
        sample.IsButtonPressed = (status & StatusButtonMask) != 0u;
        return Ok;
    }

    return sawFault ? ErrFault : ErrNoValidFrame;
}