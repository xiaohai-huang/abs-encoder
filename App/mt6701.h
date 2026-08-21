/**
 * @file mt6701.h
 * @brief MT6701 (MagnTek) SSI protocol layer.
 *
 * The sensor speaks a 24-bit frame (3 bytes, MSB first; the datasheet shows
 * CLK idling high, but mode 1 -- CPOL=0, CPHA=1 -- is what the working
 * reference drivers use and matches the CubeMX SPI1/SPI2 config):
 *   bits 23..10  14-bit angle (0..16383 = 0..360 deg)
 *   bit  9       track loss (Mg[3])
 *   bit  8       push-magnet (Mg[2])
 *   bits 7..6    field status (Mg[1:0], 0 = normal)
 *   bits 5..0    CRC-6 (see Mt6701::ComputeCrc6)
 *
 * Every call is indexed by encoder role (EncoderRole in gear_config.h):
 * Sun is the input shaft, Gear1..3 the driven gears.  On this board
 * Gear1/2 share SPI1 (CSN1/CSN2) and Sun/Gear3 share SPI2 (CSN4/CSN3);
 * the role maps to bus and CS in the HAL backends (wiring table in
 * hal_stm32.cpp).  Only this file and the simulated chips
 * (sim/mt6701_slave_sim.cpp) know the frame format; everything else
 * consumes a resolved 14-bit angle.
 */
#ifndef APP_MT6701_H
#define APP_MT6701_H

#include <cstdint>

#include "gear_config.h" /* EncoderRole: which wheel each MT6701 reads */

class Hal; /* platform backend, defined in hal.h; the MT6701 protocol only
              needs the SPI/clock/delay surface at call time */

/* CRC-6 (X^6+X+1) validation per datasheet Rev 1.8 SSI section; kept as a
 * preprocessor toggle (not a constexpr) so a build can override it with
 * -DMt6701Crc6Enabled=0 (documented in the sim Makefile and AGENTS.md). */
#ifndef Mt6701Crc6Enabled
#define Mt6701Crc6Enabled 1
#endif

struct Mt6701Sample
{
    uint16_t Angle;           /* 0..16383 */
    bool     IsButtonPressed; /* push-magnet state */
};

/** MT6701 SSI protocol layer; stateless, all calls are static. */
class Mt6701
{
public:
    static constexpr uint16_t AngleMax     = 16383u;
    static constexpr uint16_t EncoderCount = 4u; /* 0,1 on SPI1; 2,3 on SPI2 */
    static constexpr int      BytesInFrame = 3;  /* one 24-bit SSI frame */

    /**
     * Read one angle frame with status validation and retries.
     * @param hal     platform backend (SPI + clock + delay)
     * @param encoder encoder role (Sun, Gear1..3)
     * @param sample  receives the resolved angle and button state
     * @return 0 on success, -1 if the chip reports a fault (track loss /
     *         bad field status), -2 if no valid frame arrived in time.
     */
    static int ReadSample(Hal& hal, EncoderRole encoder, Mt6701Sample& sample);

    /** Raw 24-bit frame access without validation (diagnostics, tests). */
    static int ReadFrame(Hal& hal, EncoderRole encoder, uint8_t frame[3]);

    /**
     * CRC-6 over the 18 data bits (angle + status), returned in the low
     * 6 bits.  Polynomial X^6+X+1 (0x43), initial value 0, no final XOR,
     * MSB first -- per datasheet Rev 1.8, SSI section.  Used when
     * Mt6701Crc6Enabled is 1.
     */
    static uint8_t ComputeCrc6(const uint8_t frame[3]);

    static constexpr int Ok             = 0;
    static constexpr int ErrFault       = -1; /* chip reported track loss / bad field */
    static constexpr int ErrNoValidFrame = -2; /* bad role, or retries exhausted */

private:
    static constexpr int  MaxReadRetries         = 3;
    static constexpr int  RetryDelayMicroseconds = 10;
    static constexpr uint8_t Crc6Polynomial      = 0x43u;

    /* 24-bit SSI frame field layout (MSB first, per file header):
     *   bits 23..10 angle, bits 9..6 status, bits 5..0 CRC-6.
     * Field shifts point at each field's low bit; masks are (1<<bits)-1. */
    static constexpr int  BitsInByte       = 8;
    static constexpr int  AngleFieldBits   = 14;
    static constexpr int  StatusFieldBits  = 4;
    static constexpr int  CrcFieldBits     = 6;
    static constexpr int  AngleFieldShift  = StatusFieldBits + CrcFieldBits;
    static constexpr int  StatusFieldShift = CrcFieldBits;
    static constexpr int  CrcTopBitShift   = CrcFieldBits - 1;
    static constexpr uint16_t AngleFieldMask =
        static_cast<uint16_t>((1u << AngleFieldBits) - 1u);
    static constexpr uint8_t RawStatusMask =
        static_cast<uint8_t>((1u << StatusFieldBits) - 1u);
    static constexpr uint8_t CrcFieldMask =
        static_cast<uint8_t>((1u << CrcFieldBits) - 1u);

    /* bits of the resolved status byte (frame bits 9..6 shifted down). */
    static constexpr uint8_t StatusTrackLossMask = 0x08u; /* bit 3 */
    static constexpr uint8_t StatusButtonMask    = 0x04u; /* bit 2 */
    static constexpr uint8_t StatusFieldMask     = 0x03u; /* bits 1..0 */

    static uint8_t UpdateCrc6(uint8_t crc, uint8_t frameByte, int bitCount);
};

#endif /* APP_MT6701_H */