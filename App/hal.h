/**
 * @file hal.h
 * @brief Hardware abstraction interface (grblHAL-style).
 *
 * All application logic (Mt6701) depends only on this header and never
 * touches MCU registers or STM32 HAL headers.  The SPI byte exchange and
 * chip-select calls are indexed by encoder (0..Mt6701::EncoderCount-1);
 * the platform maps each encoder to a bus and a CS line.  Exactly one
 * backend is linked per build, providing the `hal` instance:
 *   - Stm32Hal (hal_stm32.cpp) -- firmware (EIDE Debug/Release targets)
 *   - SimHal   (hal_sim.cpp)   -- PC test build (sim/Makefile)
 */
#ifndef APP_HAL_H
#define APP_HAL_H

#include <cstdint>

#include "gear_config.h" /* EncoderRole */

class Hal
{
public:
    virtual ~Hal() = default;

    /** One-time setup; call after clocks and peripherals are running. */
    virtual void Init() = 0;

    /** Full-duplex byte exchange with the given encoder, MSB first. */
    virtual uint8_t SpiTransfer(EncoderRole encoder, uint8_t txByte) = 0;

    /** Chip-select line of the given encoder (asserted = active low). */
    virtual void SelectChip(EncoderRole encoder, bool asserted) = 0;

    /** Monotonic microseconds since init (wrap-safe on arithmetic). */
    virtual uint32_t GetMicroseconds() = 0;

    /** Busy-wait delay. */
    virtual void DelayMicroseconds(uint32_t microseconds) = 0;
};

/** The backend instance; exactly one backend is linked per build. */
extern Hal& hal;

#endif /* APP_HAL_H */