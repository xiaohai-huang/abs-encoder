/**
 * @file hal.h
 * @brief Hardware abstraction interface (grblHAL-style).
 *
 * All application logic (mt6701.c, multi_turn.c) depends only on this
 * header and never touches MCU registers or STM32 HAL headers.  The SPI
 * byte exchange and chip-select calls are indexed by encoder
 * (0..MT6701_ENC_COUNT-1); the platform maps each encoder to a bus and a
 * CS line.  Exactly one implementation is linked per build:
 *   - hal_stm32.c  -- firmware (EIDE Debug/Release targets), 2 buses
 *   - hal_sim.c    -- PC test build (sim/Makefile), one chip per encoder
 */
#ifndef APP_HAL_H
#define APP_HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /** One-time setup; call after clocks and peripherals are running. */
    void (*init)(void);

    /** Full-duplex byte exchange with the given encoder, MSB first. */
    uint8_t (*spi_transfer)(uint8_t enc, uint8_t tx);

    /** Chip-select line of the given encoder (asserted = active low). */
    void (*spi_cs)(uint8_t enc, bool asserted);

    /** Monotonic microseconds since init. */
    uint32_t (*now_us)(void);

    /** Busy-wait delay. */
    void (*delay_us)(uint32_t us);

    /** Non-volatile storage with byte granularity; one slot per encoder. */
    bool (*nvs_read)(uint32_t addr, void *buf, uint32_t len);
    bool (*nvs_write)(uint32_t addr, const void *buf, uint32_t len);
} app_hal_t;

extern const app_hal_t app_hal;

#endif /* APP_HAL_H */