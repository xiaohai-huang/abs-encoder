/**
 * @file hal.h
 * @brief Hardware abstraction interface (grblHAL-style).
 *
 * All application logic (mt6701.c, multi_turn.c) depends only on this
 * header and never touches MCU registers or STM32 HAL headers.  Exactly
 * one implementation is linked per build:
 *   - hal_stm32.c  -- firmware (EIDE Debug/Release targets)
 *   - hal_sim.c    -- PC test build (sim/Makefile)
 */
#ifndef APP_HAL_H
#define APP_HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /** One-time setup; call after clocks and peripherals are running. */
    void (*init)(void);

    /** Full-duplex byte exchange with the encoder chip, MSB first. */
    uint8_t (*spi_transfer)(uint8_t tx);

    /** Encoder chip-select line (MT6701_CSN). */
    void (*spi_cs)(bool asserted);

    /** Monotonic microseconds since init. */
    uint32_t (*now_us)(void);

    /** Busy-wait delay. */
    void (*delay_us)(uint32_t us);

    /** Non-volatile storage with byte granularity (multi-turn count). */
    bool (*nvs_read)(uint32_t addr, void *buf, uint32_t len);
    bool (*nvs_write)(uint32_t addr, const void *buf, uint32_t len);
} app_hal_t;

extern const app_hal_t app_hal;

#endif /* APP_HAL_H */
