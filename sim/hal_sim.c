/**
 * @file hal_sim.c
 * @brief PC backend: SPI bridged to the simulated MT6701 chips (one per
 *        encoder index), QPC clock.  Drop-in replacement for hal_stm32.c
 *        so the logic runs unmodified on the host.
 */
#include "hal.h"

#include "mt6701_slave_sim.h"

#include <windows.h>

static LARGE_INTEGER s_freq;

static void init(void)
{
    QueryPerformanceFrequency(&s_freq);
    mt6701_slave_init();
}

static uint8_t spi_transfer(uint8_t enc, uint8_t tx)
{
    return mt6701_slave_transfer(enc, tx);
}

static void spi_cs(uint8_t enc, bool asserted)
{
    mt6701_slave_cs(enc, asserted);
}

static uint32_t now_us(void)
{
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    return (uint32_t)((uint64_t)counter.QuadPart * 1000000ull / s_freq.QuadPart);
}

static void delay_us(uint32_t us)
{
    uint32_t end = now_us() + us;
    while ((int32_t)(now_us() - end) < 0)
    {
    }
}

const app_hal_t app_hal = {
    .init = init,
    .spi_transfer = spi_transfer,
    .spi_cs = spi_cs,
    .now_us = now_us,
    .delay_us = delay_us,
};