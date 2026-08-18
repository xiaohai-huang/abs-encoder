/**
 * @file hal_sim.c
 * @brief PC backend: SPI bridged to the simulated MT6701 chips (one per
 *        encoder index), QPC clock, nvs.bin file as persistent storage.
 *        Drop-in replacement for hal_stm32.c so the logic runs unmodified
 *        on the host.
 */
#include "hal.h"

#include "mt6701_slave_sim.h"

#include <stdio.h>
#include <windows.h>

#define SIM_NVS_FILE "nvs.bin"

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

static bool nvs_read(uint32_t addr, void *buf, uint32_t len)
{
    FILE *f = fopen(SIM_NVS_FILE, "rb");
    if (f == NULL)
    {
        return false;
    }
    if (fseek(f, (long)addr, SEEK_SET) != 0)
    {
        fclose(f);
        return false;
    }
    size_t got = fread(buf, 1, len, f);
    fclose(f);
    return got == len;
}

static bool nvs_write(uint32_t addr, const void *buf, uint32_t len)
{
    FILE *f = fopen(SIM_NVS_FILE, "r+b");
    if (f == NULL)
    {
        f = fopen(SIM_NVS_FILE, "wb"); /* create on first use */
    }
    if (f == NULL)
    {
        return false;
    }
    if (fseek(f, (long)addr, SEEK_SET) != 0)
    {
        fclose(f);
        return false;
    }
    size_t put = fwrite(buf, 1, len, f);
    fclose(f);
    return put == len;
}

const app_hal_t app_hal = {
    .init = init,
    .spi_transfer = spi_transfer,
    .spi_cs = spi_cs,
    .now_us = now_us,
    .delay_us = delay_us,
    .nvs_read = nvs_read,
    .nvs_write = nvs_write,
};