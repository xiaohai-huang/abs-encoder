/**
 * @file hal_stm32.c
 * @brief STM32F103 backend: SPI1 (MT6701 SSI), DWT microsecond clock,
 *        last flash page as NVS.  The only firmware file that touches
 *        STM32 HAL registers.
 */
#include "hal.h"

#include "main.h" /* MT6701_CSN pin, SystemCoreClock */
#include "spi.h"  /* hspi1 */
#include "stm32f1xx_hal.h"

#include <string.h>

#define NVS_FLASH_ADDR 0x0800FC00u /* last 1 KiB page of the 64 KiB flash */

static void init(void)
{
    /* DWT cycle counter as free-running microsecond clock. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t spi_transfer(uint8_t tx)
{
    uint8_t rx = 0u;
    if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1u, 10u) != HAL_OK)
    {
        return 0xFFu;
    }
    return rx;
}

static void spi_cs(bool asserted)
{
    HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin,
                      asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint32_t now_us(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

static void delay_us(uint32_t us)
{
    uint32_t start = now_us();
    while ((uint32_t)(now_us() - start) < us)
    {
    }
}

static bool nvs_read(uint32_t addr, void *buf, uint32_t len)
{
    if (addr != 0u)
    {
        return false;
    }
    memcpy(buf, (const void *)NVS_FLASH_ADDR, len);
    return true;
}

static bool nvs_write(uint32_t addr, const void *buf, uint32_t len)
{
    uint32_t words[4]; /* 16-byte staging: flash programs 32-bit words */
    if (addr != 0u || len > sizeof(words))
    {
        return false;
    }

    memcpy(words, buf, len);

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = NVS_FLASH_ADDR,
        .NbPages = 1,
    };
    uint32_t page_error = 0u;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);

    if (status == HAL_OK)
    {
        for (uint32_t i = 0u; i < (len + 3u) / 4u; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  NVS_FLASH_ADDR + i * 4u, words[i]) != HAL_OK)
            {
                status = HAL_ERROR;
                break;
            }
        }
    }
    HAL_FLASH_Lock();

    return status == HAL_OK;
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
