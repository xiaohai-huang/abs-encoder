/**
 * @file hal_stm32.c
 * @brief STM32F103 backend: SPI1 + SPI2 (four MT6701 on two buses), DWT
 *        microsecond clock, last flash page as NVS (one slot per encoder).
 *        The only firmware file that touches STM32 HAL registers.
 */
#include "hal.h"

#include "main.h"    /* CSN1..CSN4 pin defines, SystemCoreClock */
#include "mt6701.h"  /* MT6701_ENC_COUNT */
#include "spi.h"     /* hspi1, hspi2 */
#include "stm32f1xx_hal.h"

#include <string.h>

#define NVS_FLASH_ADDR 0x0800FC00u /* last 1 KiB page of the 64 KiB flash */
#define NVS_PAGE_SIZE  1024u

/* encoders 0,1 on SPI1 (CSN1 = PA3, CSN2 = PA4); 2,3 on SPI2
 * (CSN3 = PA9, CSN4 = PA8) */
static const struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} s_csn[MT6701_ENC_COUNT] = {
    {CSN1_GPIO_Port, CSN1_Pin},
    {CSN2_GPIO_Port, CSN2_Pin},
    {CSN3_GPIO_Port, CSN3_Pin},
    {CSN4_GPIO_Port, CSN4_Pin},
};

static void init(void)
{
    /* CubeMX generates the CSN pins asserted (low); deselect every encoder
     * before the first SPI frame (the .ioc asks for initial HIGH, but this
     * must hold regardless of regeneration). */
    for (uint8_t enc = 0; enc < MT6701_ENC_COUNT; enc++)
    {
        HAL_GPIO_WritePin(s_csn[enc].port, s_csn[enc].pin, GPIO_PIN_SET);
    }

    /* DWT cycle counter as free-running microsecond clock. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t spi_transfer(uint8_t enc, uint8_t tx)
{
    uint8_t rx = 0u;
    SPI_HandleTypeDef *hspi = (enc < 2u) ? &hspi1 : &hspi2;
    if (HAL_SPI_TransmitReceive(hspi, &tx, &rx, 1u, 10u) != HAL_OK)
    {
        return 0xFFu;
    }
    return rx;
}

static void spi_cs(uint8_t enc, bool asserted)
{
    if (enc >= MT6701_ENC_COUNT)
    {
        return;
    }
    HAL_GPIO_WritePin(s_csn[enc].port, s_csn[enc].pin,
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

/* NVS lives in the last flash page; the per-encoder records share it, so a
 * write preserves the other records (read-modify-write of the whole page:
 * copy to scratch, erase, reprogram). */
static uint32_t s_nvs_scratch[NVS_PAGE_SIZE / 4u];

static bool nvs_read(uint32_t addr, void *buf, uint32_t len)
{
    if (addr + len > NVS_PAGE_SIZE)
    {
        return false;
    }
    memcpy(buf, (const void *)(NVS_FLASH_ADDR + addr), len);
    return true;
}

static bool nvs_write(uint32_t addr, const void *buf, uint32_t len)
{
    if (addr + len > NVS_PAGE_SIZE)
    {
        return false;
    }

    memcpy(s_nvs_scratch, (const void *)NVS_FLASH_ADDR, NVS_PAGE_SIZE);
    memcpy((uint8_t *)s_nvs_scratch + addr, buf, len);

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
        for (uint32_t i = 0u; i < NVS_PAGE_SIZE / 4u; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  NVS_FLASH_ADDR + i * 4u,
                                  s_nvs_scratch[i]) != HAL_OK)
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