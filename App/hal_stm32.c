/**
 * @file hal_stm32.c
 * @brief STM32F103 backend: SPI1 + SPI2 (four MT6701 on two buses) and a
 *        DWT microsecond clock.  The only firmware file that touches STM32
 *        HAL registers.
 */
#include "hal.h"

#include "main.h"    /* CSN1..CSN4 pin defines, SystemCoreClock */
#include "mt6701.h"  /* MT6701_ENC_COUNT */
#include "spi.h"     /* hspi1, hspi2 */
#include "stm32f1xx_hal.h"

/* encoders 0,1 on SPI1 (CSN1, CSN2); 2,3 on SPI2 (CSN3, CSN4).
 * Pin values come from the CubeMX User Labels (main.h). */
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
    /* The .ioc asks for the CSN pins to come up HIGH (deselected); enforce it
     * here so a future regeneration can't leave every encoder selected. */
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

const app_hal_t app_hal = {
    .init = init,
    .spi_transfer = spi_transfer,
    .spi_cs = spi_cs,
    .now_us = now_us,
    .delay_us = delay_us,
};