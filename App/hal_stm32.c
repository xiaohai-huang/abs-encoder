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
/* Explicit wiring, keyed by encoder role (see the role table in
 * gear_config.h); each row binds one CSN label and one SPI bus to the
 * wheel it senses.  The labels come from the CubeMX User Labels
 * (main.h) and the rows must match the physical board:
 *
 *   ENC_SUN    <- CSN4 on SPI2   (sun gear, 13T)
 *   ENC_GEAR_1 <- CSN1 on SPI1   (driven gear, 17T)
 *   ENC_GEAR_2 <- CSN2 on SPI1   (driven gear, 19T)
 *   ENC_GEAR_3 <- CSN3 on SPI2   (driven gear, 23T)
 */
static const struct
{
    GPIO_TypeDef     *port;
    uint16_t          pin;
    SPI_HandleTypeDef *spi;
} s_enc[ENC_COUNT] = {
    [ENC_SUN]    = {CSN4_GPIO_Port, CSN4_Pin, &hspi2},
    [ENC_GEAR_1] = {CSN1_GPIO_Port, CSN1_Pin, &hspi1},
    [ENC_GEAR_2] = {CSN2_GPIO_Port, CSN2_Pin, &hspi1},
    [ENC_GEAR_3] = {CSN3_GPIO_Port, CSN3_Pin, &hspi2},
};

static void init(void)
{
    /* The .ioc asks for the CSN pins to come up HIGH (deselected); enforce it
     * here so a future regeneration can't leave every encoder selected. */
    for (encoder_role_t enc = ENC_SUN; enc < ENC_COUNT; enc++)
    {
        HAL_GPIO_WritePin(s_enc[enc].port, s_enc[enc].pin, GPIO_PIN_SET);
    }

    /* DWT cycle counter as free-running microsecond clock. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t spi_transfer(encoder_role_t enc, uint8_t tx)
{
    uint8_t rx = 0u;
    if (enc >= ENC_COUNT)
    {
        return 0xFFu;
    }
    if (HAL_SPI_TransmitReceive(s_enc[enc].spi, &tx, &rx, 1u, 10u) != HAL_OK)
    {
        return 0xFFu;
    }
    return rx;
}

static void spi_cs(encoder_role_t enc, bool asserted)
{
    if (enc >= ENC_COUNT)
    {
        return;
    }
    HAL_GPIO_WritePin(s_enc[enc].port, s_enc[enc].pin,
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