/**
 * @file hal_stm32.c
 * @brief STM32F103 backend: SPI1 + SPI2 (four MT6701 on two buses), a
 *        DWT microsecond clock and the I2C1 slave transport that serves
 *        the position register map (App/i2c_pos.h, docs/i2c.md).  The only
 *        firmware file that touches STM32 HAL registers.
 */
#include "hal.h"

#include "main.h"    /* CSN1..CSN4 pin defines, SystemCoreClock */
#include "mt6701.h"  /* MT6701_ENC_COUNT */
#include "i2c_pos.h" /* position register map, I2C_POS_ADDR */
#include "i2c.h"     /* hi2c1 */
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

/* I2C1 slave transport for the position register map (docs/i2c.md): the
 * master's transactions are served byte-by-byte from these buffers; every
 * address-match event copies a fresh snapshot out of App/i2c_pos.c, so a
 * 1 kHz update in the main loop can never tear a transaction in flight. */
static uint8_t s_tx[I2C_POS_REG_COUNT];
static uint8_t s_rx;

static void i2c_slave_start(void)
{
    /* MX_I2C1_Init leaves OwnAddress1 = 0 (no slave role possible); apply
     * the encoder address and re-init so OAR1 matches.  Done here, not in
     * i2c.c, so a CubeMX regeneration can't lose the address.  F1 OAR1
     * holds a 7-bit address shifted left by one. */
    hi2c1.Init.OwnAddress1 = (uint16_t)(I2C_POS_ADDR << 1u);
    HAL_I2C_Init(&hi2c1);

    /* The transport runs on the I2C1 EV/ER interrupts; TIM2 keeps preempt
     * priority 0 so the 1 kHz sample tick always wins over byte traffic. */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

    i2c_pos_init();
    HAL_I2C_EnableListen_IT(&hi2c1);
}

/* Master wants to read: serve the register map from the current pointer
 * (HAL passes the master's direction: RECEIVE = master receives = slave
 * transmits).  The transfer count is fixed to the rest of the map; a master
 * that stops early simply ends the transaction, which the listen-complete
 * and error callbacks below clean up. */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }
    if (dir == I2C_DIRECTION_RECEIVE)
    {
        uint8_t n = i2c_pos_read(s_tx, (uint8_t)sizeof(s_tx));
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, s_tx, n, I2C_FIRST_AND_LAST_FRAME);
    }
    else
    {
        /* master writes: exactly one register-pointer byte */
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &s_rx, 1u, I2C_FIRST_AND_LAST_FRAME);
    }
}

/* The pointer byte arrived (before the master's STOP, so a repeated-start
 * read right after it already points at the right register). */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2c_pos_select(s_rx);
    }
}

/* The HAL parks the peripheral in READY once a transaction ends (master
 * NACKs the last byte or issues STOP); re-arm for the next address match. */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        HAL_I2C_EnableListen_IT(hi2c);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF | I2C_FLAG_BERR |
                                       I2C_FLAG_ARLO | I2C_FLAG_OVR);
        HAL_I2C_EnableListen_IT(hi2c);
    }
}

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

    i2c_slave_start();
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