/**
 * @file hal_stm32.cpp
 * @brief STM32F103 backend: SPI1 + SPI2 (four MT6701 on two buses), a
 *        DWT microsecond clock and the I2C1 slave transport that serves
 *        the position register map (App/i2c_pos.h, docs/i2c.md).  The
 *        only firmware file that touches STM32 HAL registers.
 */
#include "hal.h"

#include "main.h"    /* CSN1..CSN4 pin defines, SystemCoreClock */
#include "i2c_pos.h" /* PositionRegister, positionRegister */
#include "i2c.h"     /* hi2c1 */
#include "spi.h"     /* hspi1, hspi2 */
#include "stm32f1xx_hal.h"

/* Explicit wiring, keyed by encoder role (see the role table in
 * gear_config.h); each row binds one CSN label and one SPI bus to the
 * wheel it senses.  The labels come from the CubeMX User Labels
 * (main.h) and the rows must match the physical board:
 *
 *   Sun    <- CSN4 on SPI2   (sun gear, 13T)
 *   Gear1  <- CSN1 on SPI1   (driven gear, 17T)
 *   Gear2  <- CSN2 on SPI1   (driven gear, 19T)
 *   Gear3  <- CSN3 on SPI2   (driven gear, 23T)
 *
 * The table is positional: EncoderRole order Sun, Gear1, Gear2, Gear3. */
struct EncoderWiring
{
    GPIO_TypeDef*      Port;
    uint16_t           Pin;
    SPI_HandleTypeDef* Spi;
};

static const EncoderWiring _encoderWiring[static_cast<size_t>(
    EncoderRole::RoleCount)] = {
    {CSN4_GPIO_Port, CSN4_Pin, &hspi2}, /* Sun   */
    {CSN1_GPIO_Port, CSN1_Pin, &hspi1}, /* Gear1 */
    {CSN2_GPIO_Port, CSN2_Pin, &hspi1}, /* Gear2 */
    {CSN3_GPIO_Port, CSN3_Pin, &hspi2}, /* Gear3 */
};

/* I2C1 slave transport for the position register map (docs/i2c.md): the
 * master's transactions are served byte-by-byte from these buffers; every
 * address-match event copies a fresh snapshot out of App/i2c_pos.cpp, so
 * a 1 kHz update in the sample loop can never tear a transaction in
 * flight. */
static uint8_t _txBuffer[PositionRegister::RegisterCount];
static uint8_t _rxByte;

static void StartI2cSlave()
{
    /* MX_I2C1_Init leaves OwnAddress1 = 0 (no slave role possible); apply
     * the encoder address and re-init so OAR1 matches.  Done here, not in
     * i2c.c, so a CubeMX regeneration can't lose the address.  F1 OAR1
     * holds a 7-bit address shifted left by one. */
    hi2c1.Init.OwnAddress1 =
        static_cast<uint16_t>(PositionRegister::Address << 1u);
    HAL_I2C_Init(&hi2c1);

    /* The transport runs on the I2C1 EV/ER interrupts; TIM2 keeps preempt
     * priority 0 so the 1 kHz sample tick always wins over byte traffic. */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

    positionRegister.Init();
    HAL_I2C_EnableListen_IT(&hi2c1);
}

class Stm32Hal : public Hal
{
public:
    void Init() override;
    uint8_t SpiTransfer(EncoderRole encoder, uint8_t txByte) override;
    void SelectChip(EncoderRole encoder, bool asserted) override;
    uint32_t GetMicroseconds() override;
    void DelayMicroseconds(uint32_t microseconds) override;
};

void Stm32Hal::Init()
{
    /* The .ioc asks for the CSN pins to come up HIGH (deselected); enforce it
     * here so a future regeneration can't leave every encoder selected. */
    for (int roleIndex = 0;
         roleIndex < static_cast<int>(EncoderRole::RoleCount); roleIndex++)
    {
        HAL_GPIO_WritePin(_encoderWiring[roleIndex].Port,
                          _encoderWiring[roleIndex].Pin, GPIO_PIN_SET);
    }

    /* DWT cycle counter as free-running microsecond clock. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    StartI2cSlave();
}

uint8_t Stm32Hal::SpiTransfer(EncoderRole encoder, uint8_t txByte)
{
    uint8_t rxByte = 0u;
    if (encoder >= EncoderRole::RoleCount)
    {
        return 0xFFu;
    }
    size_t roleIndex = static_cast<size_t>(encoder);
    if (HAL_SPI_TransmitReceive(_encoderWiring[roleIndex].Spi, &txByte,
                                &rxByte, 1u, 10u) != HAL_OK)
    {
        return 0xFFu;
    }
    return rxByte;
}

void Stm32Hal::SelectChip(EncoderRole encoder, bool asserted)
{
    if (encoder >= EncoderRole::RoleCount)
    {
        return;
    }
    size_t roleIndex = static_cast<size_t>(encoder);
    HAL_GPIO_WritePin(_encoderWiring[roleIndex].Port,
                      _encoderWiring[roleIndex].Pin,
                      asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint32_t Stm32Hal::GetMicroseconds()
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

void Stm32Hal::DelayMicroseconds(uint32_t microseconds)
{
    uint32_t startTime = GetMicroseconds();
    while ((uint32_t)(GetMicroseconds() - startTime) < microseconds)
    {
    }
}

/* The I2C1 callback definitions keep their prototypes' C linkage: the HAL
 * headers declare them inside extern "C", so these strong definitions
 * replace the __weak stubs in the HAL library. */
extern "C" void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t dir,
                                     uint16_t)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }
    if (dir == I2C_DIRECTION_RECEIVE)
    {
        /* Master wants to read: serve the register map from the current
         * pointer (HAL passes the master's direction: RECEIVE = master
         * receives = slave transmits).  The transfer count is fixed to the
         * rest of the map; a master that stops early simply ends the
         * transaction, which the listen-complete and error callbacks below
         * clean up. */
        uint8_t byteCount = positionRegister.Read(
            _txBuffer, static_cast<uint8_t>(sizeof(_txBuffer)));
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, _txBuffer, byteCount,
                                      I2C_FIRST_AND_LAST_FRAME);
    }
    else
    {
        /* master writes: exactly one register-pointer byte */
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &_rxByte, 1u,
                                     I2C_FIRST_AND_LAST_FRAME);
    }
}

extern "C" void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    /* The pointer byte arrived (before the master's STOP, so a
     * repeated-start read right after it already points at the right
     * register). */
    if (hi2c->Instance == I2C1)
    {
        positionRegister.Select(_rxByte);
    }
}

extern "C" void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c)
{
    /* The HAL parks the peripheral in READY once a transaction ends (master
     * NACKs the last byte or issues STOP); re-arm for the next address
     * match. */
    if (hi2c->Instance == I2C1)
    {
        HAL_I2C_EnableListen_IT(hi2c);
    }
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF | I2C_FLAG_BERR |
                                       I2C_FLAG_ARLO | I2C_FLAG_OVR);
        HAL_I2C_EnableListen_IT(hi2c);
    }
}

static Stm32Hal _stm32Hal;
Hal& hal = _stm32Hal;