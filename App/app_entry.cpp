/**
 * @file app_entry.cpp
 * @brief Firmware entry glue: bridges the CubeMX-generated main.c (C) to
 *        the C++ application logic (Mt6701, GearDecoder, PositionRegister).
 *
 * Owns the firmware's application state: the latest successful angle
 * sample per encoder role and the absolute-position decoder.  The I2C
 * register map (positionRegister, i2c_pos.cpp) is shared with the I2C
 * transport in hal_stm32.cpp.
 */
#include "app_entry.h"

#include "hal.h"
#include "mt6701.h"
#include "gear_decode.h"
#include "i2c_pos.h"

/* Latest successful angle sample per role; a failed read keeps the field
 * from the previous successful one (see AppProcessSample). */
static GearAngles _sampleAngles;

static GearDecoder _gearDecoder;

extern "C" int AppInit(void)
{
    hal.Init();
    if (!_gearDecoder.IsInitialized())
    {
        return 0; /* gear config invalid, see App/gear_config.h */
    }
    return 1;
}

extern "C" void AppProcessSample(void)
{
    Mt6701Sample sample;
    uint8_t readHealth = 0u; /* bit per role, see PositionRegister::HealthBit */

    if (Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == 0)
    {
        _sampleAngles.Sun = sample.Angle;
    }
    else
    {
        readHealth |= PositionRegister::HealthBit(EncoderRole::Sun);
    }
    if (Mt6701::ReadSample(hal, EncoderRole::Gear1, sample) == 0)
    {
        _sampleAngles.Gear1 = sample.Angle;
    }
    else
    {
        readHealth |= PositionRegister::HealthBit(EncoderRole::Gear1);
    }
    if (Mt6701::ReadSample(hal, EncoderRole::Gear2, sample) == 0)
    {
        _sampleAngles.Gear2 = sample.Angle;
    }
    else
    {
        readHealth |= PositionRegister::HealthBit(EncoderRole::Gear2);
    }
    if (Mt6701::ReadSample(hal, EncoderRole::Gear3, sample) == 0)
    {
        _sampleAngles.Gear3 = sample.Angle;
    }
    else
    {
        readHealth |= PositionRegister::HealthBit(EncoderRole::Gear3);
    }

    GearPosition position = _gearDecoder.Decode(_sampleAngles);
    positionRegister.Update(position, readHealth); /* publish to the I2C
                                                    * slave register map */
}