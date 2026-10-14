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

/**
 * Read one wheel over SSI and translate the outcome for the I2C map:
 * the angle lands in `angle` only when the read succeeded (a failed read
 * keeps the previous one), while `results[role]` always carries the cause
 * and the raw chip status -- magnetic fault (frame was intact, the chip
 * complained about the magnet) vs no valid frame (CRC-6 failed every
 * retry) -- so the diagnostics block can say why, not just that.
 */
static void ReadWheel(EncoderRole role, uint16_t& angle,
                      PositionRegister::ReadResult* results)
{
    Mt6701Sample sample;
    const int    readResult = Mt6701::ReadSample(hal, role, sample);
    PositionRegister::ReadResult& result =
        results[static_cast<uint8_t>(role)]; /* results run in role order */

    result.MgStatus = sample.Status;
    if (readResult == Mt6701::Ok)
    {
        angle = sample.Angle;
        result.Cause = PositionRegister::CauseOk;
    }
    else
    {
        result.Cause = (readResult == Mt6701::ErrFault)
                           ? PositionRegister::CauseMagnetic
                           : PositionRegister::CauseNoFrame;
    }
}

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
    PositionRegister::ReadResult results[PositionRegister::WheelCount];
    ReadWheel(EncoderRole::Sun, _sampleAngles.Sun, results);
    ReadWheel(EncoderRole::Gear1, _sampleAngles.Gear1, results);
    ReadWheel(EncoderRole::Gear2, _sampleAngles.Gear2, results);
    ReadWheel(EncoderRole::Gear3, _sampleAngles.Gear3, results);

    GearPosition position = _gearDecoder.Decode(_sampleAngles);
    positionRegister.Update(position, _sampleAngles,
                            results); /* publish to the I2C slave
                                       * register map: angles, health and
                                       * read diagnostics, same sample */
}