/**
 * @file sim_main.cpp
 * @brief PC test harness: drives the same protocol logic used by the
 *        firmware against simulated MT6701 chips (one per encoder role,
 *        two SPI buses).
 *
 * Exit code 0 = all checks passed; run from sim/ (make run).
 */
#include <cstdio>
#include <cstring>

#include "hal.h"
#include "mt6701.h"
#include "mt6701_slave_sim.h"
#include "gear_decode.h"
#include "i2c_pos.h"

static int _totalChecks = 0;
static int _failedChecks = 0;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        _totalChecks++;                                                      \
        if (!(cond))                                                         \
        {                                                                    \
            _failedChecks++;                                                 \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                    \
    } while (0)

static void TestStaticAngle()
{
    std::printf("static angle reads (encoders 0, 1, 3)\n");
    const EncoderRole roles[] = {EncoderRole::Sun, EncoderRole::Gear1,
                                 EncoderRole::Gear3};
    const uint16_t testAngles[] = {0u, 1u, 0x1234u, Mt6701::AngleMax};
    for (size_t roleIndex = 0u;
         roleIndex < sizeof(roles) / sizeof(roles[0]); roleIndex++)
    {
        for (size_t angleIndex = 0u;
             angleIndex < sizeof(testAngles) / sizeof(testAngles[0]);
             angleIndex++)
        {
            Mt6701SlaveSim::SetAngle(roles[roleIndex], testAngles[angleIndex]);
            Mt6701Sample sample;
            CHECK(Mt6701::ReadSample(hal, roles[roleIndex], sample) == 0);
            CHECK(sample.Angle == testAngles[angleIndex]);
            CHECK(!sample.IsButtonPressed);
        }
    }
}

static void TestEncoderIndependence()
{
    std::printf("four encoders read independently, no crosstalk\n");
    const uint16_t expectedAngles[Mt6701::EncoderCount] = {
        0x1111u, 0x2222u, 0x3333u, 0x0FF0u};
    for (int roleIndex = 0; roleIndex < Mt6701::EncoderCount; roleIndex++)
    {
        Mt6701SlaveSim::SetAngle(static_cast<EncoderRole>(roleIndex),
                                 expectedAngles[roleIndex]);
    }

    Mt6701Sample sample;
    for (int roleIndex = 0; roleIndex < Mt6701::EncoderCount; roleIndex++)
    {
        EncoderRole role = static_cast<EncoderRole>(roleIndex);
        CHECK(Mt6701::ReadSample(hal, role, sample) == 0);
        CHECK(sample.Angle == expectedAngles[roleIndex]);
        CHECK(!sample.IsButtonPressed);
    }

    /* re-reading one encoder must not disturb the others' state */
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == 0);
    CHECK(sample.Angle == expectedAngles[0]);
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear2, sample) == 0);
    CHECK(sample.Angle == expectedAngles[2]);
}

static void TestButtonFlag()
{
    std::printf("button flag passthrough (encoder 3)\n");
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear3, 0x555u);
    Mt6701SlaveSim::SetButton(EncoderRole::Gear3, true);
    Mt6701Sample sample;
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear3, sample) == 0);
    CHECK(sample.IsButtonPressed);
    CHECK(sample.Angle == 0x555u);
    Mt6701SlaveSim::SetButton(EncoderRole::Gear3, false);
}

static void TestFaults()
{
    std::printf("fault handling: track loss, bad field, stuck chip\n");
    Mt6701SlaveSim::SetAngle(EncoderRole::Sun, 1000u);
    Mt6701Sample sample;

    Mt6701SlaveSim::SetTrackLoss(EncoderRole::Sun, true);
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == -1);
    Mt6701SlaveSim::SetTrackLoss(EncoderRole::Sun, false);

    Mt6701SlaveSim::SetFieldStatus(EncoderRole::Sun, 1u);
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == -1);
    Mt6701SlaveSim::SetFieldStatus(EncoderRole::Sun, 0u);

    Mt6701SlaveSim::SetStuck(EncoderRole::Sun, true);
#if Mt6701Crc6Enabled
    /* 0xFF fails the CRC-6 check */
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == -2);
#else
    /* 0xFF decodes as track loss */
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == -1);
#endif
    Mt6701SlaveSim::SetStuck(EncoderRole::Sun, false);

    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == 0); /* recovered */

    /* same fault path on the second bus */
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear3, true);
#if Mt6701Crc6Enabled
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear3, sample) == -2);
#else
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear3, sample) == -1);
#endif
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear3, false);
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear3, sample) == 0);
}

static void TestCrc()
{
    std::printf("crc-6 validation (encoder 1)\n");
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear1, 0x1234u);
    Mt6701SlaveSim::SetCrcBroken(EncoderRole::Gear1, true);
    Mt6701Sample sample;
#if Mt6701Crc6Enabled
    /* all retries fail the CRC */
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear1, sample) == -2);
#else
    /* CRC check disabled (default) */
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear1, sample) == 0);
#endif
    Mt6701SlaveSim::SetCrcBroken(EncoderRole::Gear1, false);
}

/* Angle of driven gear `gearTeeth` after `turns` input turns */
static uint16_t ComputeGearAngle(uint32_t turns, uint32_t gearTeeth)
{
    uint32_t phase = (GearConfig::InputTeeth * turns) % gearTeeth;
    return static_cast<uint16_t>(
        (phase * 16384u + gearTeeth / 2u) / gearTeeth);
}

static void TestGearDecodeReadPath()
{
    std::printf("gear decode via the real SSI read path (roles Sun..Gear3)\n");
    GearDecoder decoder;

    Mt6701SlaveSim::SetAngle(EncoderRole::Sun, 333u);
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear1,
                             ComputeGearAngle(4000u, GearConfig::DrivenTeethCounts[0]));
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear2,
                             ComputeGearAngle(4000u, GearConfig::DrivenTeethCounts[1]));
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear3,
                             ComputeGearAngle(4000u, GearConfig::DrivenTeethCounts[2]));

    GearAngles angles;
    Mt6701Sample sample;
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == 0);
    angles.Sun = sample.Angle;
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear1, sample) == 0);
    angles.Gear1 = sample.Angle;
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear2, sample) == 0);
    angles.Gear2 = sample.Angle;
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear3, sample) == 0);
    angles.Gear3 = sample.Angle;

    GearPosition position = decoder.Decode(angles);
    CHECK(position.IsValid);
    CHECK(position.Turns == 4000u);
}

static void TestI2cPosition()
{
    std::printf("i2c position register map (App/i2c_pos.cpp)\n");
    positionRegister.Init();
    uint8_t bytes[PositionRegister::RegisterCount];
    GearPosition position;

    /* origin sample: all bytes zero except the status valid bit */
    position.IsValid = true;
    position.Turns = 0u;
    position.Angle = 0u;
    positionRegister.Update(position);
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedOrigin[] = {0u, 0u, 0u, 0u,
                                      PositionRegister::StatusValid};
    CHECK(std::memcmp(bytes, expectedOrigin, sizeof(bytes)) == 0);

    /* one whole turn is exactly CountsPerTurn (0x4000), not trailing
     * zeroes of a separate turn field */
    position.IsValid = true;
    position.Turns = 1u;
    position.Angle = 0u;
    positionRegister.Update(position);
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedOneTurn[] = {0x00u, 0x40u, 0x00u, 0x00u,
                                       PositionRegister::StatusValid};
    CHECK(std::memcmp(bytes, expectedOneTurn, sizeof(bytes)) == 0);

    /* the extreme position is one continuous 27-bit count:
     * 7428 * 16384 + 16383 = 121,716,735 = 0x07413FFF */
    position.IsValid = true;
    position.Turns = GearConfig::TurnRange - 1u;
    position.Angle = Mt6701::AngleMax;
    positionRegister.Update(position);
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedExtreme[] = {0xFFu, 0x3Fu, 0x41u, 0x07u,
                                       PositionRegister::StatusValid};
    CHECK(std::memcmp(bytes, expectedExtreme, sizeof(bytes)) == 0);

    /* a slew-guard rejection keeps the position bytes but clears status;
     * 1234 turns + 0x1234 angle = 20,222,516 = 0x01349234 */
    position.IsValid = false;
    position.Turns = 1234u;
    position.Angle = 0x1234u;
    positionRegister.Update(position);
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedRejected[] = {0x34u, 0x92u, 0x34u, 0x01u, 0x00u};
    CHECK(std::memcmp(bytes, expectedRejected, sizeof(bytes)) == 0);

    /* auto-increment across separate read calls, one byte at a time */
    positionRegister.Select(2u);
    CHECK(positionRegister.Read(bytes, 1u) == 1u && bytes[0] == 0x34u);
    CHECK(positionRegister.Read(bytes, 1u) == 1u && bytes[0] == 0x01u);
    CHECK(positionRegister.Read(bytes, 1u) == 1u && bytes[0] == 0u); /* status */
    CHECK(positionRegister.Read(bytes, 1u) == 1u && bytes[0] == 0u); /* past map */
    CHECK(positionRegister.Read(bytes, 1u) == 1u && bytes[0] == 0u); /* stays 0 */

    /* a pointer past the map reads zeros too */
    positionRegister.Select(0xFFu);
    CHECK(positionRegister.Read(bytes, 4u) == 4u);
    const uint8_t expectedZeros4[] = {0u, 0u, 0u, 0u};
    CHECK(std::memcmp(bytes, expectedZeros4, 4u) == 0);

    /* refreshing the snapshot does not disturb the cursor */
    positionRegister.Select(3u);
    position.IsValid = true;
    position.Turns = 5u;
    position.Angle = 9u;
    positionRegister.Update(position); /* count 81,929 = 0x014009 */
    CHECK(positionRegister.Read(bytes, 1u) == 1u && bytes[0] == 0x00u);
    CHECK(positionRegister.Read(bytes, 1u) == 1u &&
          bytes[0] == PositionRegister::StatusValid);

    /* window through the middle of the map (pointer + partial read) */
    positionRegister.Select(1u);
    CHECK(positionRegister.Read(bytes, 2u) == 2u);
    const uint8_t expectedWindow[] = {0x40u, 0x01u};
    CHECK(std::memcmp(bytes, expectedWindow, 2u) == 0); /* count bytes 1..2 */
}

int main()
{
    hal.Init();

    TestStaticAngle();
    TestEncoderIndependence();
    TestButtonFlag();
    TestFaults();
    TestCrc();
    TestGearDecodeReadPath();
    TestI2cPosition();

    std::printf("%d/%d checks passed\n", _totalChecks - _failedChecks,
                _totalChecks);
    return _failedChecks != 0 ? 1 : 0;
}