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
#include "app_entry.h" /* AppProcessSample: the firmware sample loop */

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

/* Bytes 0x07..0x0A of every full-map read: the constant full-scale count
 * 7429 * 16384 - 1 = 121,716,735 = 0x07413FFF, little-endian
 * (PositionRegister::MaxPosition, docs/i2c.md). */
#define EXPECTED_MAX_REG_BYTES 0xFFu, 0x3Fu, 0x41u, 0x07u

/* One 2-byte gear-angle register (base 0x0B, stride 2, one per
 * EncoderRole): the 14-bit fine angle that wheel's own MT6701 measured,
 * uint16 LE -- docs/i2c.md. */
#define GEAR_ANGLE(angle)                                                     \
    (uint8_t)((angle) & 0xFFu), (uint8_t)(((angle) >> 8u) & 0xFFu)

/* The four gear angles of the sample at 4000 turns + 333 counts of the
 * input (position 65,536,333 = 0x03E8014D), in EncoderRole order: the Sun
 * angle is the low 14 bits of that position, the driven wheels sit at their
 * exact phases at it (13/17, 13/19, 13/23 of an input turn). */
#define EXPECTED_ANGLES_HEALTHY                                               \
    GEAR_ANGLE(0x014Du), GEAR_ANGLE(0x35B3u), GEAR_ANGLE(0x36C9u),            \
        GEAR_ANGLE(0x3863u)

/* One raw angle sample per role, in EncoderRole order. */
static GearAngles MakeAngles(uint16_t sun, uint16_t gear1, uint16_t gear2,
                             uint16_t gear3)
{
    GearAngles angles;
    angles.Sun = sun;
    angles.Gear1 = gear1;
    angles.Gear2 = gear2;
    angles.Gear3 = gear3;
    return angles;
}

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
    }

    /* re-reading one encoder must not disturb the others' state */
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == 0);
    CHECK(sample.Angle == expectedAngles[0]);
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear2, sample) == 0);
    CHECK(sample.Angle == expectedAngles[2]);
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
    /* 0xFF fails the CRC-6 check */
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == -2);
    Mt6701SlaveSim::SetStuck(EncoderRole::Sun, false);

    CHECK(Mt6701::ReadSample(hal, EncoderRole::Sun, sample) == 0); /* recovered */

    /* same fault path on the second bus */
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear3, true);
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear3, sample) == -2);
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear3, false);
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear3, sample) == 0);
}

static void TestCrc()
{
    std::printf("crc-6 validation (encoder 1)\n");
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear1, 0x1234u);
    Mt6701SlaveSim::SetCrcBroken(EncoderRole::Gear1, true);
    Mt6701Sample sample;
    /* all retries fail the CRC */
    CHECK(Mt6701::ReadSample(hal, EncoderRole::Gear1, sample) == -2);
    Mt6701SlaveSim::SetCrcBroken(EncoderRole::Gear1, false);
}

/* Fine angle of driven gear `gearTeeth` when the input shaft stands at
 * `inputCounts` counts (turns * 16384 + angle): the phase relation the
 * decoder inverts, evaluated at the finest resolution the input reports --
 * what the simulated chips are set to so the train is physically real. */
static uint16_t ComputeGearAngle(uint32_t inputCounts, uint32_t gearTeeth)
{
    uint32_t wheelCounts =
        (GearConfig::InputTeeth * inputCounts) % (gearTeeth * 16384u);
    return static_cast<uint16_t>((wheelCounts + gearTeeth / 2u) / gearTeeth);
}

static void TestGearDecodeReadPath()
{
    std::printf("gear decode via the real SSI read path (roles Sun..Gear3)\n");
    GearDecoder decoder;

    /* a whole train at 4000 turns + 333 counts of the input: every wheel
       sits at its exact phase at that input position */
    const uint32_t inputCounts = 4000u * 16384u + 333u;
    Mt6701SlaveSim::SetAngle(EncoderRole::Sun, 333u);
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear1,
                             ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[0]));
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear2,
                             ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[1]));
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear3,
                             ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[2]));

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
    uint16_t expectedCounter = 0u; /* +1 per Update below */
    GearPosition position;

    /* the gear angles sit right behind the main map, one register per role,
       in EncoderRole order (docs/i2c.md) */
    CHECK(PositionRegister::RegisterGearAngle == 0x0Bu);
    CHECK(PositionRegister::RegisterCount == 19u);
    CHECK(PositionRegister::GearAngleRegister(EncoderRole::Sun) == 0x0Bu);
    CHECK(PositionRegister::GearAngleRegister(EncoderRole::Gear1) == 0x0Du);
    CHECK(PositionRegister::GearAngleRegister(EncoderRole::Gear2) == 0x0Fu);
    CHECK(PositionRegister::GearAngleRegister(EncoderRole::Gear3) == 0x11u);

    /* origin sample: position zero, status = valid, counter ticks; no wheel
       has moved, so the gear angles are all zero too */
    position.IsValid = true;
    position.Turns = 0u;
    position.Angle = 0u;
    positionRegister.Update(position, MakeAngles(0u, 0u, 0u, 0u), 0u);
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedOrigin[] = {
        0u, 0u, 0u, 0u, PositionRegister::StatusValid,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, GEAR_ANGLE(0u), GEAR_ANGLE(0u), GEAR_ANGLE(0u),
        GEAR_ANGLE(0u)};
    CHECK(std::memcmp(bytes, expectedOrigin, sizeof(bytes)) == 0);

    /* one whole turn is exactly CountsPerTurn (0x4000), not trailing
     * zeroes of a separate turn field; the four gear angles are served as
     * they came in, one register per role */
    position.IsValid = true;
    position.Turns = 1u;
    position.Angle = 0u;
    positionRegister.Update(position, MakeAngles(0u, 0x1000u, 0x2000u, 0x3000u),
                            0u);
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedOneTurn[] = {
        0x00u, 0x40u, 0x00u, 0x00u, PositionRegister::StatusValid,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, GEAR_ANGLE(0u), GEAR_ANGLE(0x1000u),
        GEAR_ANGLE(0x2000u), GEAR_ANGLE(0x3000u)};
    CHECK(std::memcmp(bytes, expectedOneTurn, sizeof(bytes)) == 0);

    /* the extreme position is one continuous 27-bit count:
     * 7428 * 16384 + 16383 = 121,716,735 = 0x07413FFF */
    position.IsValid = true;
    position.Turns = GearConfig::TurnRange - 1u;
    position.Angle = Mt6701::AngleMax;
    positionRegister.Update(
        position, MakeAngles(Mt6701::AngleMax, 0x1111u, 0x2222u, 0x3333u), 0u);
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedExtreme[] = {
        0xFFu, 0x3Fu, 0x41u, 0x07u, PositionRegister::StatusValid,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, GEAR_ANGLE(0x3FFFu), GEAR_ANGLE(0x1111u),
        GEAR_ANGLE(0x2222u), GEAR_ANGLE(0x3333u)};
    CHECK(std::memcmp(bytes, expectedExtreme, sizeof(bytes)) == 0);
    /* the Sun's gear angle is the low 14 bits of the position count */
    CHECK((0x07413FFFu % 16384u) == 0x3FFFu);

    /* ...which is exactly the exported full-scale value: the extreme
     * sample and the max register are the same number by construction,
     * so a host scaling by the register can never clamp a real sample */
    CHECK(PositionRegister::MaxPosition ==
          (uint32_t)(GearConfig::TurnRange - 1u) *
                  PositionRegister::CountsPerTurn +
              Mt6701::AngleMax);

    /* a slew-guard rejection: all reads clean, status = 0x00; position
     * 1234 turns + 0x1234 angle = 20,222,516 = 0x01349234.  The gear angles
     * are still this sample's readings -- a host debugging the guard sees
     * the wheels of the rejected decode, flagged by the status byte */
    position.IsValid = false;
    position.Turns = 1234u;
    position.Angle = 0x1234u;
    positionRegister.Update(position, MakeAngles(0x1234u, 0u, 0u, 0u), 0u);
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedRejected[] = {
        0x34u, 0x92u, 0x34u, 0x01u, 0x00u,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, GEAR_ANGLE(0x1234u), GEAR_ANGLE(0u),
        GEAR_ANGLE(0u), GEAR_ANGLE(0u)};
    CHECK(std::memcmp(bytes, expectedRejected, sizeof(bytes)) == 0);

    /* failed reads on Sun + Gear3: the health bits ride in the same
     * status byte (bit1 + bit4 = 0x12) and the valid bit clears, so the
     * verdict and the detail can never disagree.  The gear angles still
     * export the sample's readings (a failing wheel holds its previous
     * value, see TestGearAngles) */
    position.IsValid = true;
    position.Turns = 1234u;
    position.Angle = 0x1234u;
    positionRegister.Update(
        position, MakeAngles(0x1234u, 0u, 0u, 0u),
        PositionRegister::HealthBit(EncoderRole::Sun) |
            PositionRegister::HealthBit(EncoderRole::Gear3));
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedFaulty[] = {
        0x34u, 0x92u, 0x34u, 0x01u, 0x12u,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, GEAR_ANGLE(0x1234u), GEAR_ANGLE(0u),
        GEAR_ANGLE(0u), GEAR_ANGLE(0u)};
    CHECK(std::memcmp(bytes, expectedFaulty, sizeof(bytes)) == 0);

    /* auto-increment across separate read calls, one byte at a time --
     * the whole map from 0x02 on: position tail, status, counter, the
     * constant max register, the four gear angles, then zero-fill past the
     * map */
    const uint8_t expectedWalk[] = {
        0x34u, 0x01u, 0x12u,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, GEAR_ANGLE(0x1234u), GEAR_ANGLE(0u),
        GEAR_ANGLE(0u), GEAR_ANGLE(0u), 0u /* past map */, 0u /* stays 0 */};
    positionRegister.Select(2u);
    for (size_t byteIndex = 0u; byteIndex < sizeof(expectedWalk); byteIndex++)
    {
        CHECK(positionRegister.Read(bytes, 1u) == 1u);
        CHECK(bytes[0] == expectedWalk[byteIndex]);
    }

    /* the max register reads standalone too: a host scales its positions
     * with one 4-byte read at pointer 0x07, without the rest of the map */
    positionRegister.Select(PositionRegister::RegisterMaxPosition);
    CHECK(positionRegister.Read(bytes, 4u) == 4u);
    const uint8_t expectedMaxOnly[] = {EXPECTED_MAX_REG_BYTES};
    CHECK(std::memcmp(bytes, expectedMaxOnly, 4u) == 0);

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
    positionRegister.Update(position, MakeAngles(9u, 0u, 0u, 0u),
                            0u); /* count 81,929 = 0x014009 */
    expectedCounter++;
    CHECK(positionRegister.Read(bytes, 1u) == 1u && bytes[0] == 0x00u);
    CHECK(positionRegister.Read(bytes, 1u) == 1u &&
          bytes[0] == PositionRegister::StatusValid);

    /* window through the middle of the map (pointer + partial read) */
    positionRegister.Select(1u);
    CHECK(positionRegister.Read(bytes, 2u) == 2u);
    const uint8_t expectedWindow[] = {0x40u, 0x01u};
    CHECK(std::memcmp(bytes, expectedWindow, 2u) == 0); /* count bytes 1..2 */

    /* the gear angles read standalone too: 8 bytes from pointer 0x0B, one
       register per role, in role order -- the Sun's is the position's low
       14 bits, the driven wheels show what their own chips measured */
    position.IsValid = true;
    position.Turns = 4000u;
    position.Angle = 333u;
    positionRegister.Update(position, MakeAngles(333u, 13747u, 14125u, 14435u),
                            0u);
    expectedCounter++;
    const uint8_t expectedGearAngles[] = {GEAR_ANGLE(333u), GEAR_ANGLE(13747u),
                                          GEAR_ANGLE(14125u),
                                          GEAR_ANGLE(14435u)};
    positionRegister.Select(PositionRegister::RegisterGearAngle);
    CHECK(positionRegister.Read(bytes, 8u) == 8u);
    CHECK(std::memcmp(bytes, expectedGearAngles, 8u) == 0);
}

/* The full firmware sample loop (AppProcessSample: four real SSI reads ->
 * decode -> publish) driven against the simulated chips: read failures must
 * clear the status valid bit, set the health bits and freeze the position,
 * while the sample counter keeps ticking.  Position here is 4000 turns +
 * 333 counts = 65,536,333 = 0x03E8014D (LE: 4D 01 E8 03), and the gear
 * angles of every sample below are EXPECTED_ANGLES_HEALTHY: a failed read
 * holds that wheel's previous angle, so the registers keep describing the
 * sample even while the status byte flags the failure. */
static void TestAppSampleLoop()
{
    std::printf("sample loop: health and freshness through AppProcessSample\n");
    const uint32_t inputCounts = 4000u * 16384u + 333u;
    Mt6701SlaveSim::SetAngle(EncoderRole::Sun, 333u);
    Mt6701SlaveSim::SetAngle(
        EncoderRole::Gear1,
        ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[0]));
    Mt6701SlaveSim::SetAngle(
        EncoderRole::Gear2,
        ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[1]));
    Mt6701SlaveSim::SetAngle(
        EncoderRole::Gear3,
        ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[2]));

    positionRegister.Init();
    uint8_t bytes[PositionRegister::RegisterCount];
    uint16_t expectedCounter = 0u; /* +1 per AppProcessSample call below */

    /* sample 1: all four chips healthy -- status is exactly StatusValid */
    AppProcessSample();
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedHealthy[] = {
        0x4Du, 0x01u, 0xE8u, 0x03u, PositionRegister::StatusValid,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, EXPECTED_ANGLES_HEALTHY};
    CHECK(std::memcmp(bytes, expectedHealthy, sizeof(bytes)) == 0);

    /* sample 2: sun read fails (track loss): status = bit1 (0x02), valid
     * cleared, position frozen, counter still ticking */
    Mt6701SlaveSim::SetTrackLoss(EncoderRole::Sun, true);
    AppProcessSample();
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedSunFault[] = {
        0x4Du, 0x01u, 0xE8u, 0x03u,
        PositionRegister::HealthBit(EncoderRole::Sun),
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, EXPECTED_ANGLES_HEALTHY};
    CHECK(std::memcmp(bytes, expectedSunFault, sizeof(bytes)) == 0);
    Mt6701SlaveSim::SetTrackLoss(EncoderRole::Sun, false);

    /* sample 3: every chip unresponsive: all health bits set (0x1E),
     * valid cleared, position frozen -- a fully dead encoder farm no
     * longer reads as a valid stationary shaft */
    Mt6701SlaveSim::SetStuck(EncoderRole::Sun, true);
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear1, true);
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear2, true);
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear3, true);
    AppProcessSample();
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedAllDead[] = {
        0x4Du, 0x01u, 0xE8u, 0x03u, 0x1Eu,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, EXPECTED_ANGLES_HEALTHY};
    CHECK(std::memcmp(bytes, expectedAllDead, sizeof(bytes)) == 0);
    Mt6701SlaveSim::SetStuck(EncoderRole::Sun, false);
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear1, false);
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear2, false);
    Mt6701SlaveSim::SetStuck(EncoderRole::Gear3, false);

    /* sample 4: recovery -- normal service again */
    AppProcessSample();
    expectedCounter++;
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, sizeof(bytes)) == sizeof(bytes));
    const uint8_t expectedRecovered[] = {
        0x4Du, 0x01u, 0xE8u, 0x03u, PositionRegister::StatusValid,
        (uint8_t)(expectedCounter & 0xFFu), (uint8_t)(expectedCounter >> 8u),
        EXPECTED_MAX_REG_BYTES, EXPECTED_ANGLES_HEALTHY};
    CHECK(std::memcmp(bytes, expectedRecovered, sizeof(bytes)) == 0);
}

/* The gear-angle registers end to end: the real sample loop (four SSI reads
 * -> decode -> publish) must expose each wheel's own angle next to the
 * decoded position, so a host can watch all four MT6701s over I2C. */
static void TestGearAngles()
{
    std::printf("gear angles through AppProcessSample\n");
    const uint32_t inputCounts = 4000u * 16384u + 333u;
    Mt6701SlaveSim::SetAngle(EncoderRole::Sun, 333u);
    Mt6701SlaveSim::SetAngle(
        EncoderRole::Gear1,
        ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[0]));
    Mt6701SlaveSim::SetAngle(
        EncoderRole::Gear2,
        ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[1]));
    Mt6701SlaveSim::SetAngle(
        EncoderRole::Gear3,
        ComputeGearAngle(inputCounts, GearConfig::DrivenTeethCounts[2]));

    positionRegister.Init();
    uint8_t bytes[PositionRegister::RegisterCount];

    /* the four angles as their own chips reported them, one register per
       role; the Sun's is the low 14 bits of the position count beside it */
    AppProcessSample();
    positionRegister.Select(PositionRegister::RegisterGearAngle);
    CHECK(positionRegister.Read(bytes, 8u) == 8u);
    const uint8_t expectedAngles[] = {EXPECTED_ANGLES_HEALTHY};
    CHECK(std::memcmp(bytes, expectedAngles, 8u) == 0);
    positionRegister.Select(0u);
    CHECK(positionRegister.Read(bytes, 4u) == 4u);
    CHECK((((uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8u) |
            ((uint32_t)bytes[2] << 16u) | ((uint32_t)bytes[3] << 24u)) %
           16384u) == 333u);

    /* a chip that moves on: its register follows (Gear2 at 14,125 here)
       while the decoded position stays put -- the wheel is well inside its
       tooth slot, so the sample still certifies */
    Mt6701SlaveSim::SetAngle(EncoderRole::Gear2, 14125u);
    AppProcessSample();
    positionRegister.Select(PositionRegister::RegisterGearAngle);
    CHECK(positionRegister.Read(bytes, 8u) == 8u);
    const uint8_t expectedMoved[] = {GEAR_ANGLE(333u), GEAR_ANGLE(0x35B3u),
                                     GEAR_ANGLE(14125u),
                                     GEAR_ANGLE(0x3863u)};
    CHECK(std::memcmp(bytes, expectedMoved, 8u) == 0);
    positionRegister.Select(PositionRegister::RegisterStatus);
    CHECK(positionRegister.Read(bytes, 1u) == 1u &&
          bytes[0] == PositionRegister::StatusValid);

    /* a failed read leaves that wheel's last angle in its register -- the
       health bit is what tells the host the value is held, so the register
       never shows a number no chip ever reported */
    Mt6701SlaveSim::SetTrackLoss(EncoderRole::Gear2, true);
    AppProcessSample();
    positionRegister.Select(PositionRegister::RegisterStatus);
    CHECK(positionRegister.Read(bytes, 1u) == 1u &&
          bytes[0] == PositionRegister::HealthBit(EncoderRole::Gear2));
    positionRegister.Select(PositionRegister::RegisterGearAngle);
    CHECK(positionRegister.Read(bytes, 8u) == 8u);
    CHECK(std::memcmp(bytes, expectedMoved, 8u) == 0);
    Mt6701SlaveSim::SetTrackLoss(EncoderRole::Gear2, false);
}

int main()
{
    hal.Init();

    TestStaticAngle();
    TestEncoderIndependence();
    TestFaults();
    TestCrc();
    TestGearDecodeReadPath();
    TestI2cPosition();
    TestAppSampleLoop();
    TestGearAngles();

    std::printf("%d/%d checks passed\n", _totalChecks - _failedChecks,
                _totalChecks);
    return _failedChecks != 0 ? 1 : 0;
}