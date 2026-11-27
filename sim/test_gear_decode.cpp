/**
 * @file test_gear_decode.cpp
 * @brief Unit tests for the GearDecoder class (App/gear_decode.cpp).
 *
 * Standalone binary: links only gear_decode.cpp -- no HAL, no simulated
 * chips.  Every sample is a GearAngles tuple constructed directly, so
 * the decoder's public API (constructor / Reset / Decode / IsInitialized)
 * is exercised with raw angle counts rather than through the SSI read
 * path.
 *
 * Exit code 0 = all checks passed; built and run by `make run` in sim/.
 */
#include <cstdio>

#include "gear_decode.h"
#include "gear_train_sim.h" /* GearTrainSim: the same plant as the chip sim */
#include "mt6701.h"         /* Mt6701::AngleMax */

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

/* All four readings with the input shaft `shaftCounts` past zero (1/16384
 * input revolution).  The plant is shared with the chip simulator: a driven
 * wheel turns InputTeeth/teeth revolutions per input revolution, and each
 * chip reports its own wheel's angle. */
static GearAngles ShaftAngles(uint32_t shaftCounts)
{
    shaftCounts = GearTrainSim::WrapShaftPosition(shaftCounts);

    GearAngles angles;
    angles.Sun = GearTrainSim::SunAngle(shaftCounts);
    angles.Gear1 =
        GearTrainSim::WheelAngle(shaftCounts, GearConfig::DrivenTeethCounts[0]);
    angles.Gear2 =
        GearTrainSim::WheelAngle(shaftCounts, GearConfig::DrivenTeethCounts[1]);
    angles.Gear3 =
        GearTrainSim::WheelAngle(shaftCounts, GearConfig::DrivenTeethCounts[2]);
    return angles;
}

/* Sample set at a continuous input position: `turns` whole turns plus
 * `sunAngle` of the next one.  The driven gears follow that same position,
 * as they do on the real gear train. */
static GearAngles CreateAnglesAt(uint32_t turns, uint16_t sunAngle)
{
    return ShaftAngles(turns * 16384u + sunAngle);
}

/* Sample set at a whole turn (Sun at 0). */
static GearAngles CreateAnglesFor(uint32_t turns)
{
    return CreateAnglesAt(turns, 0u);
}

/* Decoded position as one continuous shaft count, turns * 16384 + angle. */
static uint32_t DecodedShaftCounts(const GearPosition& position)
{
    return static_cast<uint32_t>(position.Turns) * 16384u + position.Angle;
}

/* Decode one sample with a fresh decoder -- with no previous reading the
 * slew guard cannot hide a wrong decode -- and report the decoded values
 * when the position is wrong. */
static void CheckDecodeAt(uint32_t turns, uint16_t sunAngle)
{
    GearAngles angles = CreateAnglesAt(turns, sunAngle);
    GearDecoder decoder;
    GearPosition position = decoder.Decode(angles);

    _totalChecks++;
    if (!position.IsValid || position.Turns != turns ||
        position.Angle != sunAngle)
    {
        _failedChecks++;
        std::printf("  FAIL %u turns + %u/16384 -> decoded %u turns + "
                    "%u/16384 (valid %d)\n",
                    turns, sunAngle, position.Turns, position.Angle,
                    position.IsValid ? 1 : 0);
    }
}

/* Shortest circular distance between two turn counts, 0 .. RANGE/2. */
static uint32_t CircularDistance(uint32_t first, uint32_t second)
{
    uint32_t distance = (first + GearConfig::TurnRange - second) %
                        GearConfig::TurnRange;
    if (distance > GearConfig::TurnRange / 2u)
    {
        distance = GearConfig::TurnRange - distance;
    }
    return distance;
}

/* Raw count `rawCount + offset`, wrapped onto the 14-bit circle. */
static uint16_t AddOffset(uint16_t rawCount, int32_t offset)
{
    return static_cast<uint16_t>(((int32_t)rawCount + 16384 + offset) % 16384);
}

/* One gear field of a sample tuple (Gear1..3), by driven-gear position. */
static uint16_t& GearAngleField(GearAngles& angles, size_t gearIndex)
{
    if (gearIndex == 0u) return angles.Gear1;
    if (gearIndex == 1u) return angles.Gear2;
    return angles.Gear3;
}

/* Const read side of GearAngleField. */
static uint16_t GearAngleValue(const GearAngles& angles, size_t gearIndex)
{
    if (gearIndex == 0u) return angles.Gear1;
    if (gearIndex == 1u) return angles.Gear2;
    return angles.Gear3;
}

/* The decoder's count -> tooth mapping, restated: tooth k spans
 * [ceil((16384k - 8192) / teeth), floor((16384k + 8191) / teeth)]. */
static uint32_t CountToTooth(uint32_t rawCount, size_t gearIndex)
{
    uint32_t teeth = GearConfig::DrivenTeethCounts[gearIndex];
    return ((rawCount * teeth + 8192u) / 16384u) % teeth;
}

/* The absolute turn count a sample should decode to: the unique T in
 * [0, TurnRange) whose gear teeth match the sample's.  A brute-force CRT
 * oracle (at most TurnRange <= 8192 scans) kept independent of the
 * decoder's closed-form pairing so it can validate it.
 *
 * Assumes the sample's Sun angle adds no whole tooth (k = 0): callers use a
 * whole turn or a small within-turn angle, where the driven-gear tooth
 * numbers are still (InputTeeth * T) mod teeth. */
static uint32_t ExpectedDecodedTurns(const GearAngles& angles)
{
    for (uint32_t turns = 0u; turns < GearConfig::TurnRange; turns++)
    {
        bool matches = true;
        for (size_t i = 0u; i < GearConfig::DrivenCount; i++)
        {
            uint32_t teeth = GearConfig::DrivenTeethCounts[i];
            uint32_t expectedTooth = (GearConfig::InputTeeth * turns) % teeth;
            if (CountToTooth(GearAngleValue(angles, i), i) != expectedTooth)
            {
                matches = false;
                break;
            }
        }
        if (matches)
        {
            return turns;
        }
    }
    return 0u; /* unreachable while the geometry is valid */
}

/* Raw-count range that maps to one tooth: the decoder rounds
 * count -> tooth via ((count * teeth + 8192) / 16384) % teeth, so tooth k
 * spans [ceil((16384k - 8192) / teeth), floor((16384k + 8191) / teeth)]. */
static uint16_t ToothSlotLow(uint32_t toothNumber, uint32_t gearTeeth)
{
    int32_t numerator = (int32_t)(toothNumber * 16384u) - 8192;
    if (numerator <= 0)
    {
        return 0u;
    }
    return static_cast<uint16_t>(
        (numerator + (int32_t)gearTeeth - 1) / (int32_t)gearTeeth);
}

static uint16_t ToothSlotHigh(uint32_t toothNumber, uint32_t gearTeeth)
{
    return static_cast<uint16_t>((toothNumber * 16384u + 8191u) / gearTeeth);
}

/* Must come first: exercises the constructor's geometry validation, that
 * Reset() does not un-initialize it, and that a fresh decoder accepts the
 * first sample as the reference. */
static void TestInit()
{
    std::printf("constructor validates the geometry, reset keeps it\n");
    GearDecoder decoder;
    CHECK(decoder.IsInitialized());
    decoder.Reset();
    GearAngles anglesAtTurn0 = CreateAnglesFor(0u);
    GearPosition position = decoder.Decode(anglesAtTurn0);
    CHECK(position.IsValid);
    CHECK(position.Turns == 0u);
}

static void TestExactDecode()
{
    std::printf("exact decodes across the range at several sun angles\n");
    const uint32_t turnCounts[] = {0u, 1u, 2u, 5u, 7u, 8u, 3654u, 4000u,
                                   GearConfig::TurnRange - 1u};
    const uint16_t sunAngles[] = {0u, 1u, 163u, Mt6701::AngleMax};

    for (size_t turnIndex = 0u;
         turnIndex < sizeof(turnCounts) / sizeof(turnCounts[0]);
         turnIndex++)
    {
        for (size_t sunAngleIndex = 0u;
             sunAngleIndex < sizeof(sunAngles) / sizeof(sunAngles[0]);
             sunAngleIndex++)
        {
            CheckDecodeAt(turnCounts[turnIndex], sunAngles[sunAngleIndex]);
        }
    }
}

/* The decoder must be right at any point within a turn, not just at whole
 * turns: the driven gears move with the continuous input position, so their
 * tooth numbers carry a shared offset k = round(13 * sunAngle / 16384) that
 * the turn decode has to remove.  The angles bracket the k = 0/1 and
 * k = 12/13 boundaries and the half-turn tie. */
static void TestMidTurnDecode()
{
    std::printf("decodes at arbitrary within-turn positions\n");
    const uint32_t turnCounts[] = {0u, 7u, 4000u,
                                   GearConfig::TurnRange - 1u};
    const uint16_t sunAngles[] = {0u,    1u,    630u,  631u,  2000u, 4096u,
                                  8192u, 12000u, 15753u, 15754u, 16383u};

    for (size_t turnIndex = 0u;
         turnIndex < sizeof(turnCounts) / sizeof(turnCounts[0]);
         turnIndex++)
    {
        for (size_t sunAngleIndex = 0u;
             sunAngleIndex < sizeof(sunAngles) / sizeof(sunAngles[0]);
             sunAngleIndex++)
        {
            CheckDecodeAt(turnCounts[turnIndex], sunAngles[sunAngleIndex]);
        }
    }
}

/* One rotating shaft, sampled across a turn: every sample must decode to the
 * same turn count so the slew guard accepts the sequence.  Before the Sun
 * de-rotation each mid-turn sample landed on a different branch, so the guard
 * rejected the steps and froze the output. */
static void TestMidTurnTracking()
{
    std::printf("mid-turn samples track one turn without guard rejections\n");
    const uint32_t turns = 4000u;
    const uint16_t sunAngles[] = {0u, 4096u, 8192u, 12000u, 16383u};

    GearDecoder decoder;
    for (size_t angleIndex = 0u;
         angleIndex < sizeof(sunAngles) / sizeof(sunAngles[0]); angleIndex++)
    {
        GearAngles angles = CreateAnglesAt(turns, sunAngles[angleIndex]);
        GearPosition position = decoder.Decode(angles);
        CHECK(position.IsValid);
        CHECK(position.Turns == turns);
        CHECK(position.Angle == sunAngles[angleIndex]);
    }

    /* Across the turn boundary: 4000.99994 -> 4001.00000 is a one-turn step. */
    GearAngles nextTurn = CreateAnglesAt(turns + 1u, 0u);
    GearPosition position = decoder.Decode(nextTurn);
    CHECK(position.IsValid);
    CHECK(position.Turns == turns + 1u);
}

/* Power-up anywhere in the range: the first sample after reset has no
 * history, so it alone must decode to the absolute position. */
static void TestPowerUpAnywhere()
{
    std::printf("power-up at every turn in the range\n");
    const uint16_t sunAngles[] = {0u, 1234u, 8191u, Mt6701::AngleMax};
    for (uint32_t turns = 0u; turns < GearConfig::TurnRange; turns++)
    {
        for (size_t angleIndex = 0u;
             angleIndex < sizeof(sunAngles) / sizeof(sunAngles[0]);
             angleIndex++)
        {
            CheckDecodeAt(turns, sunAngles[angleIndex]);
        }
    }
}

/* A shaft standing at every possible within-turn angle.  The de-rotation
 * must recover the same turn count for all 16384 angular positions. */
static void TestAllWithinTurnAngles()
{
    std::printf("all 16384 within-turn angles decode\n");
    const uint32_t turnCounts[] = {0u, 4000u, GearConfig::TurnRange - 1u};
    for (size_t turnIndex = 0u;
         turnIndex < sizeof(turnCounts) / sizeof(turnCounts[0]); turnIndex++)
    {
        for (uint32_t sunCount = 0u; sunCount <= Mt6701::AngleMax; sunCount++)
        {
            CheckDecodeAt(turnCounts[turnIndex],
                          static_cast<uint16_t>(sunCount));
        }
    }
}

/* 1 kHz samples of a turning shaft: each sample must land exactly on the
 * shaft position, forward and backward, across turn boundaries and the
 * 7429-turn seam.  Rates are far below the configured slew limit. */
static void TestContinuousRotation()
{
    std::printf("1 kHz samples track a turning shaft in both directions\n");
    const uint32_t shaftRange = GearConfig::TurnRange * 16384u;
    const uint32_t rates[] = {1u, 1000u};

    for (size_t rateIndex = 0u;
         rateIndex < sizeof(rates) / sizeof(rates[0]); rateIndex++)
    {
        const uint32_t rate = rates[rateIndex];

        /* Forward, starting just before the seam. */
        GearDecoder decoder;
        uint32_t shaft = (GearConfig::TurnRange - 1u) * 16384u + 5000u;
        for (uint32_t step = 0u; step < 20000u; step++)
        {
            shaft = (shaft + rate) % shaftRange;
            GearPosition position = decoder.Decode(ShaftAngles(shaft));
            CHECK(position.IsValid);
            CHECK(DecodedShaftCounts(position) == shaft);
        }

        /* Backward, starting just after zero. */
        decoder.Reset();
        shaft = 5000u;
        for (uint32_t step = 0u; step < 20000u; step++)
        {
            shaft = (shaft + shaftRange - rate) % shaftRange;
            GearPosition position = decoder.Decode(ShaftAngles(shaft));
            CHECK(position.IsValid);
            CHECK(DecodedShaftCounts(position) == shaft);
        }
    }

    /* Exactly MaxTurnsDelta turns between samples is the fastest step the
     * guard accepts (the configured limit). */
    GearDecoder decoder;
    uint32_t shaft = 4000u * 16384u + 1000u;
    CHECK(decoder.Decode(ShaftAngles(shaft)).IsValid);
    shaft = (shaft + GearConfig::MaxTurnsDelta * 16384u) % shaftRange;
    GearPosition position = decoder.Decode(ShaftAngles(shaft));
    CHECK(position.IsValid);
    CHECK(position.Turns == 4000u + GearConfig::MaxTurnsDelta);
}

/* Deterministic last-bit jitter, -4 .. +4 counts. */
static uint32_t _noiseState = 1u;

static int32_t NextNoiseCount()
{
    _noiseState = _noiseState * 1664525u + 1013904223u;
    return static_cast<int32_t>((_noiseState >> 16) % 9u) - 4;
}

/* A stationary shaft with independent jitter on all four chips: the tooth
 * rounding must absorb it, and .Angle is the live Sun reading. */
static void TestSensorNoise()
{
    std::printf("per-chip jitter is absorbed by the tooth slot\n");
    const uint16_t sunAngles[] = {3000u, 7000u, 11000u, 15000u};
    const uint32_t turnCounts[] = {0u, 1234u, 4000u,
                                   GearConfig::TurnRange - 1u};

    for (size_t turnIndex = 0u;
         turnIndex < sizeof(turnCounts) / sizeof(turnCounts[0]); turnIndex++)
    {
        for (size_t angleIndex = 0u;
             angleIndex < sizeof(sunAngles) / sizeof(sunAngles[0]);
             angleIndex++)
        {
            GearAngles clean =
                CreateAnglesAt(turnCounts[turnIndex], sunAngles[angleIndex]);
            GearDecoder decoder;
            for (uint32_t sample = 0u; sample < 200u; sample++)
            {
                GearAngles noisy = clean;
                noisy.Sun = AddOffset(noisy.Sun, NextNoiseCount());
                noisy.Gear1 = AddOffset(noisy.Gear1, NextNoiseCount());
                noisy.Gear2 = AddOffset(noisy.Gear2, NextNoiseCount());
                noisy.Gear3 = AddOffset(noisy.Gear3, NextNoiseCount());

                GearPosition position = decoder.Decode(noisy);
                CHECK(position.IsValid);
                CHECK(position.Turns == turnCounts[turnIndex]);
                CHECK(position.Angle == noisy.Sun);
            }
        }
    }
}

/* The four chips are read one after another, so one sample is a tuple from
 * slightly different instants.  At 1 kHz and 1000 counts/sample an SPI
 * frame is ~24 counts of shaft motion: a 72-count skew across the tuple is
 * 0.06 tooth and must decode exactly.  A whole stale chip is a one-tooth
 * slip and must be rejected. */
static void TestReadSkew()
{
    std::printf("sequential reads: small skew decodes, a stale chip is rejected\n");
    const uint32_t shaft = 4000u * 16384u + 5000u;
    const uint32_t skew = 24u;

    GearAngles skewed;
    skewed.Sun = static_cast<uint16_t>(shaft % 16384u);
    skewed.Gear1 =
        GearTrainSim::WheelAngle(shaft + skew, GearConfig::DrivenTeethCounts[0]);
    skewed.Gear2 =
        GearTrainSim::WheelAngle(shaft + 2u * skew,
                                 GearConfig::DrivenTeethCounts[1]);
    skewed.Gear3 =
        GearTrainSim::WheelAngle(shaft + 3u * skew,
                                 GearConfig::DrivenTeethCounts[2]);

    GearDecoder decoder;
    GearPosition position = decoder.Decode(skewed);
    CHECK(position.IsValid);
    CHECK(position.Turns == 4000u);
    CHECK(position.Angle == skewed.Sun);

    /* The shaft advances one driven tooth while one chip's reading is a
     * whole sample old: the tooth slips by one, which decodes hundreds of
     * turns away, so the guard must reject it. */
    const uint32_t oneInputTooth = 16384u / GearConfig::InputTeeth;
    GearAngles clean = ShaftAngles(shaft + oneInputTooth);
    GearAngles stale = clean;
    stale.Gear3 = skewed.Gear3; /* the previous sample's reading */

    position = decoder.Decode(stale);
    CHECK(!position.IsValid);

    /* The next clean sample is accepted at the advanced position. */
    position = decoder.Decode(clean);
    CHECK(position.IsValid);
    CHECK(DecodedShaftCounts(position) == shaft + oneInputTooth);
}

/* Golden samples: raw MT6701 counts at known shaft positions, derived from
 * the gear ratio only -- a driven wheel turns 13/teeth revolutions per
 * input revolution -- and frozen here as literals.  The decoder cannot
 * redefine what the hardware reports at these positions. */
struct GoldenSample
{
    uint16_t Sun;
    uint16_t Gear1;
    uint16_t Gear2;
    uint16_t Gear3;
    uint16_t Turns;
};

static const GoldenSample _goldenSamples[] = {
    {0u, 0u, 0u, 0u, 0u},                     /* zero position */
    {0u, 5783u, 12935u, 15672u, 7u},          /* 7 turns, theta = 0 */
    {631u, 6265u, 13366u, 16028u, 7u},        /* 7 turns + half a tooth */
    {2000u, 7312u, 14303u, 418u, 7u},         /* 7 turns + 1.6 teeth */
    {4096u, 8915u, 15737u, 1603u, 7u},        /* 7 turns + 1/4 turn */
    {8192u, 12047u, 2156u, 3918u, 7u},        /* 7 turns + 1/2 turn, tooth edge */
    {12000u, 14959u, 4761u, 6070u, 7u},       /* 7 turns + 9.5 teeth */
    {15754u, 1446u, 7330u, 8192u, 7u},        /* 7 turns + 12.5 teeth */
    {16383u, 1927u, 7760u, 8548u, 7u},        /* 7 turns + last count */
    {8192u, 6264u, 5605u, 4630u, 0u},         /* 0 turns + 1/2 turn */
    {1234u, 14436u, 14641u, 14944u, 4000u},   /* 4000 turns + 1234 */
    {16383u, 16383u, 16383u, 16383u, 7428u},  /* last count of the range */
    {3000u, 12896u, 7227u, 9531u, 1234u},     /* 1234 turns + 3000 */
};

static void TestGoldenSamples()
{
    std::printf("golden raw samples from the real gear ratio\n");
    for (size_t sampleIndex = 0u;
         sampleIndex < sizeof(_goldenSamples) / sizeof(_goldenSamples[0]);
         sampleIndex++)
    {
        const GoldenSample& sample = _goldenSamples[sampleIndex];
        GearAngles angles = {
            sample.Sun, sample.Gear1, sample.Gear2, sample.Gear3};

        /* Power-up: with no previous reading, this sample alone is the
         * absolute position. */
        GearDecoder decoder;
        GearPosition position = decoder.Decode(angles);
        CHECK(position.IsValid);
        CHECK(position.Turns == sample.Turns);
        CHECK(position.Angle == sample.Sun);
        CHECK(DecodedShaftCounts(position) ==
              static_cast<uint32_t>(sample.Turns) * 16384u + sample.Sun);
    }

    /* One driven tooth ahead of the same physical sample: the guard must
     * reject it instead of accepting a misread position. */
    GearAngles reference = {4096u, 8915u, 15737u, 1603u};
    GearAngles misread = reference;
    misread.Gear3 = 2137u; /* next tooth centre, 3 * 16384 / 23 */
    GearDecoder decoder;
    CHECK(decoder.Decode(reference).IsValid);
    GearPosition position = decoder.Decode(misread);
    CHECK(!position.IsValid);

    /* A whole-turn-stale gear reading taken from the same turn: rejected
     * too, and tracking stays on the last accepted position. */
    misread = reference;
    misread.Gear3 = 15672u; /* Gear3's reading at 7 turns exactly */
    position = decoder.Decode(misread);
    CHECK(!position.IsValid);
    CHECK(decoder.Decode(reference).IsValid);
}

static void TestAngleEdges()
{
    std::printf("raw-count extremes and tooth-slot edges\n");

    /* The Sun's extremes are valid positions at the turn boundary; the
     * driven gears must follow the Sun angle to the same position. */
    const uint16_t sunEdgeCounts[] = {0u, 1u, Mt6701::AngleMax - 1u,
                                      Mt6701::AngleMax};
    for (size_t edgeIndex = 0u;
         edgeIndex < sizeof(sunEdgeCounts) / sizeof(sunEdgeCounts[0]);
         edgeIndex++)
    {
        CheckDecodeAt(0u, sunEdgeCounts[edgeIndex]);
    }

    /* A driven gear's raw extremes at a whole turn both sit inside tooth 0,
     * so an extreme count on one gear cannot change the decode. */
    const uint16_t gearEdgeCounts[] = {0u, 1u, Mt6701::AngleMax - 1u,
                                       Mt6701::AngleMax};
    for (size_t fieldIndex = 1u; fieldIndex < 4u; fieldIndex++)
    {
        for (size_t edgeIndex = 0u;
             edgeIndex < sizeof(gearEdgeCounts) / sizeof(gearEdgeCounts[0]);
             edgeIndex++)
        {
            GearAngles angles = CreateAnglesFor(0u);
            switch (fieldIndex)
            {
            case 1u: angles.Gear1 = gearEdgeCounts[edgeIndex]; break;
            case 2u: angles.Gear2 = gearEdgeCounts[edgeIndex]; break;
            default: angles.Gear3 = gearEdgeCounts[edgeIndex]; break;
            }
            GearDecoder decoder;
            GearPosition position = decoder.Decode(angles);
            CHECK(position.IsValid);
            CHECK(position.Turns == 0u);
            CHECK(position.Angle == angles.Sun);
        }
    }

    /* All three driven gears at an extreme count still decode to turn 0. */
    GearDecoder decoder;
    GearAngles allGearExtremes = CreateAnglesFor(0u);
    allGearExtremes.Gear1 = Mt6701::AngleMax;
    allGearExtremes.Gear2 = Mt6701::AngleMax;
    allGearExtremes.Gear3 = Mt6701::AngleMax;
    GearPosition position = decoder.Decode(allGearExtremes);
    CHECK(position.IsValid);
    CHECK(position.Turns == 0u);
    CHECK(position.Angle == 0u);

    /* One count past the slot edge of tooth 0 is the first count of tooth
     * 1: gear1 crossing the edge alone decodes to a misread turn count.
     * Fresh from reset it is accepted as a reference; with a reference at
     * 0 the guard rejects it.  The expected value comes from the CRT
     * oracle, so it tracks whatever geometry GearConfig declares. */
    const uint16_t lastCountOfTooth0 =
        ToothSlotHigh(0u, GearConfig::DrivenTeethCounts[0]);
    const uint16_t firstCountOfTooth1 =
        ToothSlotLow(1u, GearConfig::DrivenTeethCounts[0]);

    /* At a whole turn the raw tooth-slot edges are the de-rotated ones, so
     * the oracle maps this sample directly. */
    const uint16_t referenceAngle = 0u;
    GearAngles misreadAngles = CreateAnglesAt(0u, referenceAngle);
    misreadAngles.Gear1 = firstCountOfTooth1;
    uint32_t expectedMisread = ExpectedDecodedTurns(misreadAngles);
    /* A single-tooth slip on a driven gear jumps by at least the product of
     * the other two gears: the guard rejection below is meaningful. */
    CHECK(CircularDistance(expectedMisread, 0u) > GearConfig::MaxTurnsDelta);

    GearDecoder freshDecoder;
    position = freshDecoder.Decode(misreadAngles);
    CHECK(position.IsValid);
    CHECK(position.Turns == expectedMisread);

    GearAngles anglesAtTurn0 = CreateAnglesAt(0u, referenceAngle);
    decoder.Reset();
    CHECK(decoder.Decode(anglesAtTurn0).IsValid); /* reference at turn 0 */
    GearAngles lastCountAngles = CreateAnglesAt(0u, referenceAngle);
    lastCountAngles.Gear1 = lastCountOfTooth0; /* last count of tooth 0 */
    position = decoder.Decode(lastCountAngles);
    CHECK(position.IsValid);
    CHECK(position.Turns == 0u);

    GearAngles firstCountAngles = CreateAnglesAt(0u, referenceAngle);
    firstCountAngles.Gear1 = firstCountOfTooth1; /* first count of tooth 1 */
    position = decoder.Decode(firstCountAngles);
    CHECK(!position.IsValid);
    /* .Turns carries the decoded-but-rejected reading: the guard rejects
     * it, it was never accepted as the position */
    CHECK(position.Turns == expectedMisread);
    CHECK(position.Angle == referenceAngle);
}

static void TestSlotQuantization()
{
    std::printf("every count inside a tooth slot decodes to the same turns\n");
    /* The low and high edge of the expected slot are far apart (up to a
     * full slot, ~16384/teeth counts) yet must map to the same tooth and
     * thus the same absolute position. */
    const uint32_t turnCounts[] = {0u, 1234u, 4000u, GearConfig::TurnRange - 1u};
    for (size_t turnIndex = 0u;
         turnIndex < sizeof(turnCounts) / sizeof(turnCounts[0]);
         turnIndex++)
    {
        for (size_t gearIndex = 0u; gearIndex < GearConfig::DrivenCount;
             gearIndex++)
        {
            uint32_t toothNumber =
                (GearConfig::InputTeeth * turnCounts[turnIndex]) %
                GearConfig::DrivenTeethCounts[gearIndex];
            uint16_t slotLow =
                ToothSlotLow(toothNumber, GearConfig::DrivenTeethCounts[gearIndex]);
            uint16_t slotHigh =
                ToothSlotHigh(toothNumber, GearConfig::DrivenTeethCounts[gearIndex]);
            CHECK(slotLow <= slotHigh);

            GearAngles angles = CreateAnglesFor(turnCounts[turnIndex]);
            GearAngleField(angles, gearIndex) = slotLow;
            GearDecoder decoder;
            GearPosition position = decoder.Decode(angles);
            CHECK(position.IsValid);
            CHECK(position.Turns == turnCounts[turnIndex]);

            angles = CreateAnglesFor(turnCounts[turnIndex]);
            GearAngleField(angles, gearIndex) = slotHigh;
            decoder.Reset();
            position = decoder.Decode(angles);
            CHECK(position.IsValid);
            CHECK(position.Turns == turnCounts[turnIndex]);
        }
    }
}

static void TestNoiseAndSmallSteps()
{
    std::printf("sub-slot noise is absorbed, small steps accepted\n");
    /* Moderate noise within half a tooth slot changes nothing. */
    GearAngles angles = CreateAnglesFor(1234u);
    angles.Gear1 = static_cast<uint16_t>((angles.Gear1 + 80u) % 16384u);
    angles.Gear2 = static_cast<uint16_t>(angles.Gear2 - 60u); /* wraps, same circle */
    angles.Gear3 = static_cast<uint16_t>((angles.Gear3 + 300u) % 16384u);
    GearDecoder decoder;
    GearPosition position = decoder.Decode(angles);
    CHECK(position.IsValid);
    CHECK(position.Turns == 1234u);

    /* A repeated identical sample stays accepted (distance 0). */
    position = decoder.Decode(angles);
    CHECK(position.IsValid);
    CHECK(position.Turns == 1234u);

    /* One-turn forward step: 7 -> 8. */
    decoder.Reset();
    GearAngles anglesAtTurn7 = CreateAnglesFor(7u);
    CHECK(decoder.Decode(anglesAtTurn7).IsValid);
    GearAngles anglesAtTurn8 = CreateAnglesFor(8u);
    position = decoder.Decode(anglesAtTurn8);
    CHECK(position.IsValid);
    CHECK(position.Turns == 8u);
}

static void TestWrapSeam()
{
    std::printf("%u-turn seam: distance-1 steps wrap, distance-2 rejected\n",
                (unsigned)GearConfig::TurnRange);
    GearAngles anglesAtTurn0 = CreateAnglesFor(0u);

    /* +1 across the seam: 7428 -> 0. */
    GearDecoder decoder;
    {
        GearAngles anglesAtLastTurn = CreateAnglesFor(GearConfig::TurnRange - 1u);
        CHECK(decoder.Decode(anglesAtLastTurn).IsValid);
    }
    GearPosition position = decoder.Decode(anglesAtTurn0);
    CHECK(position.IsValid);
    CHECK(position.Turns == 0u);

    /* -1 across the seam: 0 -> 7428. */
    decoder.Reset();
    CHECK(decoder.Decode(anglesAtTurn0).IsValid);
    GearAngles anglesAtLastTurn = CreateAnglesFor(GearConfig::TurnRange - 1u);
    position = decoder.Decode(anglesAtLastTurn);
    CHECK(position.IsValid);
    CHECK(position.Turns == GearConfig::TurnRange - 1u);

    /* Distance 2 is a full half-flight: rejected in both directions. */
    decoder.Reset();
    CHECK(decoder.Decode(anglesAtTurn0).IsValid);
    GearAngles anglesAtTurn2 = CreateAnglesFor(2u);
    position = decoder.Decode(anglesAtTurn2);
    CHECK(!position.IsValid);

    decoder.Reset();
    GearAngles anglesAtTurn7427 = CreateAnglesFor(GearConfig::TurnRange - 2u);
    CHECK(decoder.Decode(anglesAtTurn7427).IsValid);
    position = decoder.Decode(anglesAtTurn0);
    CHECK(!position.IsValid);

    decoder.Reset();
    GearAngles anglesAtTurn5 = CreateAnglesFor(5u);
    CHECK(decoder.Decode(anglesAtTurn5).IsValid);
    GearAngles anglesAtTurn7 = CreateAnglesFor(7u);
    position = decoder.Decode(anglesAtTurn7);
    CHECK(!position.IsValid);

    decoder.Reset();
    {
        GearAngles anglesAtTurn7Again = CreateAnglesFor(7u);
        CHECK(decoder.Decode(anglesAtTurn7Again).IsValid);
        GearAngles anglesAtTurn5Again = CreateAnglesFor(5u);
        position = decoder.Decode(anglesAtTurn5Again);
        CHECK(!position.IsValid);
    }
}

static void TestSlewGuardSemantics()
{
    std::printf("rejections clear status but tracking stays at the last accept\n");
    /* A whole turn keeps the raw tooth slots and the de-rotated ones aligned,
     * so the CRT oracle maps the sample directly. */
    const uint16_t referenceAngle = 0u;
    GearAngles angles = CreateAnglesAt(7u, referenceAngle);
    GearDecoder decoder;
    GearPosition position = decoder.Decode(angles);
    CHECK(position.IsValid);
    CHECK(position.Turns == 7u);
    CHECK(position.Angle == referenceAngle);

    /* One tooth ahead on the last driven gear: its residue shifts by the
     * modular inverse of the input teeth, so the decode jumps by the other
     * gears' product -- rejected; .Turns still carries the rejected
     * decode, .Angle echoes sun. */
    size_t lastGearIndex = GearConfig::DrivenCount - 1u;
    uint32_t lastGearTeeth = GearConfig::DrivenTeethCounts[lastGearIndex];
    uint32_t trueTooth =
        CountToTooth(GearAngleValue(angles, lastGearIndex), lastGearIndex);
    GearAngles misread = CreateAnglesAt(7u, referenceAngle);
    GearAngleField(misread, lastGearIndex) =
        ToothSlotLow((trueTooth + 1u) % lastGearTeeth, lastGearTeeth);
    uint32_t expectedMisread = ExpectedDecodedTurns(misread);
    /* A one-tooth slip must land beyond the slew limit. */
    CHECK(CircularDistance(expectedMisread, 7u) > GearConfig::MaxTurnsDelta);

    position = decoder.Decode(misread);
    CHECK(!position.IsValid);
    CHECK(position.Turns == expectedMisread);
    CHECK(position.Angle == referenceAngle);

    /* The guard still measures from turn 7, so 8 is accepted next. */
    GearAngles anglesAtTurn8 = CreateAnglesAt(8u, referenceAngle);
    position = decoder.Decode(anglesAtTurn8);
    CHECK(position.IsValid);
    CHECK(position.Turns == 8u);

    /* After Reset() any wild sample becomes the new reference. */
    angles = CreateAnglesAt(123u, referenceAngle);
    decoder.Reset();
    position = decoder.Decode(angles);
    CHECK(position.IsValid);
    CHECK(position.Turns == 123u);
}

static void TestMisreadInvariants()
{
    std::printf("misreads are either rejected or within one turn of the truth\n");
    /* The slew guard's contract can be checked as an equivalence: a decode
     * is accepted exactly when its circular distance from the previous
     * accepted sample is at most GearConfig::MaxTurnsDelta.  Perturb one
     * gear (or two, correlated) and verify the invariant holds. */
    const uint32_t referenceTurns[] = {7u, 4000u};
    const int32_t midSlotOffsets[] = {1800, -1800}; /* gear3-only, mid-gap */

    for (size_t referenceIndex = 0u;
         referenceIndex < sizeof(referenceTurns) / sizeof(referenceTurns[0]);
         referenceIndex++)
    {
        uint32_t referenceTurn = referenceTurns[referenceIndex];
        for (size_t gearIndex = 0u; gearIndex < GearConfig::DrivenCount;
             gearIndex++)
        {
            /* Full slot flips the tooth (jump of hundreds of turns); half
             * slot stays inside the same tooth (distance 0); half + 1 is
             * the first count of the next tooth. */
            int32_t fullSlotWidth =
                16384 / (int32_t)GearConfig::DrivenTeethCounts[gearIndex];
            int32_t halfSlotWidth =
                8192 / (int32_t)GearConfig::DrivenTeethCounts[gearIndex];
            const int32_t offsets[] = {
                fullSlotWidth, -fullSlotWidth,
                halfSlotWidth, -halfSlotWidth,
                halfSlotWidth + 1, -(halfSlotWidth + 1),
            };
            size_t offsetCount = sizeof(offsets) / sizeof(offsets[0]);

            if (gearIndex == 2u) /* the mid-gap disturbance from the old harness */
            {
                offsetCount = sizeof(midSlotOffsets) / sizeof(midSlotOffsets[0]);
                for (size_t offsetIndex = 0u; offsetIndex < offsetCount;
                     offsetIndex++)
                {
                    GearDecoder decoder;
                    GearAngles reference = CreateAnglesFor(referenceTurn);
                    CHECK(decoder.Decode(reference).IsValid);
                    GearAngles angles = CreateAnglesFor(referenceTurn);
                    GearAngleField(angles, gearIndex) =
                        AddOffset(GearAngleField(angles, gearIndex),
                                  midSlotOffsets[offsetIndex]);
                    GearPosition position = decoder.Decode(angles);
                    CHECK((CircularDistance(position.Turns, referenceTurn) <=
                           GearConfig::MaxTurnsDelta) == position.IsValid);
                }
            }
            else
            {
                for (size_t offsetIndex = 0u; offsetIndex < offsetCount;
                     offsetIndex++)
                {
                    GearDecoder decoder;
                    GearAngles reference = CreateAnglesFor(referenceTurn);
                    CHECK(decoder.Decode(reference).IsValid);
                    GearAngles angles = CreateAnglesFor(referenceTurn);
                    GearAngleField(angles, gearIndex) =
                        AddOffset(GearAngleField(angles, gearIndex),
                                  offsets[offsetIndex]);
                    GearPosition position = decoder.Decode(angles);
                    CHECK((CircularDistance(position.Turns, referenceTurn) <=
                           GearConfig::MaxTurnsDelta) == position.IsValid);
                }
            }
        }

        /* Correlated two-gear failures, one slot each. */
        const int32_t fullSlotOffsets[GearConfig::DrivenCount] = {
            16384 / (int32_t)GearConfig::DrivenTeethCounts[0],
            16384 / (int32_t)GearConfig::DrivenTeethCounts[1],
            16384 / (int32_t)GearConfig::DrivenTeethCounts[2],
        };
        {
            GearDecoder decoder;
            GearAngles angles = CreateAnglesFor(referenceTurn);
            GearAngles reference = CreateAnglesFor(referenceTurn);
            CHECK(decoder.Decode(reference).IsValid);
            GearAngleField(angles, 1u) =
                AddOffset(GearAngleField(angles, 1u), fullSlotOffsets[1]);
            GearAngleField(angles, 2u) =
                AddOffset(GearAngleField(angles, 2u), -fullSlotOffsets[2]);
            GearPosition position = decoder.Decode(angles);
            CHECK((CircularDistance(position.Turns, referenceTurn) <=
                   GearConfig::MaxTurnsDelta) == position.IsValid);
        }
        {
            GearDecoder decoder;
            GearAngles angles = CreateAnglesFor(referenceTurn);
            GearAngles reference = CreateAnglesFor(referenceTurn);
            CHECK(decoder.Decode(reference).IsValid);
            GearAngleField(angles, 0u) =
                AddOffset(GearAngleField(angles, 0u), -fullSlotOffsets[0]);
            GearAngleField(angles, 1u) =
                AddOffset(GearAngleField(angles, 1u), fullSlotOffsets[1]);
            GearPosition position = decoder.Decode(angles);
            CHECK((CircularDistance(position.Turns, referenceTurn) <=
                   GearConfig::MaxTurnsDelta) == position.IsValid);
        }
        {
            GearDecoder decoder;
            GearAngles angles = CreateAnglesFor(referenceTurn);
            GearAngles reference = CreateAnglesFor(referenceTurn);
            CHECK(decoder.Decode(reference).IsValid);
            GearAngleField(angles, 0u) =
                AddOffset(GearAngleField(angles, 0u), fullSlotOffsets[0]);
            GearAngleField(angles, 2u) =
                AddOffset(GearAngleField(angles, 2u), -fullSlotOffsets[2]);
            GearPosition position = decoder.Decode(angles);
            CHECK((CircularDistance(position.Turns, referenceTurn) <=
                   GearConfig::MaxTurnsDelta) == position.IsValid);
        }
    }
}

static void TestFullRangeSweep()
{
    std::printf("all %u states decode with in-slot offsets on every gear\n",
                (unsigned)GearConfig::TurnRange);
    /* Offsets stay inside every tooth slot: size them from the narrowest
     * slot (the gear with the most teeth), with a 2-count rounding margin,
     * so the same probe works for any valid geometry. */
    uint32_t maxTeeth = 0u;
    for (size_t i = 0u; i < GearConfig::DrivenCount; i++)
    {
        if (GearConfig::DrivenTeethCounts[i] > maxTeeth)
        {
            maxTeeth = GearConfig::DrivenTeethCounts[i];
        }
    }
    const int32_t maxOffset = (int32_t)(8192u / maxTeeth) - 2;
    CHECK(maxOffset > 0); /* premise: the half-slot covers the probe */
    const int32_t offsets[] = {-maxOffset, 0, maxOffset};
    for (uint32_t turns = 0u; turns < GearConfig::TurnRange; turns++)
    {
        for (size_t firstOffsetIndex = 0u;
             firstOffsetIndex < sizeof(offsets) / sizeof(offsets[0]);
             firstOffsetIndex++)
        {
            for (size_t secondOffsetIndex = 0u;
                 secondOffsetIndex < sizeof(offsets) / sizeof(offsets[0]);
                 secondOffsetIndex++)
            {
                for (size_t thirdOffsetIndex = 0u;
                     thirdOffsetIndex < sizeof(offsets) / sizeof(offsets[0]);
                     thirdOffsetIndex++)
                {
                    GearDecoder decoder;
                    GearAngles angles = CreateAnglesFor(turns);
                    GearAngleField(angles, 0u) =
                        AddOffset(GearAngleField(angles, 0u),
                                  offsets[firstOffsetIndex]);
                    GearAngleField(angles, 1u) =
                        AddOffset(GearAngleField(angles, 1u),
                                  offsets[secondOffsetIndex]);
                    GearAngleField(angles, 2u) =
                        AddOffset(GearAngleField(angles, 2u),
                                  offsets[thirdOffsetIndex]);
                    GearPosition position = decoder.Decode(angles);
                    CHECK(position.IsValid);
                    CHECK(position.Turns == turns);
                }
            }
        }
    }
}

int main()
{
    TestInit();
    TestExactDecode();
    TestMidTurnDecode();
    TestMidTurnTracking();
    TestGoldenSamples();
    TestPowerUpAnywhere();
    TestAllWithinTurnAngles();
    TestContinuousRotation();
    TestSensorNoise();
    TestReadSkew();
    TestAngleEdges();
    TestSlotQuantization();
    TestNoiseAndSmallSteps();
    TestWrapSeam();
    TestSlewGuardSemantics();
    TestMisreadInvariants();
    TestFullRangeSweep();

    std::printf("%d/%d checks passed\n", _totalChecks - _failedChecks,
                _totalChecks);
    return _failedChecks != 0 ? 1 : 0;
}