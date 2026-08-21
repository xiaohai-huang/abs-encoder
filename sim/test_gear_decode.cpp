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
#include "mt6701.h" /* Mt6701::AngleMax */

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

/* Angle of driven gear `gearTeeth` after `turns` input turns, rounded to the
 * nearest tooth-slot center. */
static uint16_t ComputeGearAngle(uint32_t turns, uint32_t gearTeeth)
{
    uint32_t phase = (GearConfig::InputTeeth * turns) % gearTeeth;
    return static_cast<uint16_t>(
        (phase * 16384u + gearTeeth / 2u) / gearTeeth);
}

/* Sample set that decodes to `turns`; the sun angle is arbitrary.  Fields
 * are positional: Sun, Gear1, Gear2, Gear3 (see the struct in gear_decode.h). */
static GearAngles CreateAnglesFor(uint32_t turns)
{
    GearAngles angles = {
        0u, /* Sun */
        ComputeGearAngle(turns, GearConfig::DrivenTeethCounts[0]),
        ComputeGearAngle(turns, GearConfig::DrivenTeethCounts[1]),
        ComputeGearAngle(turns, GearConfig::DrivenTeethCounts[2])};
    return angles;
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
 * decoder's closed-form pairing so it can validate it. */
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
    std::printf("exact decodes across the range, sun angle passthrough\n");
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
            GearAngles angles = CreateAnglesFor(turnCounts[turnIndex]);
            angles.Sun = sunAngles[sunAngleIndex];
            GearDecoder decoder;
            GearPosition position = decoder.Decode(angles);
            CHECK(position.IsValid);
            CHECK(position.Turns == turnCounts[turnIndex]);
            CHECK(position.Angle == sunAngles[sunAngleIndex]);
        }
    }
}

static void TestAngleEdges()
{
    std::printf("raw-count extremes 0/1/16382/16383 stay inside tooth 0\n");
    /* With a reference at 0 every gear sits in tooth 0, and tooth 0's raw
     * range spans both ends of the 14-bit angle: the low tail [0, ~8192/teeth]
     * and the high tail [16384 - 8192/teeth, 16383] (the wrap always lands
     * in tooth 0 for any tooth count below 8192).  All four fields may
     * therefore take the extreme counts without changing the decode. */
    const uint16_t edgeCounts[] = {0u, 1u, Mt6701::AngleMax - 1u,
                                   Mt6701::AngleMax};
    for (size_t fieldIndex = 0u; fieldIndex < 4u; fieldIndex++)
    {
        for (size_t edgeIndex = 0u;
             edgeIndex < sizeof(edgeCounts) / sizeof(edgeCounts[0]);
             edgeIndex++)
        {
            GearAngles angles = CreateAnglesFor(0u);
            switch (fieldIndex)
            {
            case 0u: angles.Sun = edgeCounts[edgeIndex];   break;
            case 1u: angles.Gear1 = edgeCounts[edgeIndex]; break;
            case 2u: angles.Gear2 = edgeCounts[edgeIndex]; break;
            default: angles.Gear3 = edgeCounts[edgeIndex]; break;
            }
            GearDecoder decoder;
            GearPosition position = decoder.Decode(angles);
            CHECK(position.IsValid);
            CHECK(position.Turns == 0u);
            CHECK(position.Angle == angles.Sun);
        }
    }

    /* All four extremes at once still decode to 0. */
    GearDecoder decoder;
    GearAngles allExtremes = {Mt6701::AngleMax, Mt6701::AngleMax,
                              Mt6701::AngleMax, Mt6701::AngleMax};
    GearPosition position = decoder.Decode(allExtremes);
    CHECK(position.IsValid);
    CHECK(position.Turns == 0u);
    CHECK(position.Angle == Mt6701::AngleMax);

    /* One count past the slot edge of tooth 0 is the first count of tooth
     * 1: gear1 crossing the edge alone decodes to a misread turn count.
     * Fresh from reset it is accepted as a reference; with a reference at
     * 0 the guard rejects it.  The expected value comes from the CRT
     * oracle, so it tracks whatever geometry GearConfig declares. */
    const uint16_t lastCountOfTooth0 =
        ToothSlotHigh(0u, GearConfig::DrivenTeethCounts[0]);
    const uint16_t firstCountOfTooth1 =
        ToothSlotLow(1u, GearConfig::DrivenTeethCounts[0]);

    GearAngles misreadAngles = {0u, firstCountOfTooth1, Mt6701::AngleMax,
                                Mt6701::AngleMax};
    uint32_t expectedMisread = ExpectedDecodedTurns(misreadAngles);
    /* A single-tooth slip on a driven gear jumps by at least the product of
     * the other two gears: the guard rejection below is meaningful. */
    CHECK(CircularDistance(expectedMisread, 0u) > GearConfig::MaxTurnsDelta);

    GearDecoder freshDecoder;
    position = freshDecoder.Decode(misreadAngles);
    CHECK(position.IsValid);
    CHECK(position.Turns == expectedMisread);

    GearAngles anglesAtTurn0 = CreateAnglesFor(0u);
    decoder.Reset();
    CHECK(decoder.Decode(anglesAtTurn0).IsValid); /* reference at turn 0 */
    allExtremes.Gear1 = lastCountOfTooth0;        /* last count of tooth 0 */
    position = decoder.Decode(allExtremes);
    CHECK(position.IsValid);
    CHECK(position.Turns == 0u);

    allExtremes.Gear1 = firstCountOfTooth1; /* first count of tooth 1 */
    position = decoder.Decode(allExtremes);
    CHECK(!position.IsValid);
    /* .Turns carries the decoded-but-rejected reading: the guard rejects
     * it, it was never accepted as the position */
    CHECK(position.Turns == expectedMisread);
    CHECK(position.Angle == Mt6701::AngleMax);
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
    GearAngles angles = CreateAnglesFor(7u);
    angles.Sun = 0x55AAu;
    GearDecoder decoder;
    CHECK(decoder.Decode(angles).IsValid);

    /* One tooth ahead on the last driven gear: its residue shifts by the
     * modular inverse of the input teeth, so the decode jumps by the other
     * gears' product -- rejected; .Turns still carries the rejected
     * decode, .Angle echoes sun. */
    size_t lastGearIndex = GearConfig::DrivenCount - 1u;
    uint32_t lastGearTeeth = GearConfig::DrivenTeethCounts[lastGearIndex];
    uint32_t trueTooth = (GearConfig::InputTeeth * 7u) % lastGearTeeth;
    GearAngles misread = CreateAnglesFor(7u);
    misread.Sun = 0x55AAu; /* the rejected read still echoes the sun angle */
    GearAngleField(misread, lastGearIndex) =
        ToothSlotLow((trueTooth + 1u) % lastGearTeeth, lastGearTeeth);
    uint32_t expectedMisread = ExpectedDecodedTurns(misread);
    /* A one-tooth slip must land beyond the slew limit. */
    CHECK(CircularDistance(expectedMisread, 7u) > GearConfig::MaxTurnsDelta);

    GearPosition position = decoder.Decode(misread);
    CHECK(!position.IsValid);
    CHECK(position.Turns == expectedMisread);
    CHECK(position.Angle == 0x55AAu);

    /* The guard still measures from turn 7, so 8 is accepted next. */
    GearAngles anglesAtTurn8 = CreateAnglesFor(8u);
    position = decoder.Decode(anglesAtTurn8);
    CHECK(position.IsValid);
    CHECK(position.Turns == 8u);

    /* After Reset() any wild sample becomes the new reference. */
    angles = CreateAnglesFor(123u);
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