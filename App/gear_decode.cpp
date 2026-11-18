/**
 * @file gear_decode.cpp
 * @brief Multi-turn position decoder using three driven gears (tooth counts
 *        are declared in gear_config.h).
 *
 * HOW IT WORKS (plain language):
 *   1. Each MT6701 measures one revolution of its gear.  Scaled by the tooth
 *      count, that angle gives how far the gear has advanced in teeth.  At
 *      input position N + theta the gear is pushed by InputTeeth * (N + theta)
 *      teeth, so the decoder first removes the sun's own within-turn share,
 *      InputTeeth * theta (read by the Sun encoder).  What remains is the
 *      input teeth passed by whole turns, modulo that gear's tooth count:
 *
 *        tooth_i = (N * InputTeeth) mod teeth_i
 *
 *   2. The Chinese Remainder Theorem combines the three tooth numbers into
 *      the unique number of input teeth that have passed, modulo the turn
 *      range M.  With spotlights e_i -- 1 modulo teeth_i, 0 modulo the other
 *      two -- the whole CRT is one weighted sum:
 *
 *        y = (sum tooth_i * e_i) mod M
 *
 *   3. y counts input teeth, so one modular multiply turns it into input
 *      turns; the multiplier is inverse(InputTeeth mod M):
 *
 *        N = y * inverse(InputTeeth, M) mod M
 *
 *   4. Reject physically impossible jumps (slew guard).
 */
#include "gear_decode.h"

#include <cstddef>

#include "mt6701.h" /* Mt6701::EncoderCount (static asserts) */

/* ---------- Hardware constants ---------- */
static constexpr uint16_t SensorBits     = 14u;                /* MT6701 resolution */
static constexpr uint16_t SensorMaxCount = (1u << SensorBits); /* 16384 */

static_assert(GearConfig::DrivenCount == Mt6701::EncoderCount - 1u,
              "Expected one driven gear per auxiliary encoder");
static_assert(static_cast<int>(EncoderRole::RoleCount) ==
                  Mt6701::EncoderCount,
              "Expected one role per encoder");

/* ---------- Math helpers ---------- */

static uint32_t GreatestCommonDivisor(uint32_t first, uint32_t second)
{
    while (second != 0u)
    {
        uint32_t remainder = first % second;
        first = second;
        second = remainder;
    }
    return first;
}

/**
 * Modular inverse of 'number' mod 'modulus' via the extended Euclidean
 * algorithm.  Returns 0 when no inverse exists (not coprime).
 */
static uint16_t ComputeModularInverse(uint32_t number, uint32_t modulus)
{
    int32_t modulusSigned = static_cast<int32_t>(modulus);
    int32_t oldRemainder = static_cast<int32_t>(number % modulus);
    int32_t remainder = modulusSigned;
    int32_t oldCoefficient = 1; /* coefficient of 'number' in oldRemainder */
    int32_t coefficient = 0;

    while (remainder != 0)
    {
        int32_t quotient = oldRemainder / remainder;

        int32_t nextRemainder = oldRemainder - quotient * remainder;
        oldRemainder = remainder;
        remainder = nextRemainder;

        int32_t nextCoefficient = oldCoefficient - quotient * coefficient;
        oldCoefficient = coefficient;
        coefficient = nextCoefficient;
    }

    if (oldRemainder != 1)
    {
        return 0u; /* not coprime */
    }
    return static_cast<uint16_t>(
        (oldCoefficient % modulusSigned + modulusSigned) % modulusSigned);
}

/**
 * CRT spotlight for one driven gear: the number that is congruent to 1
 * modulo that gear's teeth and to 0 modulo the other gears' tooth counts.
 *
 *   e = (M / teeth) * inverse(M / teeth mod teeth)
 *
 * M / teeth already contains the other tooth counts, so it vanishes on their
 * clocks; multiplying by its inverse makes it read 1 on its own.
 */
static uint16_t ComputeSpotlight(uint16_t teeth)
{
    uint32_t cofactor = GearConfig::TurnRange / teeth;
    uint16_t inverse = ComputeModularInverse(cofactor, teeth);

    /* inverse < teeth, so cofactor * inverse < M: already reduced. */
    return static_cast<uint16_t>(cofactor * inverse);
}

/**
 * Tooth number of one driven gear: the raw angle scaled to the gear's teeth,
 * with the sun's within-turn motion removed, then rounded to the nearest
 * tooth.  Without the subtraction, the fraction of the current turn that the
 * driven gears have already travelled would be mistaken for whole turns (see
 * the file overview).
 *
 * The raw angle is scaled by `teeth` (1/SensorMaxCount tooth units); the sun
 * contributes InputTeeth * sunSensorCount of the same units, because one sun
 * turn advances every driven gear by InputTeeth teeth.
 */
static uint16_t ComputeToothNumber(uint16_t rawSensorCount, uint16_t teeth,
                                   uint16_t sunSensorCount)
{
    int32_t toothOffset =
        static_cast<int32_t>(rawSensorCount) * teeth -
        static_cast<int32_t>(GearConfig::InputTeeth) * sunSensorCount;

    /* Round to the nearest tooth (half away from zero), then wrap into
     * [0, teeth): a reading at the top of the circle maps to tooth 0. */
    int32_t halfSensorRange = SensorMaxCount / 2;
    int32_t toothNumber;
    if (toothOffset >= 0)
    {
        toothNumber = (toothOffset + halfSensorRange) / SensorMaxCount;
    }
    else
    {
        toothNumber = -((-toothOffset + halfSensorRange) / SensorMaxCount);
    }

    toothNumber %= teeth;
    if (toothNumber < 0)
    {
        toothNumber += teeth;
    }
    return static_cast<uint16_t>(toothNumber);
}

/**
 * Convert three raw sensor readings into an absolute sun-gear turn count.
 *
 * @param rawSensorCounts   Raw encoder counts (0..16383), ordered as
 *                          gear1, gear2, gear3.
 * @param spotlights        Precomputed CRT spotlights, same order.
 * @param inputTeethInverse inverse(InputTeeth) modulo TurnRange.
 * @param sunSensorCount    Raw sun angle (0..16383); its within-turn share
 *                          of the driven gears' motion is removed first.
 * @return Absolute turn count in [0, GearConfig::TurnRange).
 */
static uint16_t ComputeAbsoluteTurns(
    const uint16_t rawSensorCounts[GearConfig::DrivenCount],
    const uint16_t spotlights[GearConfig::DrivenCount],
    uint16_t inputTeethInverse, uint16_t sunSensorCount)
{
    /* CRT: weight each tooth number by its gear's spotlight and add.  On
     * gear i's own clock the other spotlights are 0 and its own is 1, so
     * the sum reads tooth number i there -- all three clues at once. */
    uint32_t inputTeethPassed = 0u;
    for (size_t i = 0u; i < GearConfig::DrivenCount; i++)
    {
        uint16_t toothNumber =
            ComputeToothNumber(rawSensorCounts[i],
                               GearConfig::DrivenTeethCounts[i],
                               sunSensorCount);
        inputTeethPassed += static_cast<uint32_t>(toothNumber) * spotlights[i];
    }
    inputTeethPassed %= GearConfig::TurnRange;

    /* The sum counts input teeth; dividing by the input tooth count gives
     * turns.  InputTeeth is coprime with the range, so the division is an
     * exact modular multiply. */
    return static_cast<uint16_t>(inputTeethPassed * inputTeethInverse %
                                 GearConfig::TurnRange);
}

/* ---------- GearDecoder ---------- */

GearDecoder::GearDecoder()
    : _initialized(false)
    , _lastAcceptedTurns(0u)
    , _hasPreviousReading(false)
    , _spotlights{}
    , _inputTeethInverse(0u)
{
    _initialized = ValidateGeometry();
    if (!_initialized)
    {
        return;
    }

    /* Geometry fixed at compile time: derive the CRT weights once. */
    for (size_t i = 0u; i < GearConfig::DrivenCount; i++)
    {
        _spotlights[i] = ComputeSpotlight(GearConfig::DrivenTeethCounts[i]);
    }
    _inputTeethInverse =
        ComputeModularInverse(GearConfig::InputTeeth, GearConfig::TurnRange);
}

bool GearDecoder::IsInitialized() const
{
    return _initialized;
}

bool GearDecoder::ValidateGeometry()
{
    for (size_t i = 0u; i < GearConfig::DrivenCount; i++)
    {
        /* CRT moduli: every pair of driven gears must be coprime. */
        for (size_t j = i + 1u; j < GearConfig::DrivenCount; j++)
        {
            if (GreatestCommonDivisor(GearConfig::DrivenTeethCounts[i],
                                      GearConfig::DrivenTeethCounts[j]) != 1u)
            {
                return false;
            }
        }
        /* The final division by the input count needs it invertible too:
         * coprime with every driven count (and so with their product). */
        if (GreatestCommonDivisor(GearConfig::InputTeeth,
                                  GearConfig::DrivenTeethCounts[i]) != 1u)
        {
            return false;
        }
    }
    return true;
}

void GearDecoder::Reset()
{
    _hasPreviousReading = false;
}

GearPosition GearDecoder::Decode(const GearAngles& angles)
{
    GearPosition result;
    result.IsValid = false;
    result.Turns = 0u;
    result.Angle = angles.Sun;

    if (!_initialized)
    {
        return result;
    }

    /* Map struct fields to ordered array matching DrivenTeethCounts[] */
    const uint16_t rawSensorCounts[GearConfig::DrivenCount] = {
        angles.Gear1, angles.Gear2, angles.Gear3};

    /* Decode the absolute turn count; the sun angle strips the driven gears'
     * within-turn motion before the CRT. */
    uint16_t currentTurns = ComputeAbsoluteTurns(
        rawSensorCounts, _spotlights, _inputTeethInverse, angles.Sun);
    result.Turns = currentTurns;

    /* Step 4: Slew guard — reject physically impossible jumps */
    if (_hasPreviousReading)
    {
        uint32_t forwardDistance =
            (currentTurns + GearConfig::TurnRange - _lastAcceptedTurns) %
            GearConfig::TurnRange;

        /* Use shortest circular distance */
        uint32_t shortestDistance = forwardDistance;
        if (shortestDistance > GearConfig::TurnRange / 2u)
        {
            shortestDistance = GearConfig::TurnRange - shortestDistance;
        }

        if (shortestDistance > GearConfig::MaxTurnsDelta)
        {
            /* Implausible jump — likely a sensor misread. Keep last position. */
            return result;
        }
    }

    /* Accept this reading */
    _lastAcceptedTurns = currentTurns;
    _hasPreviousReading = true;
    result.IsValid = true;
    return result;
}
