/**
 * @file gear_decode.cpp
 * @brief Multi-turn position decoder using three driven gears (17/19/23 teeth).
 *
 * HOW IT WORKS (plain language):
 *   1. Each sensor gives a raw angle (0..16383). Convert to a tooth number.
 *   2. Multiply each tooth number by its precomputed inverse so it represents
 *      the sun gear's turn count modulo that gear's tooth count.
 *   3. Combine the three residues via CRT into one absolute turn count.
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
 * Modular inverse of 'number' mod 'modulus' for small moduli.
 * Returns 0 if no inverse exists (not coprime).
 */
static uint16_t ComputeModularInverse(uint32_t number, uint32_t modulus)
{
    for (uint32_t candidate = 1u; candidate < modulus; candidate++)
    {
        if ((number * candidate) % modulus == 1u)
        {
            return static_cast<uint16_t>(candidate);
        }
    }
    return 0u;
}

/**
 * Combine two residues into one value via CRT.
 * Returns the unique N in [0, modulusA * modulusB) satisfying:
 *   N ≡ residueA (mod modulusA)
 *   N ≡ residueB (mod modulusB)
 */
static uint32_t CombineTwoResidues(uint32_t residueA, uint32_t modulusA,
                                   uint32_t residueB, uint32_t modulusB)
{
    uint32_t difference =
        (residueB + modulusB - (residueA % modulusB)) % modulusB;
    uint32_t multiplier =
        difference * ComputeModularInverse(modulusA % modulusB, modulusB) %
        modulusB;
    return residueA + modulusA * multiplier;
}

/**
 * Convert three raw sensor readings into an absolute sun-gear turn count.
 *
 * @param rawSensorCounts         Array of 3 raw encoder counts (0..16383),
 *                                ordered as gear1, gear2, gear3.
 * @param inputTeethInversePerGear Precomputed modular inverse of the input
 *                                teeth per driven gear (see ValidateGeometry).
 * @return Absolute turn count in [0, GearConfig::TurnRange).
 */
static uint16_t ComputeAbsoluteTurns(
    const uint16_t rawSensorCounts[GearConfig::DrivenCount],
    const uint16_t inputTeethInversePerGear[GearConfig::DrivenCount])
{
    uint32_t sunTurnResidues[GearConfig::DrivenCount];

    /* Step 1 & 2: Convert each raw reading → sun turn residue */
    for (size_t i = 0u; i < GearConfig::DrivenCount; i++)
    {
        /* Scale raw count to tooth position with rounding */
        uint32_t scaledCount =
            static_cast<uint32_t>(rawSensorCounts[i]) *
            GearConfig::DrivenTeethCounts[i];
        uint32_t toothNumber =
            ((scaledCount + SensorMaxCount / 2u) / SensorMaxCount) %
            GearConfig::DrivenTeethCounts[i];

        /* Convert gear tooth → sun turn count mod gear teeth */
        sunTurnResidues[i] =
            (toothNumber * inputTeethInversePerGear[i]) %
            GearConfig::DrivenTeethCounts[i];
    }

    /* Step 3: CRT — combine pairwise */
    uint32_t combinedPair = CombineTwoResidues(
        sunTurnResidues[0], GearConfig::DrivenTeethCounts[0],
        sunTurnResidues[1], GearConfig::DrivenTeethCounts[1]);
    uint32_t pairModulus = static_cast<uint32_t>(
        GearConfig::DrivenTeethCounts[0] * GearConfig::DrivenTeethCounts[1]);

    /* Extend to third gear */
    uint32_t thirdModulus = GearConfig::DrivenTeethCounts[2];
    uint32_t thirdDifference =
        ((sunTurnResidues[2] + thirdModulus) - (combinedPair % thirdModulus)) %
        thirdModulus;
    uint32_t thirdMultiplier =
        thirdDifference *
        ComputeModularInverse(pairModulus % thirdModulus, thirdModulus) %
        thirdModulus;
    uint32_t absoluteTurns = combinedPair + pairModulus * thirdMultiplier;

    return static_cast<uint16_t>(absoluteTurns);
}

/* ---------- GearDecoder ---------- */

GearDecoder::GearDecoder()
    : _initialized(false)
    , _lastAcceptedTurns(0u)
    , _hasPreviousReading(false)
    , _inputTeethInversePerGear{}
{
    _initialized = ValidateGeometry();
}

bool GearDecoder::IsInitialized() const
{
    return _initialized;
}

bool GearDecoder::ValidateGeometry()
{
    for (size_t i = 0u; i < GearConfig::DrivenCount; i++)
    {
        /* Every pair of driven gears must be coprime */
        for (size_t j = i + 1u; j < GearConfig::DrivenCount; j++)
        {
            if (GreatestCommonDivisor(GearConfig::DrivenTeethCounts[i],
                                      GearConfig::DrivenTeethCounts[j]) != 1u)
            {
                return false;
            }
        }
        /* Input (sun) teeth must be coprime with each driven gear */
        if (GreatestCommonDivisor(GearConfig::InputTeeth,
                                  GearConfig::DrivenTeethCounts[i]) != 1u)
        {
            return false;
        }
        /* Precompute the inverse for this gear */
        _inputTeethInversePerGear[i] = ComputeModularInverse(
            GearConfig::InputTeeth % GearConfig::DrivenTeethCounts[i],
            GearConfig::DrivenTeethCounts[i]);
        if (_inputTeethInversePerGear[i] == 0u)
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

    /* Decode absolute turn count from three driven-gear sensors */
    uint16_t currentTurns =
        ComputeAbsoluteTurns(rawSensorCounts, _inputTeethInversePerGear);
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