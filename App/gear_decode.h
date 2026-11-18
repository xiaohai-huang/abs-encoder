/**
 * @file gear_decode.h
 * @brief Absolute multi-turn position from coprime gear phases.
 *
 * The input shaft carries its own MT6701 (fine angle within one turn) and
 * each driven gear carries one (tooth counts in gear_config.h).  Because
 * the driven counts are pairwise coprime, the phase triple
 *
 *   phase_i(N) = (N * GearConfig::InputTeeth / teeth_i) mod 1
 *
 * is unique for N in 0..GearConfig::TurnRange-1, so every sample set
 * decodes to the absolute multi-turn position -- no counters, no storage,
 * battery-free by construction (docs/architecture.md).  A sample whose
 * position would jump more than GearConfig::MaxTurnsDelta from the
 * previous accepted one (misread gear, mixed-epoch sample) is rejected
 * and reported invalid.
 */
#ifndef APP_GEAR_DECODE_H
#define APP_GEAR_DECODE_H

#include <cstdint>

#include "gear_config.h" /* EncoderRole, GearConfig */

/* One angle sample per role; fields are named after the wheels (see the
 * role table in gear_config.h), so the sun gear and each driven gear are
 * associated explicitly and cannot be reordered. */
struct GearAngles
{
    uint16_t Sun;   /* Sun    -- input shaft, CSN1     */
    uint16_t Gear1; /* Gear1  -- 17 teeth, CSN2        */
    uint16_t Gear2; /* Gear2  -- 19 teeth, CSN3        */
    uint16_t Gear3; /* Gear3  -- 23 teeth, CSN4        */
};

struct GearPosition
{
    bool     IsValid; /* false: rejected by the slew guard */
    uint16_t Turns;   /* whole input-shaft turns, 0 .. GearConfig::TurnRange-1 */
    uint16_t Angle;   /* fine angle within the turn, 0 .. Mt6701::AngleMax */
};

/**
 * Absolute multi-turn decoder.
 *
 * The constructor validates the configured tooth counts (pairwise coprime
 * among the driven gears and with the input count) and derives the CRT
 * spotlights; decoding stays disabled until that passes (Decode reports
 * invalid, and IsInitialized exposes the geometry check result to the
 * caller).
 */
class GearDecoder
{
public:
    GearDecoder();

    /** True when the configured tooth counts form a usable gear set. */
    bool IsInitialized() const;

    /** Forget the tracking state; the next sample becomes the new reference. */
    void Reset();

    /**
     * Decode one set of four angle samples into the absolute position.
     * The sun angle is part of the solve: the driven gears also move with
     * the within-turn angle, so it is removed from their readings before
     * the turn count is recovered.
     * @param angles angle counts by role (.Sun, .Gear1..3).
     * @return position; .IsValid is false when the decoded position would
     *         jump more than GearConfig::MaxTurnsDelta from the previous
     *         accepted sample.  On rejection .Turns still carries that
     *         decoded-but-rejected count (do not use it) and .Angle echoes
     *         angles.Sun.  The first sample after construction/reset is
     *         always accepted.
     */
    GearPosition Decode(const GearAngles& angles);

private:
    bool ValidateGeometry();

    bool     _initialized;
    uint16_t _lastAcceptedTurns;
    bool     _hasPreviousReading;

    /* CRT spotlights: 1 modulo their own gear's tooth count, 0 modulo the
     * other two; built in the constructor. */
    uint16_t _spotlights[GearConfig::DrivenCount];

    /* inverse(InputTeeth) modulo TurnRange: turns the CRT's input-teeth
     * count into input revolutions. */
    uint16_t _inputTeethInverse;
};

#endif /* APP_GEAR_DECODE_H */