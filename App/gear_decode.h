/**
 * @file gear_decode.h
 * @brief Absolute multi-turn position from coprime gear phases.
 *
 * The input shaft carries its own MT6701 (fine angle within one turn) and
 * each driven gear carries one (tooth counts in gear_config.h).  Because
 * the driven counts are pairwise coprime, the phase triple
 *
 *   phase_i(N) = (N * GEAR_INPUT_TEETH / teeth_i) mod 1
 *
 * is unique for N in 0..GEAR_TURN_RANGE-1, so every sample set decodes to
 * the absolute multi-turn position -- no counters, no storage, battery-free
 * by construction (docs/architecture.md).  A sample whose position would
 * jump more than GEAR_MAX_TURNS_DELTA from the previous accepted one
 * (misread gear, mixed-epoch sample) is rejected and reported invalid.
 */
#ifndef APP_GEAR_DECODE_H
#define APP_GEAR_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#include "gear_config.h" /* GEAR_*, encoder_role_t */
#include "mt6701.h"      /* MT6701_ENC_COUNT, MT6701_ANGLE_MAX */

/* One angle sample per role; fields are named after the wheels (see the
 * role table in gear_config.h), so the sun gear and each driven gear are
 * associated explicitly and cannot be reordered. */
typedef struct
{
    uint16_t sun;   /* ENC_SUN    -- input shaft, CSN4     */
    uint16_t gear1; /* ENC_GEAR_1 -- 17 teeth, CSN1        */
    uint16_t gear2; /* ENC_GEAR_2 -- 19 teeth, CSN2        */
    uint16_t gear3; /* ENC_GEAR_3 -- 23 teeth, CSN3        */
} gear_angles_t;

typedef struct
{
    bool     valid; /* false: rejected by the slew guard */
    uint16_t turns; /* whole input-shaft turns, 0 .. GEAR_TURN_RANGE-1 */
    uint16_t angle; /* fine angle within the turn, 0 .. MT6701_ANGLE_MAX */
} gear_pos_t;

/**
 * Validate the configured tooth counts: pairwise coprime among the driven
 * gears and with the input count.  Decoding stays disabled until this
 * passes.
 * @return true when the gear set is usable.
 */
bool gear_decode_init(void);

/** Forget the tracking state; the next sample becomes the new reference. */
void gear_decode_reset(void);

/**
 * Decode one set of four angle samples into the absolute position.
 * @param a angle counts by role (.sun, .gear1..3).
 * @return position; .valid is false when the decoded position would jump
 *         more than GEAR_MAX_TURNS_DELTA from the previous sample (the
 *         last accepted position is held and reported).  The first sample
 *         after init/reset is always accepted.
 */
gear_pos_t gear_decode(const gear_angles_t *a);

#endif /* APP_GEAR_DECODE_H */