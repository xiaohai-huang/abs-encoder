/**
 * @file gear_decode.c
 * @brief Multi-turn position decoder using three planet gears (17/19/23 teeth).
 *
 * HOW IT WORKS (plain language):
 *   1. Each sensor gives a raw angle (0..16383). Convert to a tooth number.
 *   2. Multiply each tooth number by its precomputed inverse so it represents
 *      the sun gear's turn count modulo that planet's tooth count.
 *   3. Combine the three residues via CRT into one absolute turn count.
 *   4. Reject physically impossible jumps (slew guard).
 */
#include "gear_decode.h"
#include <stddef.h>

/* ---------- Hardware constants ---------- */
#define SENSOR_BITS       14u                /* MT6701 resolution */
#define SENSOR_MAX_COUNT  (1u << SENSOR_BITS) /* 16384 */

/* ---------- Gear geometry (from gear_config.h) ---------- */
static const uint16_t planet_teeth[GEAR_DRIVEN_COUNT] = GEAR_DRIVEN_TEETH;

/* Precomputed: modular inverse of SUN_TEETH mod each planet's tooth count.
 * Converts "which planet tooth is engaged" → "sun turn count mod planet_teeth". */
static uint16_t sun_inverse_per_planet[GEAR_DRIVEN_COUNT];

static bool     system_initialized;
static uint16_t last_accepted_turns;
static bool     have_previous_reading;

/* ---------- Compile-time safety checks ---------- */
_Static_assert(GEAR_DRIVEN_COUNT == MT6701_ENC_COUNT - 1u,
               "Expected one driven gear per auxiliary encoder");
_Static_assert(ENC_COUNT == MT6701_ENC_COUNT,
               "Expected one role per encoder");

/* ---------- Math helpers ---------- */

static uint32_t gcd(uint32_t a, uint32_t b)
{
    while (b != 0u) {
        uint32_t remainder = a % b;
        a = b;
        b = remainder;
    }
    return a;
}

/**
 * Modular inverse of 'value' mod 'modulus' for small moduli.
 * Returns 0 if no inverse exists (not coprime).
 */
static uint16_t modular_inverse(uint32_t value, uint32_t modulus)
{
    for (uint32_t candidate = 1u; candidate < modulus; candidate++) {
        if ((value * candidate) % modulus == 1u) {
            return (uint16_t)candidate;
        }
    }
    return 0u;
}

/* ---------- Initialization ---------- */

static bool validate_gear_geometry(void)
{
    for (size_t i = 0u; i < GEAR_DRIVEN_COUNT; i++) {
        /* Every pair of planet gears must be coprime */
        for (size_t j = i + 1u; j < GEAR_DRIVEN_COUNT; j++) {
            if (gcd(planet_teeth[i], planet_teeth[j]) != 1u) {
                return false;
            }
        }
        /* Sun gear teeth must be coprime with each planet */
        if (gcd(GEAR_INPUT_TEETH, planet_teeth[i]) != 1u) {
            return false;
        }
        /* Precompute the inverse for this planet */
        sun_inverse_per_planet[i] = modular_inverse(
            GEAR_INPUT_TEETH % planet_teeth[i],
            planet_teeth[i]
        );
        if (sun_inverse_per_planet[i] == 0u) {
            return false;
        }
    }
    return true;
}

bool gear_decode_init(void)
{
    if (!system_initialized) {
        system_initialized = validate_gear_geometry();
    }
    return system_initialized;
}

void gear_decode_reset(void)
{
    have_previous_reading = false;
}

/* ---------- Core decoding ---------- */

/**
 * Combine two residues into one value via CRT.
 * Returns the unique N in [0, modulus_a * modulus_b) satisfying:
 *   N ≡ residue_a (mod modulus_a)
 *   N ≡ residue_b (mod modulus_b)
 */
static uint32_t crt_combine_two(
    uint32_t residue_a, uint32_t modulus_a,
    uint32_t residue_b, uint32_t modulus_b)
{
    uint32_t diff = (residue_b + modulus_b - (residue_a % modulus_b)) % modulus_b;
    uint32_t k = diff * modular_inverse(modulus_a % modulus_b, modulus_b) % modulus_b;
    return residue_a + modulus_a * k;
}

/**
 * Convert three raw sensor readings into an absolute sun-gear turn count.
 *
 * @param raw_sensor_counts  Array of 3 raw encoder counts (0..16383),
 *                           ordered as gear1, gear2, gear3.
 * @return Absolute turn count in [0, GEAR_TURN_RANGE).
 */
static uint16_t compute_absolute_turns(const uint16_t raw_sensor_counts[GEAR_DRIVEN_COUNT])
{
    uint32_t sun_residue[GEAR_DRIVEN_COUNT];

    /* Step 1 & 2: Convert each raw reading → sun turn residue */
    for (size_t i = 0u; i < GEAR_DRIVEN_COUNT; i++) {
        /* Scale raw count to tooth position with rounding */
        uint32_t scaled = (uint32_t)raw_sensor_counts[i] * planet_teeth[i];
        uint32_t tooth_number = ((scaled + SENSOR_MAX_COUNT / 2u) / SENSOR_MAX_COUNT)
                                % planet_teeth[i];

        /* Convert planet tooth → sun turn count mod planet_teeth[i] */
        sun_residue[i] = (tooth_number * sun_inverse_per_planet[i]) % planet_teeth[i];
    }

    /* Step 3: CRT — combine pairwise */
    uint32_t combined_ab = crt_combine_two(
        sun_residue[0], planet_teeth[0],
        sun_residue[1], planet_teeth[1]
    );
    uint32_t modulus_ab = planet_teeth[0] * planet_teeth[1];

    /* Extend to third planet */
    uint32_t t = planet_teeth[2];
    uint32_t diff_c = ((sun_residue[2] + t) - (combined_ab % t)) % t;
    uint32_t k_c = diff_c * modular_inverse(modulus_ab % t, t) % t;
    uint32_t absolute_turns = combined_ab + modulus_ab * k_c;

    return (uint16_t)absolute_turns;
}

/* ---------- Public API ---------- */

gear_pos_t gear_decode(const gear_angles_t *angles)
{
    gear_pos_t result = {
        .valid = false,
        .turns = 0u,
        .angle = angles->sun
    };

    /* Map struct fields to ordered array matching planet_teeth[] */
    const uint16_t raw_sensor_counts[GEAR_DRIVEN_COUNT] = {
        angles->gear1,
        angles->gear2,
        angles->gear3
    };

    if (!gear_decode_init()) {
        return result;
    }

    /* Decode absolute turn count from three planet sensors */
    uint16_t current_turns = compute_absolute_turns(raw_sensor_counts);
    result.turns = current_turns;

    /* Step 4: Slew guard — reject physically impossible jumps */
    if (have_previous_reading) {
        uint32_t forward_distance =
            (current_turns + (uint32_t)GEAR_TURN_RANGE - last_accepted_turns)
            % (uint32_t)GEAR_TURN_RANGE;

        /* Use shortest circular distance */
        uint32_t distance = forward_distance;
        if (distance > GEAR_TURN_RANGE / 2u) {
            distance = GEAR_TURN_RANGE - distance;
        }

        if (distance > GEAR_MAX_TURNS_DELTA) {
            /* Implausible jump — likely a sensor misread. Keep last position. */
            return result;
        }
    }

    /* Accept this reading */
    last_accepted_turns = current_turns;
    have_previous_reading = true;
    result.valid = true;
    return result;
}