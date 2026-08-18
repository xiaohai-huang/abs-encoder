/**
 * @file gear_decode.c
 * @brief Multi-turn decode: turn count N in 0..GEAR_TURN_RANGE-1 from the
 *        three driven-gear phases via modular arithmetic (CRT).
 *
 * Per gear i, phase_i = (N * GEAR_INPUT_TEETH / teeth_i) mod 1, so the
 * nearest slot q_i = round(teeth_i * phase_i) satisfies
 * q_i == N * GEAR_INPUT_TEETH (mod teeth_i).  With GEAR_INPUT_TEETH
 * coprime to every teeth_i the residue is r_i = q_i * inv(input, teeth_i)
 * mod teeth_i; the residues combine into N mod t_1*t_2*t_3 via the Chinese
 * remainder theorem.
 *
 * Misread detection is temporal, not per-sample: within one sample a
 * misread gear is indistinguishable from a genuine neighbouring state
 * (both are nearest-slot-consistent -- the quantization cell is the
 * information unit).  What a misread always breaks is continuity: it
 * decodes to a jump of a multiple of the other gears' product (e.g. 323
 * turns for a 17/19/23 set), which no real shaft can make between
 * samples.  A slew guard therefore rejects any accepted position that
 * moves more than GEAR_MAX_TURNS_DELTA (circularly) from the last one.
 */
#include "gear_decode.h"

#include <stddef.h>

#define GEAR_ENC_BITS 14u /* MT6701 angle resolution */
#define GEAR_ENC_STEPS (1u << GEAR_ENC_BITS)

/* Driven-gear teeth in config order: s_teeth[0] is ENC_GEAR_1 (17T),
 * [1] ENC_GEAR_2 (19T), [2] ENC_GEAR_3 (23T) -- see gear_config.h. */
static const uint16_t s_teeth[GEAR_DRIVEN_COUNT] = GEAR_DRIVEN_TEETH;
static uint16_t s_inv_input[GEAR_DRIVEN_COUNT]; /* inv(input mod t_i, t_i) */
static bool     s_ready;

static uint16_t s_last_turns; /* last accepted position (slew reference) */
static bool     s_have_last;

_Static_assert(GEAR_DRIVEN_COUNT == MT6701_ENC_COUNT - 1u,                  \
               "one driven gear per auxiliary encoder");
_Static_assert(ENC_COUNT == MT6701_ENC_COUNT,                               \
               "one role per encoder");

static uint32_t gcd_u32(uint32_t a, uint32_t b)
{
    while (b != 0u)
    {
        uint32_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

/* Modular inverse of a mod m for 1 <= a < m with m small (bounded by
 * GEAR_TURN_RANGE <= 8192 via the config static assert). */
static uint16_t mod_inv(uint32_t a, uint32_t m)
{
    for (uint32_t x = 1u; x < m; x++)
    {
        if ((a * x) % m == 1u)
        {
            return (uint16_t)x;
        }
    }
    return 0u; /* not invertible: not coprime */
}

static bool configure(void)
{
    for (size_t i = 0u; i < GEAR_DRIVEN_COUNT; i++)
    {
        /* driven counts pairwise coprime and each coprime with the input
         * count, or the residues alias and decoding is ambiguous */
        for (size_t j = i + 1u; j < GEAR_DRIVEN_COUNT; j++)
        {
            if (gcd_u32(s_teeth[i], s_teeth[j]) != 1u)
            {
                return false;
            }
        }
        if (gcd_u32(GEAR_INPUT_TEETH, s_teeth[i]) != 1u)
        {
            return false;
        }
        s_inv_input[i] = mod_inv(GEAR_INPUT_TEETH % s_teeth[i], s_teeth[i]);
        if (s_inv_input[i] == 0u)
        {
            return false;
        }
    }
    return true;
}

bool gear_decode_init(void)
{
    if (!s_ready)
    {
        s_ready = configure();
    }
    return s_ready;
}

void gear_decode_reset(void)
{
    s_have_last = false;
}

/* Unique N in 0..t0*t1-1 with residues (r0, r1) modulo (t0, t1); the
 * moduli must be coprime (enforced by configure). */
static uint32_t solve_pair(uint32_t r0, uint32_t t0, uint32_t r1, uint32_t t1)
{
    uint32_t k = ((r1 + t1 - (r0 % t1)) % t1) * mod_inv(t0 % t1, t1) % t1;
    return r0 + t0 * k;
}

/* The position consistent with the three sampled phases; every residue
 * triple has exactly one solution in 0..GEAR_TURN_RANGE-1, so this always
 * returns a value -- correctness of the *combination* is guaranteed by the
 * coprime geometry, and correctness of the *identification* is guarded by
 * the slew logic below.  g[] follows the config order of the driven
 * gears (see s_teeth). */
static uint16_t decode_turns(const uint16_t g[GEAR_DRIVEN_COUNT])
{
    uint32_t r[GEAR_DRIVEN_COUNT];
    for (size_t i = 0u; i < GEAR_DRIVEN_COUNT; i++)
    {
        uint32_t slots = (uint32_t)g[i] * s_teeth[i];
        uint32_t q = ((slots + GEAR_ENC_STEPS / 2u) / GEAR_ENC_STEPS) % s_teeth[i];
        r[i] = (q * s_inv_input[i]) % s_teeth[i];
    }

    uint32_t n = solve_pair(r[0], s_teeth[0], r[1], s_teeth[1]);
    uint32_t m = s_teeth[0] * s_teeth[1];
    uint32_t t = s_teeth[2];
    n += m * (((r[2] + t) - (n % t)) % t * mod_inv(m % t, t) % t);
    return (uint16_t)n;
}

gear_pos_t gear_decode(const gear_angles_t *a)
{
    gear_pos_t pos = {.valid = false, .turns = 0u, .angle = a->sun};
    /* associate each field with its gear by name; the config order of
     * the teeth array is fixed by the role table in gear_config.h */
    const uint16_t g[GEAR_DRIVEN_COUNT] = {a->gear1, a->gear2, a->gear3};
    uint16_t n = 0u;

    if (!gear_decode_init())
    {
        return pos;
    }

    n = decode_turns(g);
    pos.turns = n;

    if (s_have_last)
    {
        /* circular distance on the 0..GEAR_TURN_RANGE-1 turn ring */
        uint32_t d = (n + (uint32_t)GEAR_TURN_RANGE - s_last_turns) %
                     (uint32_t)GEAR_TURN_RANGE;
        if (d > GEAR_TURN_RANGE / 2u)
        {
            d = GEAR_TURN_RANGE - d;
        }
        if (d > GEAR_MAX_TURNS_DELTA)
        {
            return pos; /* implausible jump: keep the last position */
        }
    }

    s_last_turns = n;
    s_have_last = true;
    pos.valid = true;
    return pos;
}