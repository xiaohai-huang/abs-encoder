/**
 * @file test_gear_decode.c
 * @brief Unit tests for the gear_decode public API (App/gear_decode.c).
 *
 * Standalone binary: links only gear_decode.c -- no HAL, no simulated
 * chips.  Every sample is a gear_angles_t tuple constructed directly, so
 * the module's public API (init / reset / decode) is exercised with raw
 * angle counts rather than through the SSI read path.
 *
 * Exit code 0 = all checks passed; built and run by `make run` in sim/.
 */
#include <stdint.h>
#include <stdio.h>

#include "gear_decode.h"

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        g_checks++;                                                          \
        if (!(cond))                                                         \
        {                                                                    \
            g_failures++;                                                    \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                    \
    } while (0)

/* The driven gears, ordered like gear_decode's internal planet_teeth[]. */
static const uint16_t teeth[GEAR_DRIVEN_COUNT] = GEAR_DRIVEN_TEETH;

/* Angle of driven gear `teeth` after `turns` input turns, rounded to the
 * nearest tooth-slot center. */
static uint16_t gear_angle(uint32_t turns, uint32_t teeth)
{
    uint32_t phase = (GEAR_INPUT_TEETH * turns) % teeth;
    return (uint16_t)((phase * 16384u + teeth / 2u) / teeth);
}

/* Sample set that decodes to `turns`; the sun angle is arbitrary. */
static gear_angles_t angles_for(uint32_t turns)
{
    gear_angles_t ga = {.sun = 0u,
                        .gear1 = gear_angle(turns, teeth[0]),
                        .gear2 = gear_angle(turns, teeth[1]),
                        .gear3 = gear_angle(turns, teeth[2])};
    return ga;
}

/* Shortest circular distance between two turn counts, 0 .. RANGE/2. */
static uint32_t circular_distance(uint32_t a, uint32_t b)
{
    uint32_t d = (a + GEAR_TURN_RANGE - b) % GEAR_TURN_RANGE;
    if (d > GEAR_TURN_RANGE / 2u)
    {
        d = GEAR_TURN_RANGE - d;
    }
    return d;
}

/* Raw count `count + off`, wrapped onto the 14-bit circle. */
static uint16_t add_off(uint16_t count, int32_t off)
{
    return (uint16_t)(((int32_t)count + 16384 + off) % 16384);
}

/* One gear field of a sample tuple, indexed like planet_teeth[]. */
static uint16_t *gear_member(gear_angles_t *ga, size_t i)
{
    if (i == 0u) return &ga->gear1;
    if (i == 1u) return &ga->gear2;
    return &ga->gear3;
}

/* Raw-count range that maps to one tooth: the decoder rounds
 * count -> tooth via ((count * teeth + 8192) / 16384) % teeth, so tooth k
 * spans [ceil((16384k - 8192) / teeth), floor((16384k + 8191) / teeth)]. */
static uint16_t slot_low(uint32_t tooth, uint32_t teeth)
{
    int32_t num = (int32_t)(tooth * 16384u) - 8192;
    if (num <= 0)
    {
        return 0u;
    }
    return (uint16_t)((num + (int32_t)teeth - 1) / (int32_t)teeth);
}

static uint16_t slot_high(uint32_t tooth, uint32_t teeth)
{
    return (uint16_t)((tooth * 16384u + 8191u) / teeth);
}

/* Must come first: system_initialized is memoized.  gear_decode() calls
 * gear_decode_init() itself, so with a valid geometry the "disabled before
 * init" path is not reachable through the public API -- instead pin that
 * init is idempotent and that reset() must not un-initialize it. */
static void test_init(void)
{
    printf("init validates the geometry, idempotent, reset keeps it\n");
    CHECK(gear_decode_init());
    CHECK(gear_decode_init());
    gear_decode_reset();
    gear_angles_t ga0 = angles_for(0u);
    gear_pos_t p = gear_decode(&ga0);
    CHECK(p.valid);
    CHECK(p.turns == 0u);
}

static void test_exact_decode(void)
{
    printf("exact decodes across the range, sun angle passthrough\n");
    const uint32_t ns[] = {0u, 1u, 2u, 5u, 7u, 8u, 3654u, 4000u,
                           GEAR_TURN_RANGE - 1u};
    const uint16_t suns[] = {0u, 1u, 163u, MT6701_ANGLE_MAX};

    for (size_t k = 0u; k < sizeof(ns) / sizeof(ns[0]); k++)
    {
        for (size_t s = 0u; s < sizeof(suns) / sizeof(suns[0]); s++)
        {
            gear_angles_t ga = angles_for(ns[k]);
            ga.sun = suns[s];
            gear_decode_reset();
            gear_pos_t p = gear_decode(&ga);
            CHECK(p.valid);
            CHECK(p.turns == ns[k]);
            CHECK(p.angle == suns[s]);
        }
    }
}

static void test_angle_edges(void)
{
    printf("raw-count extremes 0/1/16382/16383 stay inside tooth 0\n");
    /* With a reference at 0 every gear sits in tooth 0, and tooth 0's raw
     * range spans both ends of the 14-bit angle: the low tail [0, ~481]
     * and the high tail [~15903, 16383] (17 x 964 > 16384, so the last
     * slot wraps onto the first).  All four fields may therefore take the
     * extreme counts without changing the decode. */
    const uint16_t edges[] = {0u, 1u, MT6701_ANGLE_MAX - 1u, MT6701_ANGLE_MAX};
    for (size_t f = 0u; f < 4u; f++)
    {
        for (size_t e = 0u; e < sizeof(edges) / sizeof(edges[0]); e++)
        {
            gear_angles_t ga = angles_for(0u);
            switch (f)
            {
            case 0u: ga.sun = edges[e];   break;
            case 1u: ga.gear1 = edges[e]; break;
            case 2u: ga.gear2 = edges[e]; break;
            default: ga.gear3 = edges[e]; break;
            }
            gear_decode_reset();
            gear_pos_t p = gear_decode(&ga);
            CHECK(p.valid);
            CHECK(p.turns == 0u);
            CHECK(p.angle == ga.sun);
        }
    }

    /* All four extremes at once still decode to 0. */
    gear_angles_t all = {MT6701_ANGLE_MAX, MT6701_ANGLE_MAX,
                         MT6701_ANGLE_MAX, MT6701_ANGLE_MAX};
    gear_decode_reset();
    gear_pos_t p = gear_decode(&all);
    CHECK(p.valid);
    CHECK(p.turns == 0u);
    CHECK(p.angle == MT6701_ANGLE_MAX);

    /* One count past the slot edge of tooth 0 is the first count of tooth
     * 1: gear1 = 482 alone decodes to 2622 turns (residue (1,0,0):
     * 19 * 23 * (4 * 13^{-1} mod 17 = 6) = 2622).  Fresh from reset it is
     * accepted as a reference; with a reference at 0 the guard rejects it. */
    gear_decode_reset();
    gear_angles_t misread = {.sun = 0u, .gear1 = 482u,
                             .gear2 = MT6701_ANGLE_MAX,
                             .gear3 = MT6701_ANGLE_MAX};
    p = gear_decode(&misread);
    CHECK(p.valid);
    CHECK(p.turns == 2622u);

    gear_angles_t ga0 = angles_for(0u);
    gear_decode_reset();
    CHECK(gear_decode(&ga0).valid); /* reference at turn 0 */
    all.gear1 = 481u;                          /* last count of tooth 0 */
    p = gear_decode(&all);
    CHECK(p.valid);
    CHECK(p.turns == 0u);

    all.gear1 = 482u; /* first count of tooth 1 */
    p = gear_decode(&all);
    CHECK(!p.valid);
    /* .turns carries the decoded-but-rejected 2622-turn reading: the
     * guard rejects it, it was never accepted as the position */
    CHECK(p.turns == 2622u);
    CHECK(p.angle == MT6701_ANGLE_MAX);
}

static void test_slot_quantization(void)
{
    printf("every count inside a tooth slot decodes to the same turns\n");
    /* The low and high edge of the expected slot are far apart (up to a
     * full slot, ~964 counts on the 17T gear) yet must map to the same
     * tooth and thus the same absolute position. */
    const uint32_t ns[] = {0u, 1234u, 4000u, GEAR_TURN_RANGE - 1u};
    for (size_t k = 0u; k < sizeof(ns) / sizeof(ns[0]); k++)
    {
        for (size_t g = 0u; g < GEAR_DRIVEN_COUNT; g++)
        {
            uint32_t tooth = (GEAR_INPUT_TEETH * ns[k]) % teeth[g];
            uint16_t lo = slot_low(tooth, teeth[g]);
            uint16_t hi = slot_high(tooth, teeth[g]);
            CHECK(lo <= hi);

            gear_angles_t ga = angles_for(ns[k]);
            *gear_member(&ga, g) = lo;
            gear_decode_reset();
            gear_pos_t p = gear_decode(&ga);
            CHECK(p.valid);
            CHECK(p.turns == ns[k]);

            ga = angles_for(ns[k]);
            *gear_member(&ga, g) = hi;
            gear_decode_reset();
            p = gear_decode(&ga);
            CHECK(p.valid);
            CHECK(p.turns == ns[k]);
        }
    }
}

static void test_noise_and_small_steps(void)
{
    printf("sub-slot noise is absorbed, small steps accepted\n");
    /* Moderate noise within half a tooth slot changes nothing. */
    gear_angles_t ga = angles_for(1234u);
    ga.gear1 = (uint16_t)((ga.gear1 + 80u) % 16384u);
    ga.gear2 = (uint16_t)(ga.gear2 - 60u); /* underflow wraps, same circle */
    ga.gear3 = (uint16_t)((ga.gear3 + 300u) % 16384u);
    gear_decode_reset();
    gear_pos_t p = gear_decode(&ga);
    CHECK(p.valid);
    CHECK(p.turns == 1234u);

    /* A repeated identical sample stays accepted (distance 0). */
    p = gear_decode(&ga);
    CHECK(p.valid);
    CHECK(p.turns == 1234u);

    /* One-turn forward step: 7 -> 8. */
    gear_decode_reset();
    gear_angles_t g7 = angles_for(7u);
    CHECK(gear_decode(&g7).valid);
    gear_angles_t g8 = angles_for(8u);
    p = gear_decode(&g8);
    CHECK(p.valid);
    CHECK(p.turns == 8u);
}

static void test_wrap_seam(void)
{
    printf("7429-turn seam: distance-1 steps wrap, distance-2 rejected\n");
    gear_angles_t g0 = angles_for(0u);

    /* +1 across the seam: 7428 -> 0. */
    gear_decode_reset();
    {
        gear_angles_t g_top = angles_for(GEAR_TURN_RANGE - 1u);
        CHECK(gear_decode(&g_top).valid);
    }
    gear_pos_t p = gear_decode(&g0);
    CHECK(p.valid);
    CHECK(p.turns == 0u);

    /* -1 across the seam: 0 -> 7428. */
    gear_decode_reset();
    CHECK(gear_decode(&g0).valid);
    gear_angles_t g_top = angles_for(GEAR_TURN_RANGE - 1u);
    p = gear_decode(&g_top);
    CHECK(p.valid);
    CHECK(p.turns == GEAR_TURN_RANGE - 1u);

    /* Distance 2 is a full half-flight: rejected in both directions. */
    gear_decode_reset();
    CHECK(gear_decode(&g0).valid);
    gear_angles_t g2 = angles_for(2u);
    p = gear_decode(&g2);
    CHECK(!p.valid);

    gear_decode_reset();
    gear_angles_t g7427 = angles_for(GEAR_TURN_RANGE - 2u);
    CHECK(gear_decode(&g7427).valid);
    p = gear_decode(&g0);
    CHECK(!p.valid);

    gear_decode_reset();
    gear_angles_t g5 = angles_for(5u);
    CHECK(gear_decode(&g5).valid);
    gear_angles_t g7 = angles_for(7u);
    p = gear_decode(&g7);
    CHECK(!p.valid);

    gear_decode_reset();
    gear_angles_t g7b = angles_for(7u);
    CHECK(gear_decode(&g7b).valid);
    gear_angles_t g5b = angles_for(5u);
    p = gear_decode(&g5b);
    CHECK(!p.valid);
}

static void test_slew_guard_semantics(void)
{
    printf("rejections clear status but tracking stays at the last accept\n");
    gear_angles_t ga = angles_for(7u);
    ga.sun = 0x55AAu;
    gear_decode_reset();
    CHECK(gear_decode(&ga).valid);

    /* Tooth +1 on the 23T gear shifts the residue by 13^{-1} = 16, i.e. by
     * 17 * 19 * 16 = 5168 turns: rejected, .turns still carries the
     * rejected decode (7 + 5168), .angle echoes sun. */
    ga.gear3 = add_off(ga.gear3, 16384 / 23);
    gear_pos_t p = gear_decode(&ga);
    CHECK(!p.valid);
    CHECK(p.turns == 5175u);
    CHECK(p.angle == 0x55AAu);

    /* The guard still measures from turn 7, so 8 is accepted next. */
    gear_angles_t g8 = angles_for(8u);
    p = gear_decode(&g8);
    CHECK(p.valid);
    CHECK(p.turns == 8u);

    /* After reset() any wild sample becomes the new reference. */
    ga = angles_for(123u);
    gear_decode_reset();
    p = gear_decode(&ga);
    CHECK(p.valid);
    CHECK(p.turns == 123u);
}

static void test_misread_invariants(void)
{
    printf("misreads are either rejected or within one turn of the truth\n");
    /* The slew guard's contract can be checked as an equivalence: a decode
     * is accepted exactly when its circular distance from the previous
     * accepted sample is at most GEAR_MAX_TURNS_DELTA.  Perturb one gear
     * (or two, correlated) and verify the invariant holds. */
    const uint32_t refs[] = {7u, 4000u};
    const int32_t mid_slot[] = {1800, -1800}; /* gear3-only, mid-gap */

    for (size_t r = 0u; r < sizeof(refs) / sizeof(refs[0]); r++)
    {
        uint32_t n = refs[r];
        for (size_t g = 0u; g < GEAR_DRIVEN_COUNT; g++)
        {
            /* Full slot flips the tooth (jump of hundreds of turns); half
             * slot stays inside the same tooth (distance 0); half + 1 is
             * the first count of the next tooth. */
            int32_t full = 16384 / (int32_t)teeth[g];
            int32_t half = 8192 / (int32_t)teeth[g];
            const int32_t offs[] = {
                full, -full, half, -half, half + 1, -(half + 1),
            };
            size_t off_count = sizeof(offs) / sizeof(offs[0]);

            if (g == 2u) /* the mid-gap disturbance from the old harness */
            {
                off_count = sizeof(mid_slot) / sizeof(mid_slot[0]);
                for (size_t o = 0u; o < off_count; o++)
                {
                    gear_angles_t ref = angles_for(n);
                    gear_decode_reset();
                    CHECK(gear_decode(&ref).valid);
                    gear_angles_t ga = angles_for(n);
                    *gear_member(&ga, g) = add_off(*gear_member(&ga, g),
                                                   mid_slot[o]);
                    gear_pos_t p = gear_decode(&ga);
                    CHECK((circular_distance(p.turns, n) <=
                           GEAR_MAX_TURNS_DELTA) == p.valid);
                }
            }
            else
            {
                for (size_t o = 0u; o < off_count; o++)
                {
                    gear_angles_t ref = angles_for(n);
                    gear_decode_reset();
                    CHECK(gear_decode(&ref).valid);
                    gear_angles_t ga = angles_for(n);
                    *gear_member(&ga, g) = add_off(*gear_member(&ga, g),
                                                   offs[o]);
                    gear_pos_t p = gear_decode(&ga);
                    CHECK((circular_distance(p.turns, n) <=
                           GEAR_MAX_TURNS_DELTA) == p.valid);
                }
            }
        }

        /* Correlated two-gear failures, one slot each. */
        const int32_t f0 = 16384 / (int32_t)teeth[0];
        const int32_t f1 = 16384 / (int32_t)teeth[1];
        const int32_t f2 = 16384 / (int32_t)teeth[2];
        {
            gear_angles_t ga = angles_for(n);
            gear_angles_t ref = angles_for(n);
            gear_decode_reset();
            CHECK(gear_decode(&ref).valid);
            *gear_member(&ga, 1u) = add_off(*gear_member(&ga, 1u), f1);
            *gear_member(&ga, 2u) = add_off(*gear_member(&ga, 2u), -f2);
            gear_pos_t p = gear_decode(&ga);
            CHECK((circular_distance(p.turns, n) <=
                   GEAR_MAX_TURNS_DELTA) == p.valid);
        }
        {
            gear_angles_t ga = angles_for(n);
            gear_angles_t ref = angles_for(n);
            gear_decode_reset();
            CHECK(gear_decode(&ref).valid);
            *gear_member(&ga, 0u) = add_off(*gear_member(&ga, 0u), -f0);
            *gear_member(&ga, 1u) = add_off(*gear_member(&ga, 1u), f1);
            gear_pos_t p = gear_decode(&ga);
            CHECK((circular_distance(p.turns, n) <=
                   GEAR_MAX_TURNS_DELTA) == p.valid);
        }
        {
            gear_angles_t ga = angles_for(n);
            gear_angles_t ref = angles_for(n);
            gear_decode_reset();
            CHECK(gear_decode(&ref).valid);
            *gear_member(&ga, 0u) = add_off(*gear_member(&ga, 0u), f0);
            *gear_member(&ga, 2u) = add_off(*gear_member(&ga, 2u), -f2);
            gear_pos_t p = gear_decode(&ga);
            CHECK((circular_distance(p.turns, n) <=
                   GEAR_MAX_TURNS_DELTA) == p.valid);
        }
    }
}

static void test_full_range_sweep(void)
{
    printf("all %u states decode with in-slot offsets on every gear\n",
           (unsigned)GEAR_TURN_RANGE);
    /* Offsets stay inside every tooth slot: the narrowest slot is the 23T
     * gear's (16384/23 ~= 712 counts, half ~= 356), and the slots of tooth
     * 0 and the last tooth both touch the 14-bit wrap, so +/-300 covers
     * the edges without crossing into a neighbor tooth. */
    const int32_t offs[] = {-300, 0, 300};
    for (uint32_t n = 0u; n < GEAR_TURN_RANGE; n++)
    {
        for (size_t a = 0u; a < sizeof(offs) / sizeof(offs[0]); a++)
        {
            for (size_t b = 0u; b < sizeof(offs) / sizeof(offs[0]); b++)
            {
                for (size_t c = 0u; c < sizeof(offs) / sizeof(offs[0]); c++)
                {
                    gear_angles_t ga = angles_for(n);
                    *gear_member(&ga, 0u) = add_off(*gear_member(&ga, 0u), offs[a]);
                    *gear_member(&ga, 1u) = add_off(*gear_member(&ga, 1u), offs[b]);
                    *gear_member(&ga, 2u) = add_off(*gear_member(&ga, 2u), offs[c]);
                    gear_decode_reset();
                    gear_pos_t p = gear_decode(&ga);
                    CHECK(p.valid);
                    CHECK(p.turns == n);
                }
            }
        }
    }
}

int main(void)
{
    test_init();
    test_exact_decode();
    test_angle_edges();
    test_slot_quantization();
    test_noise_and_small_steps();
    test_wrap_seam();
    test_slew_guard_semantics();
    test_misread_invariants();
    test_full_range_sweep();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures != 0 ? 1 : 0;
}