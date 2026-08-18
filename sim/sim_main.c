/**
 * @file sim_main.c
 * @brief PC test harness: drives the same protocol logic used by the
 *        firmware against simulated MT6701 chips (one per encoder index,
 *        two SPI buses).
 *
 * Exit code 0 = all checks passed; run from sim/ (make run).
 */
#include <stdio.h>

#include "hal.h"
#include "mt6701.h"
#include "mt6701_slave_sim.h"
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

static void test_static_angle(void)
{
    printf("static angle reads (encoders 0, 1, 3)\n");
    const uint8_t encs[] = {0u, 1u, 3u};
    const uint16_t angles[] = {0u, 1u, 0x1234u, MT6701_ANGLE_MAX};
    for (size_t e = 0; e < sizeof(encs) / sizeof(encs[0]); e++)
    {
        for (size_t i = 0; i < sizeof(angles) / sizeof(angles[0]); i++)
        {
            mt6701_slave_set_angle(encs[e], angles[i]);
            mt6701_sample_t s;
            CHECK(mt6701_read_sample(encs[e], &s) == 0);
            CHECK(s.angle == angles[i]);
            CHECK(!s.button);
        }
    }
}

static void test_encoder_independence(void)
{
    printf("four encoders read independently, no crosstalk\n");
    const uint16_t angles[MT6701_ENC_COUNT] = {0x1111u, 0x2222u, 0x3333u, 0x0FF0u};
    for (uint8_t e = 0; e < MT6701_ENC_COUNT; e++)
    {
        mt6701_slave_set_angle(e, angles[e]);
    }

    mt6701_sample_t s;
    for (uint8_t e = 0; e < MT6701_ENC_COUNT; e++)
    {
        CHECK(mt6701_read_sample(e, &s) == 0);
        CHECK(s.angle == angles[e]);
        CHECK(!s.button);
    }

    /* re-reading one encoder must not disturb the others' state */
    CHECK(mt6701_read_sample(0u, &s) == 0);
    CHECK(s.angle == angles[0u]);
    CHECK(mt6701_read_sample(2u, &s) == 0);
    CHECK(s.angle == angles[2u]);
}

static void test_button_flag(void)
{
    printf("button flag passthrough (encoder 3)\n");
    mt6701_slave_set_angle(3u, 0x555u);
    mt6701_slave_set_button(3u, true);
    mt6701_sample_t s;
    CHECK(mt6701_read_sample(3u, &s) == 0);
    CHECK(s.button);
    CHECK(s.angle == 0x555u);
    mt6701_slave_set_button(3u, false);
}

static void test_faults(void)
{
    printf("fault handling: track loss, bad field, stuck chip\n");
    mt6701_slave_set_angle(0u, 1000u);
    mt6701_sample_t s;

    mt6701_slave_set_track_loss(0u, true);
    CHECK(mt6701_read_sample(0u, &s) == -1);
    mt6701_slave_set_track_loss(0u, false);

    mt6701_slave_set_field_status(0u, 1u);
    CHECK(mt6701_read_sample(0u, &s) == -1);
    mt6701_slave_set_field_status(0u, 0u);

    mt6701_slave_set_stuck(0u, true);
#if MT6701_CRC6_ENABLED
    CHECK(mt6701_read_sample(0u, &s) == -2); /* 0xFF fails the CRC-6 check */
#else
    CHECK(mt6701_read_sample(0u, &s) == -1); /* 0xFF decodes as track loss */
#endif
    mt6701_slave_set_stuck(0u, false);

    CHECK(mt6701_read_sample(0u, &s) == 0); /* recovered */

    /* same fault path on the second bus */
    mt6701_slave_set_stuck(3u, true);
#if MT6701_CRC6_ENABLED
    CHECK(mt6701_read_sample(3u, &s) == -2); /* 0xFF fails the CRC-6 check */
#else
    CHECK(mt6701_read_sample(3u, &s) == -1); /* 0xFF decodes as track loss */
#endif
    mt6701_slave_set_stuck(3u, false);
    CHECK(mt6701_read_sample(3u, &s) == 0);
}

static void test_crc(void)
{
    printf("crc-6 validation (encoder 1)\n");
    mt6701_slave_set_angle(1u, 0x1234u);
    mt6701_slave_set_crc_broken(1u, true);
    mt6701_sample_t s;
#if MT6701_CRC6_ENABLED
    CHECK(mt6701_read_sample(1u, &s) == -2); /* all retries fail the CRC */
#else
    CHECK(mt6701_read_sample(1u, &s) == 0); /* CRC check disabled (default) */
#endif
    mt6701_slave_set_crc_broken(1u, false);
}

/* angle of driven gear `teeth` after `turns` input turns */
static uint16_t gear_angle(uint32_t turns, uint32_t teeth)
{
    uint32_t phase = (GEAR_INPUT_TEETH * turns) % teeth;
    return (uint16_t)((phase * 16384u + teeth / 2u) / teeth);
}

/* sample set for the given turn count; the sun angle is arbitrary */
static gear_angles_t angles_for(uint32_t turns, const uint16_t teeth[GEAR_DRIVEN_COUNT])
{
    gear_angles_t ga = {.sun = 0u,
                        .gear1 = gear_angle(turns, teeth[0]),
                        .gear2 = gear_angle(turns, teeth[1]),
                        .gear3 = gear_angle(turns, teeth[2])};
    return ga;
}

static void test_gear_decode(void)
{
    printf("absolute gear-phase decode (encoder 0 = input shaft)\n");
    const uint16_t teeth[GEAR_DRIVEN_COUNT] = GEAR_DRIVEN_TEETH;
    CHECK(gear_decode_init());

    /* exact decodes across the whole unambiguous range; each case starts
     * from a fresh reference (arbitrary jumps are not the guard's job) */
    const uint32_t ns[] = {0u, 1u, 5u, 3654u, GEAR_TURN_RANGE - 1u};
    for (size_t k = 0u; k < sizeof(ns) / sizeof(ns[0]); k++)
    {
        gear_angles_t ga = angles_for(ns[k], teeth);
        ga.sun = 163u;
        gear_decode_reset();
        gear_pos_t p = gear_decode(&ga);
        CHECK(p.valid);
        CHECK(p.turns == ns[k]);
        CHECK(p.angle == ga.sun);
    }

    /* moderate noise within half a tooth slot changes nothing */
    {
        gear_angles_t ga = angles_for(1234u, teeth);
        ga.gear1 = (uint16_t)((ga.gear1 + 80u) % 16384u);
        ga.gear2 = (uint16_t)(ga.gear2 - 60u); /* underflow wraps, same circle */
        ga.gear3 = (uint16_t)((ga.gear3 + 300u) % 16384u);
        gear_decode_reset();
        gear_pos_t p = gear_decode(&ga);
        CHECK(p.valid);
        CHECK(p.turns == 1234u);
    }

    /* small real steps are accepted */
    {
        gear_decode_reset();
        gear_angles_t g7 = angles_for(7u, teeth);
        CHECK(gear_decode(&g7).valid);

        gear_angles_t ga = angles_for(8u, teeth);
        gear_pos_t p = gear_decode(&ga);
        CHECK(p.valid);
        CHECK(p.turns == 8u);
    }

    /* one-turn backward step across the 7429 -> 0 seam (delta is 1) */
    {
        gear_decode_reset();
        gear_angles_t g7428 = angles_for(GEAR_TURN_RANGE - 1u, teeth);
        CHECK(gear_decode(&g7428).valid); /* reference at 7428 */

        gear_angles_t ga = angles_for(0u, teeth);
        gear_pos_t p = gear_decode(&ga);
        CHECK(p.valid);
        CHECK(p.turns == 0u);
    }

    /* a misread gear decodes to a multi-hundred-turn jump: rejected */
    {
        gear_decode_reset();
        gear_angles_t ga = angles_for(7u, teeth);
        CHECK(gear_decode(&ga).valid); /* reference at N=7 */

        ga.gear3 = (uint16_t)((ga.gear3 + 1800u) % 16384u); /* 23T, mid-gap */
        CHECK(!gear_decode(&ga).valid);

        /* even a full-slot offset (a "plausible" neighbouring state) is
         * caught: it contradicts continuity by hundreds of turns */
        gear_decode_reset();
        gear_angles_t gb = angles_for(7u, teeth);
        CHECK(gear_decode(&gb).valid);
        gb.gear3 = (uint16_t)((gb.gear3 + 16384u / teeth[2]) % 16384u);
        CHECK(!gear_decode(&gb).valid);
    }
}

/* The core Vernier theorem (docs/architecture.md, Notes): the residue
 * triple is a bijection over 0..GEAR_TURN_RANGE-1 -- every turn count has
 * a unique fingerprint that decodes back to itself, and the system cannot
 * repeat before the full range.  Samples sit off the slot centers (a
 * deterministic offset inside each tooth slot), so the nearest-slot
 * rounding of the decoder is exercised, not just exact centers. */
static void test_gear_vernier_bijection(void)
{
    printf("vernier bijection over all %u turn states (off-center samples)\n",
           (unsigned)GEAR_TURN_RANGE);
    const uint16_t teeth[GEAR_DRIVEN_COUNT] = GEAR_DRIVEN_TEETH;

    for (uint32_t n = 0u; n < GEAR_TURN_RANGE; n++)
    {
        uint16_t g[GEAR_DRIVEN_COUNT];
        for (size_t i = 0u; i < GEAR_DRIVEN_COUNT; i++)
        {
            uint32_t slot = 16384u / teeth[i];
            int32_t off = (int32_t)(slot / 10u) * (int32_t)(i + 1u);
            if ((n & 1u) != 0u)
            {
                off = -off; /* alternate sides of the slot center */
            }
            g[i] = (uint16_t)((gear_angle(n, teeth[i]) + 16384u + off) %
                              16384u);
        }
        gear_angles_t ga = {.sun = 0u, .gear1 = g[0], .gear2 = g[1], .gear3 = g[2]};
        gear_decode_reset();
        gear_pos_t p = gear_decode(&ga);
        CHECK(p.valid);
        CHECK(p.turns == n);
    }
}

static void test_gear_decode_read_path(void)
{
    printf("gear decode via the real SSI read path (roles ENC_SUN..3)\n");
    const uint16_t teeth[GEAR_DRIVEN_COUNT] = GEAR_DRIVEN_TEETH;
    gear_decode_reset();

    mt6701_slave_set_angle(ENC_SUN, 333u);
    mt6701_slave_set_angle(ENC_GEAR_1, gear_angle(4000u, teeth[0]));
    mt6701_slave_set_angle(ENC_GEAR_2, gear_angle(4000u, teeth[1]));
    mt6701_slave_set_angle(ENC_GEAR_3, gear_angle(4000u, teeth[2]));

    gear_angles_t ga;
    mt6701_sample_t s;
    CHECK(mt6701_read_sample(ENC_SUN, &s) == 0);
    ga.sun = s.angle;
    CHECK(mt6701_read_sample(ENC_GEAR_1, &s) == 0);
    ga.gear1 = s.angle;
    CHECK(mt6701_read_sample(ENC_GEAR_2, &s) == 0);
    ga.gear2 = s.angle;
    CHECK(mt6701_read_sample(ENC_GEAR_3, &s) == 0);
    ga.gear3 = s.angle;

    gear_pos_t p = gear_decode(&ga);
    CHECK(p.valid);
    CHECK(p.turns == 4000u);
}

int main(void)
{
    app_hal.init();

    test_static_angle();
    test_encoder_independence();
    test_button_flag();
    test_faults();
    test_crc();
    test_gear_decode();
    test_gear_vernier_bijection();
    test_gear_decode_read_path();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures != 0 ? 1 : 0;
}