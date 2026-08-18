/**
 * @file sim_main.c
 * @brief PC test harness: drives the same protocol and multi-turn logic
 *        used by the firmware against simulated MT6701 chips (one per
 *        encoder index, two SPI buses).
 *
 * Exit code 0 = all checks passed; run from sim/ (make run) so the
 * nvs.bin backing file lands next to the harness.
 */
#include <stdio.h>

#include "hal.h"
#include "mt6701.h"
#include "mt6701_slave_sim.h"
#include "multi_turn.h"

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

static void test_multi_turn(void)
{
    printf("multi-turn accumulation (encoders 0 and 3)\n");
    mt_state_t st[MT6701_ENC_COUNT];
    for (uint8_t e = 0; e < MT6701_ENC_COUNT; e++)
    {
        mt_init(&st[e], e);
        CHECK(!st[e].valid); /* nvs.bin removed at startup */
        CHECK(st[e].turns == 0);
    }

    /* encoder 0: two forward wraps through 0 -> turns +2 */
    st[0].angle = 16300u;
    CHECK(mt_update(&st[0], 0u, 100u) == 0);
    CHECK(st[0].turns == 1);
    st[0].angle = 16300u;
    CHECK(mt_update(&st[0], 0u, 100u) == 0);
    CHECK(st[0].turns == 2);
    CHECK(st[0].angle == 100u);

    /* encoder 3: one backward wrap through 16383 -> turns -1 */
    st[3].angle = 100u;
    CHECK(mt_update(&st[3], 3u, 16300u) == 0);
    CHECK(st[3].turns == -1);
    CHECK(st[3].angle == 16300u);

    /* encoders that never wrapped keep their own, untouched state */
    CHECK(st[1].turns == 0);
    CHECK(st[2].turns == 0);
}

static void test_persistence(void)
{
    printf("per-encoder persistence across re-init\n");
    mt_state_t st[MT6701_ENC_COUNT];
    for (uint8_t e = 0; e < MT6701_ENC_COUNT; e++)
    {
        mt_init(&st[e], e);
    }

    CHECK(st[0].valid);
    CHECK(st[0].turns == 2);
    CHECK(st[0].angle == 100u);
    CHECK(st[3].valid);
    CHECK(st[3].turns == -1);
    CHECK(st[3].angle == 16300u);

    /* slots never written stay fresh (no cross-slot leakage) */
    CHECK(!st[1].valid);
    CHECK(!st[2].valid);
}

static void test_corrupt_nvs(void)
{
    printf("corrupt NVS falls back to fresh state\n");
    FILE *f = fopen("nvs.bin", "wb");
    CHECK(f != NULL);
    if (f != NULL)
    {
        uint8_t garbage[48];
        for (size_t i = 0; i < sizeof(garbage); i++)
        {
            garbage[i] = 0xFFu;
        }
        fwrite(garbage, 1, sizeof(garbage), f);
        fclose(f);
    }

    mt_state_t st[MT6701_ENC_COUNT];
    for (uint8_t e = 0; e < MT6701_ENC_COUNT; e++)
    {
        mt_init(&st[e], e);
        CHECK(!st[e].valid);
        CHECK(st[e].turns == 0);
    }
}

int main(void)
{
    /* self-contained: always start from a clean backing store */
    remove("nvs.bin");

    app_hal.init();

    test_static_angle();
    test_encoder_independence();
    test_button_flag();
    test_faults();
    test_crc();
    test_multi_turn();
    test_persistence();
    test_corrupt_nvs();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures != 0 ? 1 : 0;
}