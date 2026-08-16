/**
 * @file sim_main.c
 * @brief PC test harness: drives the same protocol and multi-turn logic
 *        used by the firmware against a simulated MT6701 chip.
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
    printf("static angle reads\n");
    const uint16_t angles[] = {0u, 1u, 0x1234u, MT6701_ANGLE_MAX};
    for (size_t i = 0; i < sizeof(angles) / sizeof(angles[0]); i++)
    {
        mt6701_slave_set_angle(angles[i]);
        mt6701_sample_t s;
        CHECK(mt6701_read_sample(&s) == 0);
        CHECK(s.angle == angles[i]);
        CHECK(!s.button);
    }
}

static void test_button_flag(void)
{
    printf("button flag passthrough\n");
    mt6701_slave_set_angle(0x555u);
    mt6701_slave_set_button(true);
    mt6701_sample_t s;
    CHECK(mt6701_read_sample(&s) == 0);
    CHECK(s.button);
    CHECK(s.angle == 0x555u);
    mt6701_slave_set_button(false);
}

static void test_faults(void)
{
    printf("fault handling: track loss, bad field, stuck chip\n");
    mt6701_slave_set_angle(1000u);
    mt6701_sample_t s;

    mt6701_slave_set_track_loss(true);
    CHECK(mt6701_read_sample(&s) == -1);
    mt6701_slave_set_track_loss(false);

    mt6701_slave_set_field_status(1u);
    CHECK(mt6701_read_sample(&s) == -1);
    mt6701_slave_set_field_status(0u);

    mt6701_slave_set_stuck(true);
#if MT6701_CRC6_ENABLED
    CHECK(mt6701_read_sample(&s) == -2); /* 0xFF fails the CRC-6 check */
#else
    CHECK(mt6701_read_sample(&s) == -1); /* 0xFF decodes as track loss */
#endif
    mt6701_slave_set_stuck(false);

    CHECK(mt6701_read_sample(&s) == 0); /* recovered */
}

static void test_crc(void)
{
    printf("crc-6 validation\n");
    mt6701_slave_set_angle(0x1234u);
    mt6701_slave_set_crc_broken(true);
    mt6701_sample_t s;
#if MT6701_CRC6_ENABLED
    CHECK(mt6701_read_sample(&s) == -2); /* all retries fail the CRC */
#else
    CHECK(mt6701_read_sample(&s) == 0); /* CRC check disabled (default) */
#endif
    mt6701_slave_set_crc_broken(false);
}

static void test_multi_turn(void)
{
    printf("multi-turn accumulation\n");
    mt_state_t st;
    mt_init(&st);
    CHECK(!st.valid); /* nvs.bin removed at startup */
    CHECK(st.turns == 0);

    CHECK(mt_update(&st, 1000u) == 0);
    CHECK(st.turns == 0);

    /* place the state near the 16383/0 wrap, then feed short hops across it */
    st.angle = 16300u;
    CHECK(mt_update(&st, 100u) == 0); /* forward wrap through 0 */
    CHECK(st.turns == 1);
    CHECK(mt_update(&st, 50u) == 0);
    CHECK(st.turns == 1);
    CHECK(mt_update(&st, 16200u) == 0); /* backward wrap through 16383 */
    CHECK(st.turns == 0);
    CHECK(mt_update(&st, 100u) == 0); /* forward wrap again */
    CHECK(st.turns == 1);
    CHECK(st.angle == 100u);
}

static void test_persistence(void)
{
    printf("multi-turn persistence across re-init\n");
    mt_state_t st;
    mt_init(&st);
    CHECK(st.valid);
    CHECK(st.turns == 1);
    CHECK(st.angle == 100u);

    /* continuing on top of the restored state stays consistent */
    CHECK(mt_update(&st, 16300u) == 0);
    CHECK(st.turns == 0);
}

static void test_corrupt_nvs(void)
{
    printf("corrupt NVS falls back to fresh state\n");
    FILE *f = fopen("nvs.bin", "wb");
    CHECK(f != NULL);
    if (f != NULL)
    {
        const uint8_t garbage[12] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        fwrite(garbage, 1, sizeof(garbage), f);
        fclose(f);
    }

    mt_state_t st;
    mt_init(&st);
    CHECK(!st.valid);
    CHECK(st.turns == 0);
}

int main(void)
{
    /* self-contained: always start from a clean backing store */
    remove("nvs.bin");

    app_hal.init();

    test_static_angle();
    test_button_flag();
    test_faults();
    test_crc();
    test_multi_turn();
    test_persistence();
    test_corrupt_nvs();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures != 0 ? 1 : 0;
}
