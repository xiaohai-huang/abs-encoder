/**
 * @file multi_turn.c
 * @brief Multi-turn accumulation and NVS persistence (platform independent).
 */
#include "multi_turn.h"

#include "hal.h"
#include "mt6701.h"

#define MT_MAGIC    0x4D543031u /* "MT01" */

typedef struct
{
    uint32_t magic;
    int32_t  turns;
    uint16_t last_angle;
    uint16_t crc; /* CRC-16/CCITT over the preceding bytes */
} mt_record_t;

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len-- > 0u)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++)
        {
            crc = (uint16_t)((crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                             : (crc << 1));
        }
    }
    return crc;
}

void mt_init(mt_state_t *st, uint8_t enc)
{
    st->turns = 0;
    st->angle = 0;
    st->valid = false;
    st->saved_turns = 0;

    mt_record_t rec;
    uint32_t addr = (uint32_t)enc * sizeof(rec);
    if (app_hal.nvs_read(addr, &rec, sizeof(rec)) &&
        rec.magic == MT_MAGIC &&
        rec.crc == crc16_ccitt((const uint8_t *)&rec, sizeof(rec) - sizeof(rec.crc)))
    {
        st->turns = rec.turns;
        st->angle = rec.last_angle;
        st->saved_turns = rec.turns;
        st->valid = true;
    }
}

int mt_update(mt_state_t *st, uint8_t enc, uint16_t angle)
{
    int32_t delta = (int32_t)angle - (int32_t)st->angle;

    if (delta > (int32_t)(MT6701_ANGLE_MAX / 2))
    {
        st->turns--; /* backward wrap through 16383 */
    }
    else if (delta < -(int32_t)(MT6701_ANGLE_MAX / 2))
    {
        st->turns++; /* forward wrap through 0 */
    }
    st->angle = angle;

    if (st->turns != st->saved_turns)
    {
        mt_record_t rec = {MT_MAGIC, st->turns, st->angle, 0u};
        rec.crc = crc16_ccitt((const uint8_t *)&rec, sizeof(rec) - sizeof(rec.crc));
        if (!app_hal.nvs_write((uint32_t)enc * sizeof(rec), &rec, sizeof(rec)))
        {
            return -1;
        }
        st->saved_turns = st->turns;
    }

    return 0;
}