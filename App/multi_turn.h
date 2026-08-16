/**
 * @file multi_turn.h
 * @brief Multi-turn accumulation over the 14-bit MT6701 angle.
 *
 * Full revolutions are counted by detecting wraparounds in the raw angle
 * and persisted to non-volatile storage so the absolute multi-turn
 * position survives power cycles.
 */
#ifndef APP_MULTI_TURN_H
#define APP_MULTI_TURN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t  turns;      /* full revolutions accumulated, signed */
    uint16_t angle;      /* current angle within the turn */
    bool     valid;      /* state was restored from NVS */
    int32_t  saved_turns; /* internal: last value persisted to NVS */
} mt_state_t;

/** Load persisted state; a fresh zero state if NVS is empty or corrupt. */
void mt_init(mt_state_t *st);

/**
 * Feed a validated angle sample; updates the turn counter on wraparound
 * and persists the new record when the counter changed.
 * @return 0 on success, -1 if persisting to NVS failed.
 */
int mt_update(mt_state_t *st, uint16_t angle);

#endif /* APP_MULTI_TURN_H */
