/**
 * @file multi_turn.h
 * @brief Multi-turn accumulation over the 14-bit MT6701 angle.
 *
 * Full revolutions are counted by detecting wraparounds in the raw angle
 * and persisted to non-volatile storage so the absolute multi-turn
 * position survives power cycles.  Each encoder has its own state and its
 * own NVS slot (slot = encoder index).
 */
#ifndef APP_MULTI_TURN_H
#define APP_MULTI_TURN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t  turns;       /* full revolutions accumulated, signed */
    uint16_t angle;       /* current angle within the turn */
    bool     valid;       /* state was restored from NVS */
    int32_t  saved_turns; /* internal: last value persisted to NVS */
} mt_state_t;

/**
 * Load persisted state; a fresh zero state if NVS is empty or corrupt.
 * @param enc encoder index, selects the NVS slot
 */
void mt_init(mt_state_t *st, uint8_t enc);

/**
 * Feed a validated angle sample; updates the turn counter on wraparound
 * and persists the new record when the counter changed.
 * @param enc encoder index, selects the NVS slot
 * @return 0 on success, -1 if persisting to NVS failed.
 */
int mt_update(mt_state_t *st, uint8_t enc, uint16_t angle);

#endif /* APP_MULTI_TURN_H */