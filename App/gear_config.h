/**
 * @file gear_config.h
 * @brief Compile-time mechanical configuration of the multi-turn gear train.
 *
 * This is the only file to touch when the gearbox changes: edit the tooth
 * counts, rebuild, reflash.  The decoder is configuration-agnostic; any
 * driven tooth set works as long as the counts stay pairwise coprime with
 * each other and with the input count (enforced by gear_decode_init), and
 * their product fits the turn field (enforced here, at compile time).
 */
#ifndef APP_GEAR_CONFIG_H
#define APP_GEAR_CONFIG_H

/**
 * The four MT6701 roles; every other file references encoders by these
 * names, never by raw index.  The electrical association (CSN label and
 * SPI bus per role) is declared once in the s_enc table of hal_stm32.c,
 * keyed by this enum:
 *
 *   role         wheel        teeth  CSN   bus
 *   -----------  -----------  -----  ----  ----
 *   ENC_SUN      sun gear     13     CSN4  SPI2
 *   ENC_GEAR_1   driven gear  17     CSN1  SPI1
 *   ENC_GEAR_2   driven gear  19     CSN2  SPI1
 *   ENC_GEAR_3   driven gear  23     CSN3  SPI2
 */
typedef enum
{
    ENC_SUN = 0,
    ENC_GEAR_1,
    ENC_GEAR_2,
    ENC_GEAR_3,
    ENC_COUNT
} encoder_role_t;

/** Teeth on the input (sun) shaft; ENC_SUN reads it directly (1:1). */
#define GEAR_INPUT_TEETH 13u

/** Teeth of the driven gears; ENC_GEAR_i reads teeth i (see role table
 *  above; the CSN/bus wiring lives in the s_enc table of hal_stm32.c). */
#define GEAR_DRIVEN_TEETH_1 17u
#define GEAR_DRIVEN_TEETH_2 19u
#define GEAR_DRIVEN_TEETH_3 23u
#define GEAR_DRIVEN_TEETH                                                       \
    {GEAR_DRIVEN_TEETH_1, GEAR_DRIVEN_TEETH_2, GEAR_DRIVEN_TEETH_3}
#define GEAR_DRIVEN_COUNT 3u

/**
 * Width (bits) of the turn-count field.  Must hold GEAR_TURN_RANGE, the
 * LCM of the driven counts -- their product while pairwise coprime.  The
 * 13-bit geometry below is the architecture's "multi-turn memory" (8192
 * states, 10% margin over the 7429-turn range; docs/architecture.md).
 */
#define GEAR_POS_BITS 13u

#define GEAR_TURN_RANGE                                                       \
    (GEAR_DRIVEN_TEETH_1 * GEAR_DRIVEN_TEETH_2 * GEAR_DRIVEN_TEETH_3)

_Static_assert(GEAR_TURN_RANGE <= (1u << GEAR_POS_BITS),                     \
               "GEAR_TURN_RANGE does not fit the GEAR_POS_BITS turn field");

/**
 * Maximum plausible position change between two samples, in turns.  The
 * slew guard in gear_decode rejects larger jumps (a misread gear decodes
 * to a jump of a multiple of the other gears' product -- hundreds of
 * turns -- which no real shaft makes between samples).  Sized for the
 * worst-case shaft speed at the 1 kHz sample cadence (TIM2.ARR in CubeMX,
 * tim.c): 1 turn per 1 ms sample allows 1000 turns/s (~360,000 deg/s).
 */
#define GEAR_MAX_TURNS_DELTA 1u

#endif /* APP_GEAR_CONFIG_H */