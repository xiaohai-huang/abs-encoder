/**
 * @file gear_config.h
 * @brief Compile-time mechanical configuration of the multi-turn gear train.
 *
 * This is the only file to touch when the gearbox changes: edit the tooth
 * counts, rebuild, reflash.  The decoder is configuration-agnostic; any
 * driven tooth set works as long as the counts stay pairwise coprime with
 * each other and with the input count (enforced by GearDecoder's
 * constructor), and their product fits the turn field (enforced here, at
 * compile time).
 */
#ifndef APP_GEAR_CONFIG_H
#define APP_GEAR_CONFIG_H

#include <cstdint>

/**
 * The four MT6701 roles; every other file references encoders by these
 * names, never by raw index.  The electrical association (CSN label and
 * SPI bus per role) is declared once in the wiring table of hal_stm32.cpp,
 * keyed by this enum:
 *
 *   role   wheel        teeth  CSN   bus
 *   -----  -----------  -----  ----  ----
 *   Sun    sun gear     13     CSN4  SPI2
 *   Gear1  driven gear  17     CSN1  SPI1
 *   Gear2  driven gear  19     CSN2  SPI1
 *   Gear3  driven gear  23     CSN3  SPI2
 */
enum class EncoderRole
{
    Sun = 0,
    Gear1,
    Gear2,
    Gear3,
    RoleCount
};

/** Compile-time mechanical geometry; see the role table above. */
namespace GearConfig
{
/** Teeth on the input (sun) shaft; the Sun encoder reads it directly
 *  (1:1). */
constexpr uint16_t InputTeeth = 13u;

/** Teeth of the driven gears; the Gear1..3 encoders read teeth 1..3. */
constexpr uint16_t DrivenTeeth1 = 17u;
constexpr uint16_t DrivenTeeth2 = 19u;
constexpr uint16_t DrivenTeeth3 = 23u;
constexpr uint16_t DrivenCount  = 3u;

/** Driven tooth counts, indexed by driven-gear position (Gear1..Gear3);
 *  the CSN/bus wiring lives in the wiring table of hal_stm32.cpp. */
constexpr uint16_t DrivenTeethCounts[DrivenCount] = {
    DrivenTeeth1, DrivenTeeth2, DrivenTeeth3};

/**
 * Width (bits) of the turn-count field.  Must hold TurnRange, the LCM of
 * the driven counts -- their product while pairwise coprime.  The 13-bit
 * geometry below is the architecture's "multi-turn memory" (8192 states,
 * 10% margin over the 7429-turn range; docs/architecture.md).
 */
constexpr uint16_t TurnFieldBits = 13u;

constexpr uint32_t TurnRange =
    static_cast<uint32_t>(DrivenTeeth1) * DrivenTeeth2 * DrivenTeeth3;

static_assert(TurnRange <= (1u << TurnFieldBits),
              "TurnRange does not fit the TurnFieldBits turn field");

/**
 * Maximum plausible position change between two samples, in turns.  The
 * slew guard in GearDecoder rejects larger jumps (a misread gear decodes
 * to a jump of a multiple of the other gears' product -- hundreds of
 * turns -- which no real shaft makes between samples).  Sized for the
 * worst-case shaft speed at the 1 kHz sample cadence (TIM2.ARR in CubeMX,
 * tim.c): 1 turn per 1 ms sample allows 1000 turns/s (~360,000 deg/s).
 */
constexpr uint16_t MaxTurnsDelta = 1u;
} // namespace GearConfig

#endif /* APP_GEAR_CONFIG_H */