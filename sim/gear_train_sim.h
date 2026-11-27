/**
 * @file gear_train_sim.h
 * @brief Mechanical plant for the PC simulation: one input-shaft position
 *        drives all four MT6701 wheels through the gear train.
 *
 * Header-only so the chip simulator and the standalone decoder test share
 * one model of the hardware.  The model is written from the mechanics, not
 * from the decoder: a driven wheel turns InputTeeth/teeth revolutions per
 * input revolution, and its chip reports that wheel's absolute angle.
 */
#ifndef SIM_GEAR_TRAIN_SIM_H
#define SIM_GEAR_TRAIN_SIM_H

#include <cstdint>

#include "gear_config.h" /* GearConfig */

namespace GearTrainSim
{
/** MT6701 counts per wheel revolution. */
constexpr uint32_t CountsPerTurn = 1u << 14u;

/** One full cycle of the sensor readings: TurnRange input turns, the LCM of
 *  the driven tooth counts.  Wrapping here keeps the model's integer
 *  arithmetic in range and matches the hardware's aliasing. */
constexpr uint32_t ShaftRange = GearConfig::TurnRange * CountsPerTurn;

/** Input position wrapped into one sensor cycle, in 1/16384 turn counts. */
inline uint32_t WrapShaftPosition(uint32_t inputCounts)
{
    return inputCounts % ShaftRange;
}

/** Sun wheel angle: the Sun chip is 1:1 with the input shaft. */
inline uint16_t SunAngle(uint32_t inputCounts)
{
    return static_cast<uint16_t>(inputCounts % CountsPerTurn);
}

/** Driven wheel angle in counts.  The wheel turns InputTeeth/teeth
 *  revolutions per input revolution, so its fractional revolution is
 *  frac(inputCounts * InputTeeth / (teeth * 16384)).  inputCounts must be
 *  < ShaftRange (wrap it first). */
inline uint16_t WheelAngle(uint32_t inputCounts, uint16_t teeth)
{
    uint32_t wheelCounts =
        (GearConfig::InputTeeth * inputCounts) % (teeth * CountsPerTurn);
    return static_cast<uint16_t>((wheelCounts + teeth / 2u) / teeth);
}
} // namespace GearTrainSim

#endif /* SIM_GEAR_TRAIN_SIM_H */
