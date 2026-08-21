/**
 * @file mt6701_slave_sim.h
 * @brief Simulated MT6701 chips: byte-level SSI slaves that build the
 *        real 24-bit frame so the firmware protocol layer runs unchanged.
 *        One independent chip per encoder role (0..Mt6701::EncoderCount-1).
 */
#ifndef SIM_MT6701_SLAVE_SIM_H
#define SIM_MT6701_SLAVE_SIM_H

#include <cstdint>

#include "gear_config.h" /* EncoderRole */
#include "mt6701.h"      /* Mt6701::EncoderCount */

/** Simulated MT6701 SSI slaves; stateless facade over per-chip state. */
class Mt6701SlaveSim
{
public:
    /** Reset all chips (angle 0, no faults). */
    static void Init();

    /** CS low starts a frame on the given encoder; the next three transfers
     *  shift it out.  Mirrors the shared-bus layout: two chips per bus, so
     *  a second chip with its CS low would fight on MISO -- not modelled,
     *  the tests select one encoder at a time. */
    static void SelectChip(EncoderRole encoder, bool asserted);

    /** One byte in, one byte out (MISO), MSB first, for the given encoder. */
    static uint8_t Transfer(EncoderRole encoder, uint8_t txByte);

    /* Test controls (per encoder role) ---------------------------------- */
    static void SetAngle(EncoderRole encoder, uint16_t angle);       /* 0..16383 */
    static void SetButton(EncoderRole encoder, bool pressed);
    static void SetTrackLoss(EncoderRole encoder, bool enabled);
    static void SetFieldStatus(EncoderRole encoder, uint8_t status); /* 0 = normal */
    static void SetCrcBroken(EncoderRole encoder, bool broken);
    static void SetStuck(EncoderRole encoder, bool stuck);

private:
    struct SimulatedChip
    {
        uint16_t Angle;
        bool     IsButtonPressed;
        bool     HasTrackLoss;
        bool     IsCrcBroken;
        bool     IsStuck;
        uint8_t  FieldStatus;
        uint32_t Frame;     /* 24-bit frame, MSB first */
        int      ByteIndex; /* 0..2, -1 while CS is high */
    };

    static SimulatedChip _chips[Mt6701::EncoderCount];

    /* Rebuild the chip's 24-bit frame from its current state. */
    static void BuildFrame(SimulatedChip& chip);
};

#endif /* SIM_MT6701_SLAVE_SIM_H */