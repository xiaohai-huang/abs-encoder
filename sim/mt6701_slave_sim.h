/**
 * @file mt6701_slave_sim.h
 * @brief Simulated MT6701 chips: byte-level SSI slaves that build the
 *        real 24-bit frame so the firmware protocol layer runs unchanged.
 *        One independent chip per encoder index (0..MT6701_ENC_COUNT-1).
 */
#ifndef SIM_MT6701_SLAVE_SIM_H
#define SIM_MT6701_SLAVE_SIM_H

#include <stdbool.h>
#include <stdint.h>

/** Reset all chips (angle 0, no faults). */
void mt6701_slave_init(void);

/** CS low starts a frame on the given encoder; the next three transfers
 *  shift it out.  Mirrors the shared-bus layout: two chips per bus, so a
 *  second chip with its CS low would fight on MISO -- not modelled, the
 *  tests select one encoder at a time. */
void mt6701_slave_cs(uint8_t enc, bool asserted);

/** One byte in, one byte out (MISO), MSB first, for the given encoder. */
uint8_t mt6701_slave_transfer(uint8_t enc, uint8_t tx);

/* Test controls (per encoder index) ------------------------------------ */
void mt6701_slave_set_angle(uint8_t enc, uint16_t angle);       /* 0..16383 */
void mt6701_slave_set_button(uint8_t enc, bool on);
void mt6701_slave_set_track_loss(uint8_t enc, bool on);
void mt6701_slave_set_field_status(uint8_t enc, uint8_t status); /* 0 = normal */
void mt6701_slave_set_crc_broken(uint8_t enc, bool on);
void mt6701_slave_set_stuck(uint8_t enc, bool on);

#endif /* SIM_MT6701_SLAVE_SIM_H */