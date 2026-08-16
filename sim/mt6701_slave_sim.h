/**
 * @file mt6701_slave_sim.h
 * @brief Simulated MT6701 chip: a byte-level SSI slave that builds the
 *        real 24-bit frame so the firmware protocol layer runs unchanged.
 */
#ifndef SIM_MT6701_SLAVE_SIM_H
#define SIM_MT6701_SLAVE_SIM_H

#include <stdbool.h>
#include <stdint.h>

/** Reset chip state (angle 0, no faults). */
void mt6701_slave_init(void);

/** CS low starts a frame; the next three transfers shift it out. */
void mt6701_slave_cs(bool asserted);

/** One byte in, one byte out (MISO), MSB first. */
uint8_t mt6701_slave_transfer(uint8_t tx);

/* Test controls -------------------------------------------------------- */
void mt6701_slave_set_angle(uint16_t angle);       /* 0..16383 */
void mt6701_slave_set_button(bool on);
void mt6701_slave_set_track_loss(bool on);
void mt6701_slave_set_field_status(uint8_t status); /* 0 = normal */
void mt6701_slave_set_crc_broken(bool on);          /* corrupt the CRC field */
void mt6701_slave_set_stuck(bool on);               /* MISO stuck high */

#endif /* SIM_MT6701_SLAVE_SIM_H */
