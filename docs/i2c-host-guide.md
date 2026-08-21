# I2C host guide

How to read the absolute multi-turn position from the encoder as an I2C
**master**.  The firmware is the I2C **slave** on address `0x50` (7-bit;
`0xA0` write / `0xA1` read on the wire).  This guide is the practical
walk-through with STM32 HAL example code; `docs/i2c.md` is the wire-level
spec and its numbers are normative.

## Bus

- Slave address `0x50` (7-bit).  It is fixed in firmware — there are no
  address pins, so put **at most one encoder per bus**, or use an I2C
  mux/multiplexer for several.
- Standard mode **100 kHz** by default; clock stretching is supported, so
  a slow master or long SCL low is fine.  Both sides can move to 400 kHz
  (on the firmware side that is a CubeMX-only change, `docs/i2c.md`).
- SCL/SDA are open-drain: the bus needs external pull-ups to 3.3 V.

## Register map (8 bytes)

| reg        | name   | meaning                                                       |
|------------|--------|---------------------------------------------------------------|
| 0x00..0x03 | pos    | absolute position count, uint32 LE, 0 .. 121,716,735          |
| 0x04       | status | one-byte verdict: bit0 = `valid`; bits 1..4 = read health     |
| 0x05..0x06 | seq    | sample counter, uint16 LE, +1 per 1 ms sample, wraps          |

Position is `turns * 16384 + angle`: turns 0..7428 from the gear phases,
angle 0..16383 from the input shaft.  The count is continuous across the
turn boundary and **absolute** — no homing, no calibration table, and it
survives power loss of the encoder.

The status byte carries the whole sample verdict in one register: it
reads exactly `0x01` when the last sample was **fresh and accepted**;
bits 1..4 name any encoder whose read failed — bit1 = Sun, bit2 = Gear1,
bit3 = Gear2, bit4 = Gear3; `0x00` means all reads were clean but the
slew guard rejected the decode.

Reading it is always three steps: write the register pointer (`0x00`),
read 7 bytes back (one combined transaction, or two separate ones — the
pointer persists), interpret.

## Trust rules (read before writing code)

- `status == 0x01` (valid bit set, nothing else) means: all four sensor
  reads succeeded **and** the decoded position passed the slew guard.
  Only then is the sample certified.
- `status` bits 1..4 set: an encoder read failed; the bits name it —
  bit1 = Sun, bit2 = Gear1, bit3 = Gear2, bit4 = Gear3.  The position
  bytes keep the last certified count.
- `status == 0x00`: all reads were clean, but the slew guard rejected
  the decode (transient misread); position bytes again keep the last
  certified count.
- Either rejection clears by itself on the next good sample — there are
  no commands or resets a host can issue (writes other than the register
  pointer do nothing).
- `seq` is the **liveness** signal: it ticks once per sample, accepted or
  not.  If it stops changing between your polls, the encoder's sample
  loop is dead and the value is stale — the one failure constant polling
  cannot detect any other way.  It wraps every ~65 s at 1 kHz: only
  compare "changed / unchanged", never the magnitude.

The causes behind each rejection — and what to check first — are in
[Troubleshooting](#troubleshooting-what-the-bytes-are-telling-you).

## STM32 HAL example (polling master)

### I2C init

CubeMX generates this inside `MX_I2C2_Init()`; the essentials are the
100 kHz clock speed and 7-bit addressing.  A slave own-address is not
needed on a master.

```c
/* your project ships this include already; it is shown here so the
 * example is copy-paste complete */
#include "stm32f1xx_hal.h"

/* inside MX_I2C2_Init() -- CubeMX generated, shown to make the bus
 * settings visible: 100 kHz standard mode, 7-bit addressing */
hi2c2.Instance = I2C2;
hi2c2.Init.ClockSpeed = 100000;      /* standard mode */
hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
hi2c2.Init.OwnAddress1 = 0;          /* master: unused */
hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
HAL_I2C_Init(&hi2c2);
```

### Reading a snapshot

```c
#include "stm32f1xx_hal.h" /* other families: include your part's HAL */

extern I2C_HandleTypeDef hi2c2; /* your master peripheral (CubeMX) */

/* Bus address, HAL style: the 7-bit address shifted left by one; HAL
 * appends the R/W bit (0xA0 write / 0xA1 read). */
#define ENC_ADDR (0x50u << 1)

#define ENC_REG_STATUS 0x04u
#define ENC_REG_SEQ    0x05u
#define ENC_FRAME_LEN  7u

#define ENC_STATUS_VALID 0x01u /* bit0: sample fresh and accepted */

/* bits 1..4: that encoder's read failed on the last sample */
#define ENC_STATUS_HEALTH_MASK 0x1Eu
#define ENC_STATUS_HEALTH_SUN   0x02u
#define ENC_STATUS_HEALTH_GEAR1 0x04u
#define ENC_STATUS_HEALTH_GEAR2 0x08u
#define ENC_STATUS_HEALTH_GEAR3 0x10u

typedef struct
{
    uint32_t Position; /* turns * 16384 + angle, absolute */
    uint8_t  Status;   /* bit0 valid, bits 1..4 read health */
    uint16_t Seq;      /* sample counter, liveness */
} EncoderSnapshot;

/* One EEPROM-style transaction: register pointer via repeated START,
 * then 7 bytes.  Fills *out and returns HAL_OK; HAL_ERROR means the
 * address was not ACKed or the transfer timed out (device absent, bus
 * stuck, pull-ups missing). */
HAL_StatusTypeDef EncoderReadSnapshot(I2C_HandleTypeDef* hi2c,
                                      EncoderSnapshot* out)
{
    uint8_t frame[ENC_FRAME_LEN];
    const uint32_t timeoutMs = 100u; /* generous for 100 kHz + stretching */

    if (HAL_I2C_Mem_Read(hi2c, ENC_ADDR, 0x00u, I2C_MEMADD_SIZE_8BIT,
                         frame, ENC_FRAME_LEN, timeoutMs) != HAL_OK)
    {
        return HAL_ERROR;
    }

    out->Position = (uint32_t)frame[0] | ((uint32_t)frame[1] << 8u) |
                    ((uint32_t)frame[2] << 16u) | ((uint32_t)frame[3] << 24u);
    out->Status = frame[ENC_REG_STATUS];
    out->Seq = (uint16_t)((uint16_t)frame[ENC_REG_SEQ] |
                          ((uint16_t)frame[ENC_REG_SEQ + 1u] << 8u));
    return HAL_OK;
}
```

### Using the snapshot (poll loop with the trust rules)

```c
/* Call once per poll period (e.g. every 10 ms).  Keeps the previous seq
 * locally to detect a stalled sample loop. */
static uint16_t lastSeq;
static uint8_t  firstPoll = 1u;

void PollEncoder(void)
{
    EncoderSnapshot snap;
    if (EncoderReadSnapshot(&hi2c2, &snap) != HAL_OK)
    {
        /* unreachable: check power, wiring, pull-ups; retry next poll */
        return;
    }

    if (firstPoll || snap.Seq != lastSeq)
    {
        firstPoll = 0u;
        lastSeq = snap.Seq;
    }
    else
    {
        /* seq frozen while we poll: the encoder's sample loop is dead
         * and the position bytes are stale, whatever status says */
        return;
    }

    if ((snap.Status & ENC_STATUS_VALID) != 0u)
    {
        /* status == 0x01 exactly: certified sample.  Conversions:
         *   turns    = snap.Position / 16384u
         *   angle    = snap.Position % 16384u
         *   degrees  = snap.Position * 360.0f / 16384.0f
         *   (fixed point: degrees*100 = snap.Position * 36000u / 16384u) */
        /* ...run your application on the certified position... */
    }
    else if ((snap.Status & ENC_STATUS_HEALTH_MASK) != 0u)
    {
        /* an encoder read failed: snap.Status bits 1..4 name it
         * (bit1 = Sun .. bit4 = Gear3); hold the last good position
         * and flag a fault */
    }
    else
    {
        /* status == 0x00: all reads clean but the slew guard rejected
         * this sample (transient misread); position bytes hold the
         * last certified value */
    }
}
```

## Practical notes

- **Partial reads are fine**: 4 bytes = position only, 5 = position +
  status.  Reading less than the full map never disturbs later reads
  (the pointer advances, never wraps; past the map everything reads 0).
- **Two transactions instead of one** also work — write the pointer, then
  read: `HAL_I2C_Master_Transmit(&hi2c2, ENC_ADDR, (uint8_t[]){0x00}, 1u,
  timeout)` followed by `HAL_I2C_Master_Receive(&hi2c2, ENC_ADDR, frame,
  ENC_FRAME_LEN, timeout)`.  The pointer persists between transactions.
  Note that after a STOP the snapshot may have advanced one sample — you
  still get one self-consistent snapshot, never a torn mix.
- **Non-blocking hosts**: `HAL_I2C_Mem_Read_IT` / `HAL_I2C_Mem_Read_DMA`
  take the same arguments; the layout and semantics are unchanged.
- **NACK on every transaction**: unit not powered or not on this bus —
  remember the address is fixed, so no two encoders may share a bus.
- **400 kHz**: set `ClockSpeed = 400000` on the master *and* regenerate
  the firmware side at 400 kHz (`docs/i2c.md`); the register map and the
  example code are unchanged.
- **Clock stretching**: enabled in the firmware by default, so `timeoutMs`
  above never trips on a slow bus — it exists to catch a wedged bus, not
  a stretched clock.

## Troubleshooting: what the bytes are telling you

The status byte holds two independent detectors in one register: bits 1..4
ask "did all four sensors answer cleanly?", the valid bit asks "is the
decoded position physically possible?".  Start from the decision table,
then follow the checklist.

| you observe                                        | meaning                                   | most likely cause                                   |
|----------------------------------------------------|-------------------------------------------|-----------------------------------------------------|
| status bits 1..4 set, `seq` ticking                | one encoder read failed                   | wiring/EMC on that encoder's SSI lines, magnet problem, dead chip |
| status = 0x1E                                      | all four reads failed                     | sensor farm power, shared bus wiring, multiple stuck chips |
| status = 0x00                                      | reads clean, decode implausible           | transient tooth-slot misread; if persistent: mechanical or speed |
| `seq` frozen between polls                         | sample loop is dead                       | firmware hang — position is stale, whatever the other bytes say |
| every transaction NACKs                            | slave did not ACK the address             | encoder unpowered, not on this bus, or pull-ups missing |

Status bits 1..4 follow EncoderRole order: bit1 = Sun, bit2 = Gear1,
bit3 = Gear2, bit4 = Gear3 — part of the wire contract, so a host can
name the failed sensor in fault reports.  Which physical connector/pin
that maps to is internal to the device (see the firmware docs, not this
guide).

### A status health bit (bits 1..4) is set — that encoder's read failed

The firmware checks each 24-bit frame twice: CRC-6 (did the bits arrive
intact?) and the chip's own status bits (is the magnet happy?).  Failures:

- **CRC-6 mismatch on all retries — corrupted frame:**
  - marginal MISO/SCK/CSN wiring on that encoder (the set bit names it)
  - EMC/noise on long leads, loose connector, cold solder joint
  - chip stuck (returns 0xFF forever): dead chip, missing 3.3 V, floating CSN
  - bus contention — two CSN lines low at once shorting the shared
    MISO/SCK (the firmware never does this; look for a CSN-CSN short)
- **Frame valid but chip reports a fault:**
  - track loss: magnet missing, fallen off, or farther than ~2 mm from the
    sensing center
  - field too strong/too weak: magnet gap wrong (0.5–2 mm per
    docs/MT6701.md), wrong magnet size, or a stray ferromagnetic part near
    the chip
- One sample with a bit set is a glitch and self-heals; the same bit set
  over many samples is one of the hardware causes above.

### `status` = 0x00 — the slew guard

- Transient (the common case): noise pushed one gear's count across a
  tooth-slot boundary during a move — the next sample usually recovers.
- Persistent: eccentric or loose magnet on a gear, gear-mesh damage, or
  the shaft genuinely faster than the guard's limit
  (`GearConfig::MaxTurnsDelta` = 1 turn per 1 ms sample ≈ 1000 turns/s).
- Note what the guard does *not* do: it keeps measuring from the last
  accepted turn count, so `valid` stays 0 until the decode lands within
  1 turn of it again — a persistent mechanical fault keeps `valid` at 0
  while the position bytes stay on the last certified count.

### `seq` stopped ticking

The counter describes *freshness*, not health — it stops when the
encoder's sample loop stops, whatever the status bytes claim.  Reset the
encoder.  A sudden drop back toward small `seq` values means the encoder
restarted; the position itself is still correct afterwards, because it is
absolute (the gears are the memory, docs/architecture.md).

### Every transaction NACKs

Encoder unpowered or off the bus, pull-ups missing (SCL/SDA are
open-drain), or an address mismatch: the encoder answers `0x50` (7-bit) —
with the STM32 HAL that is `0x50 << 1 = 0xA0`; passing raw `0x50`
silently addresses `0x28`.

> Firmware developers can reproduce every failure above in the PC sim:
> `sim/mt6701_slave_sim.cpp` injects track loss, bad field, stuck chips
> and CRC corruption (`SetTrackLoss` / `SetFieldStatus` / `SetStuck` /
> `SetCrcBroken`), and `sim/sim_main.cpp` checks the resulting byte
> patterns.