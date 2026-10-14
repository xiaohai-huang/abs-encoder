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
  In this system the two pull-ups (2 × 4.7 kΩ at 100 kHz) are fitted on
  the **host** PCB, to the host's 3.3 V rail — the encoder board ships
  with its pull-up footprints unpopulated (`docs/pcb-design-guide.md` §6).

## Register map (19 bytes + read diagnostics)

| reg        | name   | meaning                                                       |
|------------|--------|---------------------------------------------------------------|
| 0x00..0x03 | pos    | absolute position count, uint32 LE, 0 .. 121,716,735          |
| 0x04       | status | one-byte verdict: bit0 = `valid`; bits 1..4 = read health     |
| 0x05..0x06 | seq    | sample counter, uint16 LE, +1 per 1 ms sample, wraps          |
| 0x07..0x0A | max    | full-scale count, uint32 LE: the constant 121,716,735         |
| 0x0B..0x12 | gears  | each gear's measured angle, uint16 LE (Sun, Gear1..Gear3)     |
| 0x13..0x32 | diag   | per-wheel read diagnostics: cause, chip status, failure counters, last-failure seq (8 bytes per wheel) |

Position is `turns * 16384 + angle`: turns 0..7428 from the gear phases,
angle 0..16383 from the input shaft.  The count is continuous across the
turn boundary and **absolute** — no homing, no calibration table, and it
survives power loss of the encoder.

In resolution terms this is a **27-bit absolute encoder over a 7,429-turn
range**: the turn field is 13 bits (7,429 states in an 8,192-code field)
and the fine angle is 14 bits (16,384 counts per turn, ≈ 0.022° per
count); 13 + 14 is the 27 bits of the count above.  Those 7,429 turns
are a **modulo** range, not a limit the firmware enforces: past turn
7,428 the decode aliases back to 0, and because the gear phases are the
only memory the aliased reading is a perfectly valid sample.  A
mechanical end-stop is assumed — a host that must track more than 7,429
turns has to detect the wrap itself.

The status byte carries the whole sample verdict in one register: it
reads exactly `0x01` when the last sample was **fresh and accepted**;
bits 1..4 name any encoder whose read failed — bit1 = Sun, bit2 = Gear1,
bit3 = Gear2, bit4 = Gear3; `0x00` means all reads were clean but the
slew guard rejected the decode.

The last register is the **full-scale count**: the same value the extreme
sample reports (`7428 * 16384 + 16383 = 121,716,735`), exported as a
constant so you scale positions against the device's own range instead of
a number copied into your firmware.  Read it once at startup — it needs
no polling, never changes with the shaft, and reads the same before the
first sample and while an encoder is faulted.

Reading it is always three steps: write the register pointer you want
(`0x00` for the snapshot, `0x07` for the full-scale count), read the
bytes back (one combined transaction, or two separate ones — the pointer
persists), interpret.

### Wheel index (0..3)

Everything on this map that comes "once per wheel" — the status health
bits, the gear angles, the diagnostics blocks — shares one index, in
EncoderRole order (the firmware's name for that fixed order).  The
example code below passes it as the `wheel` argument, spelled
`ENC_WHEEL_SUN`..`ENC_WHEEL_GEAR3`:

| wheel | role  | wheel itself      | status bit | angle reg | diagnostics reg |
|-------|-------|-------------------|------------|-----------|-----------------|
| 0     | Sun   | 13T input shaft   | bit1 (`0x02`) | `0x0B` | `0x13` |
| 1     | Gear1 | 17T driven gear   | bit2 (`0x04`) | `0x0D` | `0x1B` |
| 2     | Gear2 | 19T driven gear   | bit3 (`0x08`) | `0x0F` | `0x23` |
| 3     | Gear3 | 23T driven gear   | bit4 (`0x10`) | `0x11` | `0x2B` |

For index `n`: gear angle = `0x0B + 2n`, diagnostics block = `0x13 + 8n`,
health bit = `1 << (n + 1)`.  So `wheel = 2` (`ENC_WHEEL_GEAR2`) is
always the 19-tooth gear: its angle reads at `0x0F`, its failure story
lives at `0x23`, and its health bit is `0x08` in the status byte.  The
tooth counts identify the physical wheel and come from the firmware's
gearbox configuration (`App/gear_config.h`); which connector or pin a
wheel sits behind is board wiring, not wire protocol
(`docs/pcb-design-guide.md` §1).

### Each gear's own angle (0x0B..0x12)

The four trailing registers hold the wheels' raw readings, for when `pos`
alone does not explain what the hardware is doing: one uint16 LE per wheel,
in EncoderRole order — Sun at `0x0B`, Gear1 at `0x0D`, Gear2 at `0x0F`,
Gear3 at `0x11`.  Each is the 14-bit angle (0 .. 16383) that wheel's own
MT6701 reported on the same sample as `pos`/`status`/`seq`.

They are the readings the decode consumed, so they say *which* wheel is
unhappy when the position cannot: the Sun register is the low 14 bits of
`pos`, and the three driven registers are the gear phases behind its turn
count (a driven wheel turns 13/teeth of a turn per input turn — Gear1 runs
13/17 as fast as the input).  Diagnostic only: `pos` (0x00) stays the
certified position, and a wheel whose read failed keeps its previous angle
here — the status health bit names it, so a held value is never mistaken
for a live one (wire-level details: `docs/i2c.md`; the hardware side:
`docs/pcb-design-guide.md` §10).

### Read diagnostics (0x13..0x32)

When a status health bit trips, the eight bytes behind the gear angles
say why: one block per wheel, in EncoderRole order — Sun at `0x13`,
Gear1 at `0x1B`, Gear2 at `0x23`, Gear3 at `0x2B`:

| offset    | name        | meaning                                                         |
|-----------|-------------|-----------------------------------------------------------------|
| +0        | cause       | 0 = read OK; 1 = magnetic fault (frame fine, chip unhappy with the magnet); 2 = no valid frame (CRC-6 failed every retry) |
| +1        | mg          | the chip's raw status Mg[3:0] of that read: `0x08` track loss, `0x01` field too strong, `0x02` field too weak, `0xFF` = no CRC-valid frame |
| +2..+3    | frameFails  | cause-2 samples since power-up, uint16 LE, saturates at 65,535  |
| +4..+5    | statusFails | cause-1 samples since power-up, uint16 LE, saturates at 65,535  |
| +6..+7    | lastFailSeq | `seq` when this wheel last failed, uint16 LE; 0 = never         |

How to read it:

- **cause 2 (frameFails climbing) is electrical** — wiring, noise,
  connector, contention or a dead chip returning 0xFF.  **cause 1
  (statusFails climbing) is magnetic** — and `mg` says which: gap wrong
  (too strong / too weak) or the magnet gone (track loss).
- The counters are cumulative since power-up and never reset (nothing on
  this map is writable): an occasional failure and a chronic one differ
  only in these numbers, so diff them between your polls to get a rate.
- `lastFailSeq` lets you place the failure in time — correlate it with
  the `seq`/`pos` history you keep to decide whether it happens at speed,
  under load, or at one angle of the rotation.  It updates when that
  wheel fails again; `seq` wraps every ~65 s, so compare it against a
  `seq` you observed, not against wall time.
- On any one sample, health bit set ⇔ that wheel's cause ≠ 0 (both are
  derived from the same read), so the block always explains the byte at
  `0x04` of the same transaction.

Read 8 bytes from `0x13 + 8·wheel` for one wheel or 32 from `0x13` for
all four; a 51-byte read from `0x00` captures position, status, seq and
every block from one sample.

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
- The diagnostics block (`0x13..0x32`) turns any health bit into a story:
  `cause` says whether it was magnetic (1, see `mg`) or a dead frame (2),
  the counters say whether it is one glitch or a chronic failure, and
  `lastFailSeq` says when it happened (see the section above).

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
#define ENC_REG_MAX_POSITION 0x07u
#define ENC_REG_GEAR_ANGLE 0x0Bu /* Sun's angle; +2 bytes per further wheel */

#define ENC_GEAR_ANGLE_LEN 2u   /* one uint16 per wheel */
#define ENC_GEAR_ANGLE_COUNT 4u /* Sun, Gear1, Gear2, Gear3 */

/* The shared wheel index (the "Wheel index" section): EncoderRole order,
 * the same numbering behind the status health bits, the gear-angle
 * registers and the `wheel` argument of EncoderReadWheelDiagnostics() */
#define ENC_WHEEL_SUN   0u /* 13T input shaft: angle 0x0B, diag 0x13, bit1 */
#define ENC_WHEEL_GEAR1 1u /* 17T driven:      angle 0x0D, diag 0x1B, bit2 */
#define ENC_WHEEL_GEAR2 2u /* 19T driven:      angle 0x0F, diag 0x23, bit3 */
#define ENC_WHEEL_GEAR3 3u /* 23T driven:      angle 0x11, diag 0x2B, bit4 */

/* the polling snapshot: position + status + seq, in ONE transaction --
 * a transaction copies one snapshot, so the three always describe the
 * same sample (split reads could pair a position with the next sample's
 * status).  Status is that sample's verdict; seq is liveness, which no
 * status byte can express: a dead sample loop's last status reads like a
 * healthy one.  The main map is 11 bytes (the full-scale count follows
 * the counter at 0x07, the gear angles after that); a poll needs neither:
 * the first never changes, the second is diagnostics only */
#define ENC_FRAME_LEN 7u

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

/* The full-scale count: the same number the extreme position reports.
 * Read once at startup with a 4-byte read at pointer 0x07 -- unlike the
 * snapshot there is nothing to validate, it never changes. */
HAL_StatusTypeDef EncoderReadMaxPosition(I2C_HandleTypeDef* hi2c,
                                         uint32_t* out)
{
    uint8_t frame[4];

    if (HAL_I2C_Mem_Read(hi2c, ENC_ADDR, ENC_REG_MAX_POSITION,
                         I2C_MEMADD_SIZE_8BIT, frame, sizeof(frame),
                         100u) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *out = (uint32_t)frame[0] | ((uint32_t)frame[1] << 8u) |
           ((uint32_t)frame[2] << 16u) | ((uint32_t)frame[3] << 24u);
    return HAL_OK;
}

/* Each gear's own 14-bit angle, one uint16 LE per wheel in EncoderRole
 * order: 8 bytes from pointer 0x0B.  Diagnostic data -- the certified
 * position is still the 4-byte read at 0x00; read both in ONE transaction
 * (19 bytes from 0x00) when you want to compare a wheel against it.  A
 * wheel whose status health bit is set is showing a held angle. */
HAL_StatusTypeDef EncoderReadGearAngles(I2C_HandleTypeDef* hi2c,
                                        uint16_t angles[ENC_GEAR_ANGLE_COUNT])
{
    uint8_t frame[ENC_GEAR_ANGLE_COUNT * ENC_GEAR_ANGLE_LEN];
    uint8_t wheel;

    if (HAL_I2C_Mem_Read(hi2c, ENC_ADDR, ENC_REG_GEAR_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, frame, sizeof(frame),
                         100u) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (wheel = 0u; wheel < ENC_GEAR_ANGLE_COUNT; wheel++)
    {
        angles[wheel] = (uint16_t)((uint16_t)frame[2u * wheel] |
                                   ((uint16_t)frame[2u * wheel + 1u] << 8u));
    }
    return HAL_OK;
}

/* One wheel's read diagnostics: 8 bytes from pointer 0x13 + 8*wheel,
 * where wheel is the shared index from the "Wheel index" section
 * (ENC_WHEEL_SUN = 0x13 .. ENC_WHEEL_GEAR3 = 0x2B).  Call it when a
 * status health bit trips: cause 1 = magnetic (mg says track loss 0x08 /
 * field too strong 0x01 / too weak 0x02), 2 = no frame passed CRC-6.
 * The counters saturate at 65535 and reset only on power-up; lastFailSeq
 * is the seq value of that wheel's last failed sample (0 = never). */
typedef struct
{
    uint8_t  Cause;      /* 0 = ok, 1 = magnetic, 2 = no valid frame */
    uint8_t  MgStatus;   /* Mg[3:0], or 0xFF when the frame never validated */
    uint16_t FrameFails; /* cause-2 samples since power-up (saturating) */
    uint16_t StatusFails;/* cause-1 samples since power-up (saturating) */
    uint16_t LastFailSeq;/* seq at the last failure, 0 = never */
} EncoderWheelDiagnostics;

HAL_StatusTypeDef EncoderReadWheelDiagnostics(I2C_HandleTypeDef* hi2c,
                                              uint8_t wheel,
                                              EncoderWheelDiagnostics* out)
{
    uint8_t frame[8];
    uint8_t lo, hi;

    if (HAL_I2C_Mem_Read(hi2c, ENC_ADDR,
                         (uint8_t)(0x13u + 8u * wheel),
                         I2C_MEMADD_SIZE_8BIT, frame, sizeof(frame),
                         100u) != HAL_OK)
    {
        return HAL_ERROR;
    }

    out->Cause = frame[0];
    out->MgStatus = frame[1];
    lo = frame[2];
    hi = frame[3];
    out->FrameFails = (uint16_t)((uint16_t)lo | ((uint16_t)hi << 8u));
    lo = frame[4];
    hi = frame[5];
    out->StatusFails = (uint16_t)((uint16_t)lo | ((uint16_t)hi << 8u));
    lo = frame[6];
    hi = frame[7];
    out->LastFailSeq = (uint16_t)((uint16_t)lo | ((uint16_t)hi << 8u));
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
         *   (fixed point: degrees*100 = (uint64_t)snap.Position * 36000u /
         *    16384u; without the widening cast the product overflows
         *    uint32 above ~119,000 counts)
         *   percent of full travel: (uint64_t)snap.Position * 100u /
         *    MaxPosition -- MaxPosition is what EncoderReadMaxPosition()
         *    returned at startup; widen here too) */
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
  status, 7 = the poll frame, 11 = the main map; the four gear angles are
  8 bytes from pointer `0x0B`, and one wheel's diagnostics are 8 bytes
  from `0x13 + 8·wheel` (32 for all four).  Reading less than the full
  map never disturbs later reads (the pointer advances, never wraps; past
  the map everything reads 0).  The full-scale count is a 4-byte read at
  pointer `0x07` on its own, which a poll never needs.
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
decoded position physically possible?".  When both answer yes but the
mechanics are still suspect, the gear angles (0x0B..0x12) answer the next
question — "what does each wheel see?" — one register per wheel, and the
diagnostics block (0x13..0x32) answers "and why does its read fail?" —
cause, chip status, failure counters and last-failure seq, also one block
per wheel.  Start from the decision table, then follow the checklist.

| you observe                                        | meaning                                   | most likely cause                                   |
|----------------------------------------------------|-------------------------------------------|-----------------------------------------------------|
| status bits 1..4 set, `seq` ticking                | one encoder read failed                   | read that wheel's `cause` byte: 2 = electrical (wiring/EMC/dead chip), 1 = magnet problem |
| diag `cause` = 2 with `frameFails` climbing         | frames never pass CRC-6                   | marginal SSI wiring, noise, connector, chip stuck at 0xFF |
| diag `cause` = 1 with `statusFails` climbing        | the chip keeps rejecting the magnet       | gap/alignment (`mg` 0x01 too strong, 0x02 too weak, 0x08 track loss) |
| status = 0x1E                                      | all four reads failed                     | sensor farm power, shared bus wiring, multiple stuck chips |
| status = 0x00                                      | reads clean, decode implausible           | transient tooth-slot misread; if persistent: mechanical or speed |
| a gear angle tracks at the wrong rate for its gear | one wheel disagrees with the decode       | magnet, mesh or wrong gear on that wheel (see the gear-angle section) |
| `seq` frozen between polls                         | sample loop is dead                       | firmware hang — position is stale, whatever the other bytes say |
| every transaction NACKs                            | slave did not ACK the address             | encoder unpowered, not on this bus, or pull-ups missing |

Status bits 1..4 follow EncoderRole order: bit1 = Sun, bit2 = Gear1,
bit3 = Gear2, bit4 = Gear3 — part of the wire contract, so a host can
name the failed sensor in fault reports.  Which physical connector/pin
that maps to is internal to the device (see the firmware docs, not this
guide).

### A status health bit (bits 1..4) is set — that encoder's read failed

The firmware checks each 24-bit frame twice: CRC-6 (did the bits arrive
intact?) and the chip's own status bits (is the magnet happy?).  The
diagnostics block already separates the two outcomes: read `cause` at
`0x13 + 8·wheel` — **2 = the first group below (no frame), 1 = the
second (magnetic)** — with `mg`, the failure counters and `lastFailSeq`
for the detail.  Failures:

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

### A gear angle looks wrong

The gear angles (0x0B..0x12, above) are the per-wheel view: `pos` can look
plausible while one wheel disagrees with the decode.

- **The Sun angle is the low 14 bits of `pos`** — if those disagree, the
  host's own frame decode is wrong, not the device.
- **A driven angle tracking at the wrong rate** (each should advance
  13/17, 13/19 or 13/23 of a turn per input turn): eccentric or tilted
  magnet on that wheel, gear-mesh damage, or a wheel whose tooth count
  does not match the firmware's gearbox (`App/gear_config.h`) — wrong gear
  fitted, teeth miscounted.
- **All three driven angles wrong while `status` never certifies**: the
  sensors are probably under the wrong gear axes — check the CSN offset
  warning in docs/pcb-design-guide.md §1 (CSN1 is the Sun, CSN2..CSN4 are
  Gear1..Gear3).
- **An angle frozen while the shaft moves, health bit set**: that wheel's
  read is failing (see the health-bit section above) — the register shows
  the last angle it reported, not a fresh one.

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