# I2C host port

The decoded absolute position (docs/architecture.md) is exported as an I2C
slave with a small register map, so any master on the bus can read the
multi-turn position on demand.  Nothing is pushed: the encoder answers reads
when addressed, like any encoder IC (AS5600-style).  Behind the position
registers, one register per gear carries that wheel's own angle, and behind
those, one 8-byte block per gear carries that wheel's read diagnostics —
why its reads fail, how often, and when (below).

> Engineers integrating a host master: the practical walk-through with
> STM32 HAL example code is in **docs/i2c-host-guide.md**.  This file is
> the wire-level spec; its numbers are normative.

## Bus facts

- Slave address: **0x50** (7-bit; `PositionRegister::Address` in `App/i2c_pos.h`).  On the
  wire that is `0xA0` for write and `0xA1` for read transactions.
- Standard mode, **100 kHz**, clock stretching enabled.
- Pins: **PB6 (SCL), PB7 (SDA)**, open-drain alternate function.  The bus
  needs external pull-ups to 3.3 V (the STM32's weak internal pulls are not
  enabled).
- The position snapshot updates every 1 ms (TIM2 sample clock).  Each
  transaction copies one snapshot at address-match time, so a read can never
  see bytes from two different samples.

## Register map

All registers are read-only, little-endian on the wire:

| reg        | name   | meaning                                                        |
|------------|--------|----------------------------------------------------------------|
| 0x00..0x03 | pos    | absolute position count, uint32 LE, 0 .. 121,716,735           |
| 0x04       | status | one-byte verdict: bit0 = `valid`; bits 1..4 = read health      |
| 0x05..0x06 | seq    | sample counter, uint16 LE: +1 per sample, wraps                |
| 0x07..0x0A | max    | full-scale count, uint32 LE: the constant 121,716,735          |
| 0x0B..0x12 | gears  | each wheel's measured angle, uint16 LE, one per role |
| 0x13..0x32 | diag   | per-wheel read diagnostics: cause, chip status, failure counters, last-failure seq (8 bytes per wheel) |

The status byte carries the whole sample verdict in one register: it
reads exactly `0x01` when the last sample was **fresh and accepted** (all
four encoder reads succeeded AND the slew guard passed); bits 1..4 name
any encoder whose read failed — bit1 = Sun, bit2 = Gear1, bit3 = Gear2,
bit4 = Gear3 (EncoderRole order); `0x00` means all reads were clean but
the slew guard rejected the decode.  `seq` increments once per sample
whether the sample was accepted or not.

That role order is also the map's **wheel index** 0..3, and it numbers
every per-wheel view the same way — health bit `1 << (n+1)`, gear angle
at `0x0B + 2n`, diagnostics block at `0x13 + 8n`, for n = 0 (Sun, 13T
input shaft) .. 3 (Gear3, 23T; Gear1 = 17T, Gear2 = 19T).  Wheel 2 is
therefore always the 19-tooth gear: status bit3 (`0x08`), angle `0x0F`,
diagnostics `0x23`.

The position count is the whole input-shaft travel in fine-angle steps:
`counts = turns * 16384 + angle`, where `turns` (0 .. 7428) comes from the
gear phases and `angle` (0 .. 16383) is the 14-bit fine angle within the
turn.  The combined value is a 27-bit integer that counts continuously
across the turn boundary — it never jumps at the seam.

The `max` register is the full-scale count of this build: the value a
maximally wound shaft reports, which is the same number as the extreme
sample (`7428 * 16384 + 16383 = 121,716,735`).  It is served as a
constant so a host scales positions against the device's real range
instead of a number copied into its own source; it changes only if the
gear geometry (`App/gear_config.h`) changes, it is unaffected by any
sample, and it reads the same before the first sample and during a
fault.

Conversions, on the host:

```
counts  = turns * 16384 + angle
turns   = counts / 16384
angle   = counts % 16384
degrees = counts * 360 / 16384

percent = counts * 100 / max    <- widen to 64-bit first: counts * 100
                                   overflows uint32 above ~42.9 M counts
```

## Gear angles (0x0B..0x12)

One register per wheel, in EncoderRole order — Sun at 0x0B, Gear1 at 0x0D,
Gear2 at 0x0F, Gear3 at 0x11 (`PositionRegister::GearAngleRegister`
computes these):

| reg        | wheel | value                                          |
|------------|-------|------------------------------------------------|
| 0x0B..0x0C | Sun   | the 13-tooth input shaft's fine angle, uint16 LE, 0 .. 16383 |
| 0x0D..0x0E | Gear1 | the 17-tooth wheel's fine angle                |
| 0x0F..0x10 | Gear2 | the 19-tooth wheel's fine angle                |
| 0x11..0x12 | Gear3 | the 23-tooth wheel's fine angle                |

Each value is the raw 14-bit angle that wheel's own MT6701 reported, from
the same sample as `pos`/`status`/`seq` (one transaction, one sample) — the
very readings the decode consumed.  They are useful on bring-up and in the
field when the decoded position alone cannot say *which* wheel is unhappy:
the Sun register repeats the low 14 bits of `pos`, and the three driven
registers show the gear phases behind its turn count (a driven wheel turns
13/teeth of a turn per input turn, so Gear1 runs 13/17 as fast as the
input shaft).

The registers are diagnostic only: `pos` (0x00) stays the certified result,
and a host that only wants the position can ignore 0x0B..0x12 entirely.  A
wheel whose read failed keeps its previous angle here (there is no fresh
measurement) — the status health bit names that wheel, so a held value is
never mistaken for a live one.

## Read diagnostics (0x13..0x32)

One 8-byte block per wheel, in EncoderRole order — Sun at 0x13, Gear1 at
0x1B, Gear2 at 0x23, Gear3 at 0x2B (`PositionRegister::DiagnosticsRegister`
computes these).  The status byte says *that* a wheel's read failed; the
block says *why, how often, and when*:

| offset    | name        | meaning                                                                  |
|-----------|-------------|--------------------------------------------------------------------------|
| +0        | cause       | how that wheel's last read ended (table below)                           |
| +1        | mg          | raw MT6701 status Mg[3:0] of the frame that decided the read; `0xFF` = no CRC-valid frame |
| +2..+3    | frameFails  | samples that ended with cause 2 since power-up, uint16 LE, saturating    |
| +4..+5    | statusFails | samples that ended with cause 1 since power-up, uint16 LE, saturating    |
| +6..+7    | lastFailSeq | `seq` value of the last sample this wheel failed on, uint16 LE; 0 = never |

The firmware validates every frame twice — CRC-6 first, then the chip's own
status field (docs/MT6701.md) — and the two ways a read can fail are kept
apart because they point at different hardware:

| cause | name      | what happened                                             | points at                                          |
|-------|-----------|-----------------------------------------------------------|----------------------------------------------------|
| 0     | ok        | the read succeeded                                        | —                                                  |
| 1     | magnetic  | frame passed CRC-6, but the chip reported track loss or bad field | magnet: gap, alignment, strength, missing magnet |
| 2     | no frame  | no frame passed CRC-6 in the three retries                | electrical: wiring, noise, contention, dead chip   |

`mg` decodes the status field of the frame that decided the read (raw
Mg[3:0], docs/MT6701.md): bit3 = track loss, bits 1..0 = field strength
(0 normal, 1 too strong, 2 too weak), bit2 = the push flag this firmware
ignores.  Next to cause 1 it says *which* magnetic problem: `0x08` =
track loss (magnet gone or too far), `0x01` = field too strong (gap too
small), `0x02` = field too weak (gap too large).  `0xFF` means the sample
produced no CRC-valid frame (cause 2), so there was no status to show;
next to cause 0 it is the healthy frame's status, normally 0.

The counters are cumulative since power-up and **saturate** at 65,535
instead of wrapping — they are RAM state only (battery-free design: no
persistent state), and a saturating counter never makes a chronic problem
look like a fresh start.  Together the pair tells the failure rate *and*
its mix: occasional `frameFails` are typical of noise, a climbing
`statusFails` of a magnet walking out of range.  A host that polls
periodically can diff them against its own clock to get a failure rate.

`lastFailSeq` stamps the `seq` register (0x05) at the sample where the
wheel last failed, so a host that also tracks seq (and `pos`) can locate
the failure in time: is it tied to shaft speed, or to one spot in the
rotation (an eccentric magnet)?  It holds until that wheel fails again
and resets to 0 only on power-up.  `seq` wraps every ~65 s, so read it
relative to a seq value you observed, never as an absolute timestamp.

Consistency: the health bits at 0x04 (bits 1..4) and every cause byte are
derived from the same per-wheel outcomes of the same sample — a health
bit set ⇔ that wheel's cause ≠ 0 in the same transaction.  So reading the
block turns "Gear2's read failed" into "Gear2 failed on a magnetic fault
(mg 0x02, field too weak) — its 47th status failure, last seen at seq
12,340".

Read them like the gear angles: 8 bytes from pointer `0x13 + 8·wheel`
diagnoses one wheel, 32 bytes from `0x13` gets all four — and, like the
rest of the map, one transaction is one sample, so cause/counters/seq of a
51-byte read from `0x00` agree with pos/status/seq of that same read.

## Transactions (EEPROM-style)

- **Write** (1 byte): sets the register pointer (`0x00`..`0x32`; anything
  past the map clamps to it, reads then return 0x00).
- **Read**: serves bytes starting at the register pointer and
  auto-increments; once past 0x32 every byte reads 0x00 (no wrap).

Read the whole snapshot with a combined transaction:

```
write 0xA0, byte 0x00,    <- register pointer
(repeated start)
read  0xA1, 11 bytes      <- main map: position (4) + status + seq (2) + max (4)
                             (19 bytes = through the gear angles at 0x12,
                              51 = the whole map incl. diagnostics at 0x32)
```

Common patterns: 4 bytes = position only; 5 = position + status; 7 = the
poll frame (position + status + seq); 11 = the main map; 8 = the four gear
angles from `0x0B` (one uint16 per wheel, role order); 8 from `0x13` = one
wheel's diagnostics (32 = all four); 19 = position through the gear
angles; 51 = the whole map.
The full-scale count on its own is 4 bytes from pointer `0x07`, which is
how a host reads it once at startup.
Writing the pointer and reading it in two separate transactions works too.
A master that stops early (e.g. reads 1 byte after selecting 0x00) is
harmless — the slave re-arms at the end of the transaction.

A host that polls should read the 7-byte frame, and in a **single**
transaction: one transaction copies one snapshot, so position, status
and `seq` always describe the same sample, while three separate reads
could pair a position with a later sample's status.  Each byte earns its
place — the count alone is not trustworthy without the status verdict,
status alone cannot reveal a dead sample loop (its last byte reads like
a healthy one, which is what `seq` is for), and `max` is left out
because it is a constant the host reads once at startup.  The gear angles
obey the same rule: read them in the same transaction as `pos` when you
want to compare a wheel against the decode, and leave them out of a poll
that only needs the certified count.  The diagnostics block likewise
describes the sample it is read with — pull it in the same transaction
when a health bit has caught your attention, so the cause you read names
the same sample that raised the bit.

## Behaviour notes

- Position bytes are always the latest sample.  The status valid bit
  clears when the slew guard rejects a sample (misread gear, see
  docs/architecture.md) or when any encoder read fails on that sample;
  the position bytes keep the last exported count — a host can decide
  whether to keep tracking on a stale reading.  Status bits 1..4 say
  which encoder(s) failed.  The gear angles are that same latest sample's
  four readings — including a rejected one, whose wheels a host can compare
  against the rejected count — but a wheel whose read failed keeps the
  previous angle in its register, since there is no fresh measurement to
  show.
- The sample counter (`seq`) is the liveness signal: it ticks once per
  1 ms sample whether the sample was accepted or not.  A host that polls
  at least once per counter wrap (~65 s) can detect a stalled sample loop
  by watching it stop changing — the one thing that distinguishes "shaft
  at rest" from "sampling died".  A read that crosses a wrap simply sees
  the new (lower) count; no transaction ever tears, because the whole map
  is copied at address-match time.
- Before the first sample the map reads all zeros (including `seq`, the
  gear angles and the diagnostics counters) and `status.valid = 0`.
- The address, once configured, survives CubeMX regeneration: the generated
  `i2c.c` keeps `OwnAddress1 = 0`; `hal_stm32.cpp` applies `PositionRegister::Address` at
  runtime during `hal.Init()`.

## Design decisions

- **Read-only map, no commands.** A write to the slave only ever selects the
  register pointer; there are no writable registers and no command codes.
  The device is deliberately a pure position source: zeroing, direction
  inversion and any other host-side math belong to the master.
- **Each gear's own angle rides along.** The four MT6701 readings are
  exported next to the decoded position, so bring-up and field problems are
  diagnosed over the same two wires that read the position
  (docs/pcb-design-guide.md §10) — no SWD session, no special firmware, no
  extra sensor traffic: the angles are copied from the sample that was
  taken anyway, and a host that does not read them pays nothing.
- **The two failure causes stay apart.** Every frame is checked twice
  (CRC-6, then the chip's status), and a `frameFails`/`statusFails` split
  plus the cause byte preserve that distinction on the wire: "no valid
  frame" and "magnetic fault" have different physical causes, so merging
  them into one failure bit would throw away the answer the diagnostics
  exist to give.  The raw Mg status rides along so a magnetic fault can be
  narrowed to track loss vs field too strong/too weak without an SWD
  session.
- **Failure history is RAM, saturating, read-only.** Counters reset on
  power-up (the battery-free principle forbids persistent state) and
  saturate at 0xFFFF rather than wrap (a wrapped counter would read as a
  sudden recovery), and no host can clear them — the map stays a pure
  view of what the device is doing.
- **No calibration or reset in firmware.** The decoded position is absolute
  — it never needs an initial "home" write, and the battery-free principle
  (no persistent state, docs/architecture.md) forbids storing an offset on
  the device. The register pointer is RAM and resets on power-up; the
  position itself never resets, because the gears are the memory.
- **No push-button feature.** The MT6701 push-magnet flag (Mg[2], frame bit
  8) is not part of this product; the firmware ignores it.
- **No transport CRC.** The map deliberately has no payload checksum:
  I2C's byte-level ACK covers framing on the wire, sample integrity is
  already guaranteed before publishing (SSI CRC-6, chip status, slew
  guard), and liveness is covered by the sample counter (0x05).  If the
  bus environment ever demands it, an SMBus-style PEC byte can be
  appended without breaking the map.

### Valve position example

To log how far a valve has opened:

1. Record `closed` and `open` — the counts read when the valve is fully
   closed and fully open (single 4-byte read each).
2. Percent open: `(counts - closed) * 100 / (open - closed)`, clamped to
   0..100.

Only the two reference readings are ever needed: the count itself is
absolute and needs no calibration table, and it never resets on power
loss.

## Raising the speed to 400 kHz

Set `I2C1.SpeedClock` to 400000 in CubeMX and regenerate (`Core/Src/i2c.c`
is the only timing source).  The slave transport itself is speed-independent.