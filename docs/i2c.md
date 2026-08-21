# I2C host port

The decoded absolute position (docs/architecture.md) is exported as an I2C
slave with a small register map, so any master on the bus can read the
multi-turn position on demand.  Nothing is pushed: the encoder answers reads
when addressed, like any encoder IC (AS5600-style).

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

The status byte carries the whole sample verdict in one register: it
reads exactly `0x01` when the last sample was **fresh and accepted** (all
four encoder reads succeeded AND the slew guard passed); bits 1..4 name
any encoder whose read failed — bit1 = Sun, bit2 = Gear1, bit3 = Gear2,
bit4 = Gear3 (EncoderRole order); `0x00` means all reads were clean but
the slew guard rejected the decode.  `seq` increments once per sample
whether the sample was accepted or not.

The position count is the whole input-shaft travel in fine-angle steps:
`counts = turns * 16384 + angle`, where `turns` (0 .. 7428) comes from the
gear phases and `angle` (0 .. 16383) is the 14-bit fine angle within the
turn.  The combined value is a 27-bit integer that counts continuously
across the turn boundary — it never jumps at the seam.

Conversions, on the host:

```
counts  = turns * 16384 + angle
turns   = counts / 16384
angle   = counts % 16384
degrees = counts * 360 / 16384
```

## Transactions (EEPROM-style)

- **Write** (1 byte): sets the register pointer (`0x00`..`0x06`; anything
  past the map clamps to it, reads then return 0x00).
- **Read**: serves bytes starting at the register pointer and
  auto-increments; once past 0x06 every byte reads 0x00 (no wrap).

Read the whole snapshot with a combined transaction:

```
write 0xA0, byte 0x00,    <- register pointer
(repeated start)
read  0xA1, 7 bytes       <- position (4) + status + seq (2)
```

Common patterns: 4 bytes = position only; 5 bytes = position + status;
7 bytes = the full snapshot.
Writing the pointer and reading it in two separate transactions works too.
A master that stops early (e.g. reads 1 byte after selecting 0x00) is
harmless — the slave re-arms at the end of the transaction.

## Behaviour notes

- Position bytes are always the latest sample.  The status valid bit
  clears when the slew guard rejects a sample (misread gear, see
  docs/architecture.md) or when any encoder read fails on that sample;
  the position bytes keep the last exported count — a host can decide
  whether to keep tracking on a stale reading.  Status bits 1..4 say
  which encoder(s) failed.
- The sample counter (`seq`) is the liveness signal: it ticks once per
  1 ms sample whether the sample was accepted or not.  A host that polls
  at least once per counter wrap (~65 s) can detect a stalled sample loop
  by watching it stop changing — the one thing that distinguishes "shaft
  at rest" from "sampling died".  A read that crosses a wrap simply sees
  the new (lower) count; no transaction ever tears, because the whole map
  is copied at address-match time.
- Before the first sample the map reads all zeros (including `seq`) and
  `status.valid = 0`.
- The address, once configured, survives CubeMX regeneration: the generated
  `i2c.c` keeps `OwnAddress1 = 0`; `hal_stm32.cpp` applies `PositionRegister::Address` at
  runtime during `hal.Init()`.

## Design decisions

- **Read-only map, no commands.** A write to the slave only ever selects the
  register pointer; there are no writable registers and no command codes.
  The device is deliberately a pure position source: zeroing, direction
  inversion and any other host-side math belong to the master.
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