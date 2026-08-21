# I2C host port

The decoded absolute position (docs/architecture.md) is exported as an I2C
slave with a small register map, so any master on the bus can read the
multi-turn position on demand.  Nothing is pushed: the encoder answers reads
when addressed, like any encoder IC (AS5600-style).

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
| 0x04       | status | bit0 = 1: sample passed the slew guard (`valid`)               |

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

- **Write** (1 byte): sets the register pointer (`0x00`..`0x04`; anything
  past the map clamps to it, reads then return 0x00).
- **Read**: serves bytes starting at the register pointer and
  auto-increments; once past 0x04 every byte reads 0x00 (no wrap).

Read the whole snapshot with a combined transaction:

```
write 0xA0, byte 0x00,    <- register pointer
(repeated start)
read  0xA1, 5 bytes       <- position count (4 bytes LE) + status
```

Common patterns: 4 bytes = position only; 5 bytes = position + status.
Writing the pointer and reading it in two separate transactions works too.
A master that stops early (e.g. reads 1 byte after selecting 0x00) is
harmless — the slave re-arms at the end of the transaction.

## Behaviour notes

- Position bytes are always the latest sample.  When the slew guard rejects
  a sample (misread gear, see docs/architecture.md), `status.valid` clears
  while the position bytes keep the last accepted count — a host can decide
  whether to keep tracking on a stale reading.
- Before the first sample the map reads all zeros and `status.valid = 0`.
- The address, once configured, survives CubeMX regeneration: the generated
  `i2c.c` keeps `OwnAddress1 = 0`; `hal_stm32.cpp` applies `PositionRegister::Address` at
  runtime during `hal.Init()`.

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