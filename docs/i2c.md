# I2C host port

The decoded absolute position (docs/architecture.md) is exported as an I2C
slave with a small register map, so any master on the bus can read the
multi-turn position on demand.  Nothing is pushed: the encoder answers reads
when addressed, like any encoder IC (AS5600-style).

## Bus facts

- Slave address: **0x50** (7-bit; `I2C_POS_ADDR` in `App/i2c_pos.h`).  On the
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

| reg  | name      | meaning                                                        |
|------|-----------|----------------------------------------------------------------|
| 0x00 | turns lo  | whole input-shaft turns, LSB                                  |
| 0x01 | turns hi  | whole input-shaft turns, MSB (0 .. 7428 = `GEAR_TURN_RANGE-1`) |
| 0x02 | angle lo  | fine angle within the turn, LSB (14-bit, 0 .. 16383)           |
| 0x03 | angle hi  | fine angle within the turn, MSB                                |
| 0x04 | status    | bit0 = 1: sample passed the slew guard (`valid`)               |

Position in counts: `turns * 16384 + angle`.  In degrees:
`counts * 360 / 16384`.

## Transactions (EEPROM-style)

- **Write** (1 byte): sets the register pointer (`0x00`..`0x04`; anything
  past the map clamps to it, reads then return 0x00).
- **Read**: serves bytes starting at the register pointer and
  auto-increments; once past 0x04 every byte reads 0x00 (no wrap).

Read the whole snapshot with a combined transaction:

```
write 0xA0, byte 0x00,    <- register pointer
(repeated start)
read  0xA1, 5 bytes       <- turns lo/hi, angle lo/hi, status
```

Common patterns: 4 bytes = position only; 5 bytes = position + status.
Writing the pointer and reading it in two separate transactions works too.
A master that stops early (e.g. reads 1 byte after selecting 0x00) is
harmless — the slave re-arms at the end of the transaction.

## Behaviour notes

- Position bytes are always the latest sample.  When the slew guard rejects
  a sample (misread gear, see docs/architecture.md), `status.valid` clears
  while the position bytes keep the last accepted turns — a host can decide
  whether to keep tracking on a stale reading.
- Before the first sample the map reads all zeros and `status.valid = 0`.
- The address, once configured, survives CubeMX regeneration: the generated
  `i2c.c` keeps `OwnAddress1 = 0`; `hal_stm32.c` applies `I2C_POS_ADDR` at
  runtime during `app_hal.init()`.

## Raising the speed to 400 kHz

Set `I2C1.SpeedClock` to 400000 in CubeMX and regenerate (`Core/Src/i2c.c`
is the only timing source).  The slave transport itself is speed-independent.