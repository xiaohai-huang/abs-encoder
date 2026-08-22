# PCB design guide

The hardware-side contract for the encoder board.  Every pin, level and
component value below is derived from the firmware sources of truth — the
`.ioc` (pin/peripheral config), the wiring table in `App/hal_stm32.cpp`
(`_encoderWiring`) and `docs/MT6701.md` (SSI timing, magnet spec).  If this
document and the firmware ever disagree, the firmware wins; fix this
document.

> What the board is: one STM32F103C8T6 reading four MT6701 magnetic
> encoders over two SPI buses and exporting the decoded multi-turn
> position as an I2C **slave** at address 0x50.  Battery-free: no coin
> cell, no backup rail, no persistent storage — the gears are the memory,
> so power may be cut at any time with no data loss.

## 1. What the firmware expects of the hardware

1. **MCU**: STM32F103C8T6, LQFP48, running 72 MHz from an **8 MHz HSE
   crystal** (PLL ×9).  The board must provide the crystal — the clock
   tree is not optional.
2. **Four MT6701** wired exactly per the table below.  The role ↔ CSN ↔
   bus assignment is baked into `hal_stm32.cpp`; a board that swaps two
   sensors decodes garbage that looks plausible.  **Note the
   non-monotonic mapping: the Sun encoder is CSN4, not CSN1.**
3. **I2C slave port on PB6/PB7**, open-drain, **external pull-ups
   mandatory** — the firmware does not enable the STM32's internal pulls.
4. **CSN pins idle HIGH** (the `.ioc` initializes them high and
   `hal.Init()` re-raises them).  During MCU reset the pins are
   high-impedance, so add external pull-ups (§4).
5. **Single 3.3 V rail, regulated onboard from the host's 5 V** (§2), no backup supply of any kind.
6. **SWD** (PA13/PA14) is the only flash/debug path — no UART or
   bootloader is used in normal operation.

## 2. Power supply

### Rail architecture — decided: 5 V from the host, local 3.3 V LDO

The host board runs from a 5 V input rail with its own onboard 3.3 V
LDO.  The host cable carries **5 V + GND + SDA + SCL**, and this board
regulates its own 3.3 V locally.  That beats tapping the host's 3.3 V
node on three counts:

- **Don't spend someone else's thermal budget or stretch their
  regulated node.**  The host's LDO was sized for the host board; a
  ~100 mA remote load adds ~170 mW to it and extends the sensitive
  3.3 V node out of its enclosure onto a cable, where picked-up noise
  lands straight on the host MCU's supply.
- **5 V over the cable absorbs drop and re-opens series protection.**
  Cable drop is irrelevant with LDO headroom, and a reverse-polarity
  Schottky (~0.3 V) still leaves ~4.7 V — comfortably above 3.3 V +
  dropout (unlike a 3.3 V feed, where a Schottky would starve the
  MT6701).
- **A clean local rail**: LDO PSRR strips host-rail noise before it
  reaches the sensor supply.

**LDO choice**: fixed 3.3 V, **dropout ≤ 0.5 V** so a worst-case ~4.5 V
at the connector still regulates (this rules out 1117-class parts at
~1.1 V dropout), rated ≥ 300 mA, ceramics-stable.  Thermal:
(5 − 3.3) V × 150 mA max ≈ 260 mW — SOT-223 handles it easily;
SOT-23-5 only with generous copper and a measured real load.

**Input front end where the cable lands**: reverse-polarity Schottky
(or PMOS ideal-diode), SMAJ5.0A-class TVS, ferrite bead, 10–47 µF bulk.
A deep dip past POR merely reboots the encoder — position is re-derived
absolutely, so hold-up buys availability, not data.

Check the host's 5 V source for +100 mA of headroom (trivial unless it
is a USB 500 mA budget already near its limit).

Both MCUs stay in 3.3 V domains.  The I2C bus is pulled up on the host
side, to the host's 3V3 (§6); both ends' I2C pins are 5 V-tolerant
(FT), so the small rail difference between the two LDOs is a non-issue.

- **GND quality**: the cable's GND pair must return to the host's
  controller GND, not ride a motor-driver or other high-current return
  — I2C noise margin erodes directly with ground offset.

Current **budget** (design the rail for ~150 mA with margin; verify
against the datasheets at final BOM):

| Load | Budget |
| :--- | :--- |
| STM32F103C8 @ 72 MHz | ~50 mA |
| 4 × MT6701 | ~10 mA each ≈ 40 mA |
| Pull-ups, power LED, margin | ~60 mA |

### Decoupling

| Location | Capacitors |
| :--- | :--- |
| Each VDD/VSS pair (3 pairs: pins 24, 36, 48) | 100 nF |
| Bulk, near MCU | 4.7–10 µF ceramic |
| VDDA/VSSA (pins 9/8) | 1 µF + 100 nF (a ferrite bead from 3V3 is good practice; direct tie acceptable — no ADC is used) |
| Each MT6701 VDD pin | 100 nF, as close to the pin as possible |
| NRST | 100 nF to GND |

### Battery-free consequences

- **VBAT (pin 1) ties to 3V3.**  No coin cell, no diode ORing.
- No hold-up capacitors are needed: a power cut simply stops the sample
  loop; position is re-derived absolutely on the next power-up.
- No brown-out special handling required (the F103's POR/PDR covers the
  3.3 V ramp).

## 3. Pin map / netlist (LQFP48)

Verify every row against the datasheet pinout table when you capture or
import the symbol — typos here are the #1 cause of a dead first board.

| Pin | # | Net | Role |
| :--- | :--- | :--- | :--- |
| PA3 | 13 | `CSN1` | Encoder CS, active low — **Gear1 (17 T, driven)**, SPI1 |
| PA4 | 14 | `CSN2` | Encoder CS, active low — **Gear2 (19 T, driven)**, SPI1 |
| PA5 | 15 | `SPI1_SCK` | Bus 1 clock → MT6701 pin B (pin 7) of Gear1+Gear2 |
| PA6 | 16 | `SPI1_MISO` | Bus 1 data ← MT6701 pin A (pin 6) of Gear1+Gear2 |
| PA7 | 17 | `SPI1_MOSI` | Configured in firmware, **not connected to the chips** (SSI is read-only; dummy bytes are sent). Route nowhere, or leave the net MCU-local. |
| PA8 | 29 | `CSN4` | Encoder CS, active low — **Sun (13 T, input shaft)**, SPI2 ⚠️ not Gear4 |
| PA9 | 30 | `CSN3` | Encoder CS, active low — **Gear3 (23 T, driven)**, SPI2 |
| PB13 | 26 | `SPI2_SCK` | Bus 2 clock → Sun + Gear3 pin B |
| PB14 | 27 | `SPI2_MISO` | Bus 2 data ← Sun + Gear3 pin A |
| PB15 | 28 | `SPI2_MOSI` | Unused, as PA7 |
| PB6 | 42 | `I2C1_SCL` | Open-drain, **external pull-up required** |
| PB7 | 43 | `I2C1_SDA` | Open-drain, **external pull-up required** |
| PA13 | 34 | `SWDIO` | Debug/flash (SWD) |
| PA14 | 37 | `SWCLK` | Debug/flash (SWD) |
| PD0 | 5 | `OSC_IN` | 8 MHz crystal |
| PD1 | 6 | `OSC_OUT` | 8 MHz crystal |
| NRST | 7 | `nRST` | 100 nF to GND; bring to the debug header |
| BOOT0 | 44 | `BOOT0` | 100 kΩ pull-down to GND; optionally to a jumper (§7) |
| BOOT1/PB2 | 20 | — | Leave unconnected (internal pull-down at reset) or 100 kΩ to GND |
| VBAT | 1 | `3V3` | **Tie to 3V3** — battery-free design |
| VDD/VSS | 24/23, 36/35, 48/47 | `3V3`/`GND` | One 100 nF per pair + bulk |
| VDDA/VSSA | 9/8 | `3V3`/`GND` | See §2 decoupling |

Free pins (route to test points or leave for expansion — using them needs
`.ioc` + firmware changes): PA0, PA1, PA2, PA10, PA11, PA12, PA15, PB0,
PB1, PB3, PB4, PB5, PB8, PB9, PB10, PB11, PC13, PC14, PC15.  Caution:
PA15/PB3/PB4 default to the JTAG function at reset; they are only usable
as GPIO after the firmware disables JTAG (SWD-only).  A power LED on PC13
is a nice touch but requires a small firmware change — the current build
never drives it.

## 4. MT6701 sensor circuit (×4)

Per-chip connections (SOP-8; QFN-16 equivalents in `docs/MT6701.md`):

| MT6701 pin | Net |
| :--- | :--- |
| 1 VDD | `3V3` + 100 nF at the pin |
| 2 MODE | `3V3` (selects SSI; the internal 200 kΩ pull-up makes floating legal, but tie it) |
| 3 OUT | NC |
| 4 GND | `GND` |
| 5 PUSH | NC |
| 6 A (DO) | that bus's `MISO` |
| 7 B (SCK) | that bus's `SCK` |
| 8 Z (CSN) | that encoder's own `CSNx` + **10 kΩ pull-up to 3V3** |

The CSN pull-ups are defensive: F1 GPIOs are high-impedance from reset
until `hal.Init()` runs, and an encoder whose CSN floats low during that
window sees clock noise on SCK.  With pull-ups the encoders stay
deselected until the firmware takes over.

Bus topology per `docs/MT6701.md`: SCK and MISO are shared by the two
encoders of a bus; only one CSN is ever low at a time (firmware-guaranteed
— two selected slaves would both drive MISO).

## 5. Placement: the gearbox dictates the board

The four MT6701 positions are **not free layout choices** — each die
center must sit under its gear's rotation axis, magnet concentric within
~0.25 mm (eccentricity shows up directly as angle ripple).

1. Export the four gear-axis coordinates (and the mounting-hole pattern)
   from the gearbox CAD; place the sensor bodies there first, everything
   else second.  The board outline follows the gearbox envelope.
2. Silkscreen a **crosshair on each sensor's die center** so the magnet
   can be aligned during mechanical assembly, and label each sensor with
   its firmware identity — this is the assembly mistake you are defending
   against:

   | Sensor position | Silkscreen |
   | :--- | :--- |
   | Sun gear axis | `SUN / CSN4 / 13T` |
   | 17 T gear axis | `G1 / CSN1 / 17T` |
   | 19 T gear axis | `G2 / CSN2 / 19T` |
   | 23 T gear axis | `G3 / CSN3 / 23T` |

3. Magnet per encoder: **Ø6 mm × 2.5 mm, diametrically magnetized**, on
   the gear/shaft end face, air gap 0.5–2 mm above the die.  Put the
   magnets in the BOM — they are as much a part of the "PCB" as anything
   soldered to it.  The MT6701 status field reports field too strong / too
   weak, which the firmware turns into read failures — final gap setting
   is verifiable over I2C (§10).
4. Fasteners and standoffs near a sensor must be non-magnetic (brass,
   nylon, non-magnetic stainless).  Copper planes under the die are fine.
5. ≥ 3 non-collinear mounting holes (M3 ⇒ Ø3.2 mm) matched to the gearbox
   bosses; keep them and the host connector away from the sense points.
6. The MCU/crystal/regulator cluster goes wherever convenient — the
   signals to the sensors are slow enough that distance is tolerable
   (§6).

## 6. Signal integrity & routing

The whole board is slow by PCB standards: SPI at **4.5 Mbit/s**, I2C at
100 kHz, one 72 MHz MCU.  A clean 2-layer board with an unbroken bottom
GND pour is fully adequate — no impedance control, no length matching.

- **Stack**: 2 layers, 1 oz, 1.6 mm; bottom = solid GND pour (only vias
  cut it), top = components/routing with GND fill; stitch the pours.
- **SPI** (PA5/PA6, PB13/PB14 + CSNs): keep each bus routed over the GND
  pour; if the sensor spread makes runs long (> ~10 cm) or stubby, fit
  **22–33 Ω series footprints** (0 Ω fitted by default) near the MCU on
  SCK and near each slave on MISO for damping.  Route each bus as a short
  star to its two sensors; keep stubs similar in length.  CSN traces are
  unconstrained.
- **I2C** (PB6/PB7): route as a pair to the connector.  The pull-ups
  live on the **host** PCB, to the host's 3V3 — see below; this board
  keeps 2 × 4.7 kΩ footprints **unpopulated (DNP)**, to be fitted only
  for standalone bench tests against a dongle that has no pull-ups of
  its own.

  Why host-side: pull-ups belong to the bus owner.  With them on the
  host, an unplugged or unpowered encoder is a clean NACK and any other
  device on the host bus keeps working; with them here, pulling this
  cable would drag SDA/SCL dead for the whole bus.  Level-wise either
  placement is safe — both ends' I2C pins are 5 V-tolerant (FT), and
  the bus HIGH then simply references the host's rail.

  | Bus speed | Pull-up (short bus) |
  | :--- | :--- |
  | 100 kHz (current firmware) | 4.7 kΩ |
  | 400 kHz (see `docs/i2c.md` §Raising the speed) | 2.2 kΩ |

  Keep total bus capacitance < 400 pF (matters if the host cable is long).
- **Crystal**: 8 MHz, CL 10–20 pF (any standard tolerance — no USB, no
  RTC; ±50 ppm is plenty), placed close to PD0/PD1 with the load caps
  (C = 2·(CL − C_stray), C_stray ≈ 4 pF ⇒ typically 15–22 pF each) and a
  local GND guard; route nothing fast under or beside it.
- Keep the SPI/I2C routing out of the crystal keep-out and away from the
  sensor sense areas.

## 7. Connectors, debug, and stray pins

**Host connector** (I2C + power), suggest a keyed 5-pin 2.54 mm on the
board edge:

| Pin | Signal |
| :--- | :--- |
| 1 | `5V` — the host's 5 V rail, regulated to 3V3 onboard (§2) |
| 2 | GND |
| 3 | SDA |
| 4 | SCL |
| 5 | GND / shield |

Two GND pins make the cable reverse-proof with a keyed housing.  Since
this port faces the outside world, a low-capacitance dual TVS array on
SDA/SCL — plus the 5 V TVS from the §2 front end — is cheap insurance
for production; optional on a prototype.

⚠️ **Address 0x50** is the common AT24Cxx EEPROM address — if the host bus
carries an EEPROM, it collides.  The address is `PositionRegister::Address`
in `App/i2c_pos.h`; changing it is a one-line firmware edit, so decide the
bus population before ordering.

**SWD header**, 5-pin 2.54 mm (this is the only way to flash — no CLI
flash script exists, cortex-debug + STLink only): `3V3 · SWDIO · GND ·
SWCLK · NRST`.  Bring NRST even though SWD rarely needs it — it rescues
boards that sleep or misbehave.

**BOOT0**: pull down 100 kΩ.  A 2-pin jumper to 3V3 selects the system
bootloader as a last-resort recovery path — note the F103 ROM bootloader
speaks USART1 on PA9/PA10, and **PA9 is CSN3**; boot-loader traffic will
harmlessly clock the Gear3 encoder.  Recovery-only; not part of normal
operation.

**Recommended test points**: `3V3`, `GND`, and one per CSN (4) — with a
scope, the CSN pulse order instantly verifies the role wiring without a
single code change.

## 8. BOM sketch

| Ref | Part | Notes |
| :--- | :--- | :--- |
| U1 | STM32F103C8T6, LQFP48 | |
| U2–U5 | MT6701 (SOP-8 or QFN-16) | pin map §4 |
| U6 | LDO 3.3 V, dropout ≤ 0.5 V, ≥ 300 mA, SOT-223 | §2 |
| — | input front end: reverse Schottky + 5 V TVS + ferrite + 10–47 µF bulk | §2 |
| Y1 | 8 MHz crystal, CL 10–20 pF | HC-49S-SMD or 3225 |
| C | 100 nF ×(3 VDD + 1 VDDA + 4 MT6701 + 1 NRST), 1 µF + 4.7–10 µF bulk | |
| C | crystal load caps ×2 (15–22 pF per §6) | |
| R | I2C pull-up footprints 2 × 4.7 kΩ — **DNP** (the fitted pair lives on the host PCB, to host 3V3) | §6 |
| R | CSN pull-ups 4 × 10 kΩ | §4 |
| R | BOOT0 100 kΩ | |
| R (opt.) | series damping on SCK/MISO | §6 |
| D | power LED + 1 kΩ on 3V3 | always-on, speeds bring-up |
| TVS (opt.) | dual low-cap array on SDA/SCL | §7 |
| Mech | 4 × diametric magnet Ø6 × 2.5 mm; M3 hardware | §5 — order with the board! |

## 9. Fab & DFM

- 2 layers / 1 oz / 1.6 mm, min 6/6 mil trace/space, via ≥ 0.3/0.7 mm —
  any budget fab (JLCPCB-class) handles this; no controlled impedance, no
  special stack.
- Surface finish: ENIG if using the QFN-16 sensor or if the board will be
  reworked; HASL acceptable for a first SOP-8 prototype.
- Run the fab's DRC plus your own: no plane-to-plane clearance violations
  under the sensors, silkscreen not on pads, connector pins numbered,
  polarity marked on the host connector.
- Export: Gerber + drill + netlist (IPC-D-356 if supported); order the
  stencil with 0.1 mm foil for the QFN/MSOP pitch if hand-assembling.

## 10. Bring-up & validation

The I2C register map is the board's built-in test bench
(`docs/i2c.md`, troubleshooting in `docs/i2c-host-guide.md`):

1. **Bare board**: continuity on rails, no shorts 3V3↔GND, before any
   chip is placed.
2. **Power**: 3.3 V in spec; current in the tens of mA; power LED on.
3. **Flash via SWD** — works with zero sensors attached.  Over I2C you
   should then read `status = 0x1E` (bits 1..4 = Sun/Gear1/Gear2/Gear3 all
   failing) with `seq` ticking ~1000/s: that single read proves the MCU,
   crystal, firmware, I2C port and the sample loop are all alive.
4. **Per-sensor debug**: the status bits **name the failing encoder**
   (bit1 Sun, bit2 Gear1, bit3 Gear2, bit4 Gear3) — a stuck bit points at
   one chip's soldering, wiring or magnet.  A chip that reads but CRC-fails
   every frame returns 0xFF-bytes (dead chip, missing 3.3 V, floating
   CSN) — see `docs/i2c-host-guide.md` for the full decode table.
   Remember the CRC-6 acceptance on real hardware is binary
   (`docs/MT6701.md`): every frame validates, or none do; if frames are
   garbage, the fallback experiment is SPI mode 2 — a CubeMX-only change.
5. **Magnet gap**: a too-strong/too-weak field shows up as that encoder's
   read failing; adjust gap until its status bit stays clear while
   spinning.
6. **System check**: rotate the input shaft slowly through several turns —
   `pos` must move monotonically (counts = turns·16384 + angle, never
   tearing at the seam) with `status = 0x01`; a deliberately fast flick
   should exercise the slew guard (`status = 0x00` briefly, position
   held).  Power-cycle at random angles: `pos` returns to the same count —
   the whole point of the design.

## 11. Final review checklist

- [ ] Sensor die centers at the four gear axes; crosshairs + role/CSN/teeth silkscreen at each (`SUN/CSN4/13T` — the Sun trap)
- [ ] CSN1→G1, CSN2→G2, CSN3→G3, CSN4→Sun; per-bus SCK/MISO shared correctly
- [ ] 10 kΩ on every CSN; 100 nF at every MT6701 VDD; MODE tied high; OUT/PUSH floating
- [ ] MOSI not routed to the encoders
- [ ] I2C pull-ups fitted on the host PCB (to host 3V3); this board's pull-up footprints left DNP; no 0x50 conflict on the host bus
- [ ] Host 5 V budget verified for the extra ~100 mA; connector front end (reverse Schottky + TVS + ferrite + bulk) fitted
- [ ] VBAT → 3V3; 3 × VDD decoupling + bulk; VDDA decoupled
- [ ] 8 MHz crystal + correct load caps; keep-out respected
- [ ] NRST cap; BOOT0 pull-down (+ optional jumper); SWD header with NRST
- [ ] Bottom GND pour unbroken under all buses
- [ ] Test points: 3V3, GND, 4 × CSN
- [ ] Magnets + non-magnetic fasteners in the BOM
