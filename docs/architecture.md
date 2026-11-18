### **Core Architecture**

* **Type:** Battery-free, true absolute mechanical encoder.
* **Operating Principle:** Coprime gear phase tracking (mechanical Vernier scale).

### **Mechanical Gear System**

* **Input Shaft (Sun Gear):** 13 teeth (Prime number).
* *Design Note:* Utilizes a positive profile shift to increase tooth root strength, prevent undercutting, and optimize assembly space. This gear is embedded with a magnet for individual position sensing.


* **Driven Gears:** 17, 19, and 23 teeth.
* *Design Note:* All tooth counts are prime and pairwise coprime. Each gear is embedded with a magnet for individual position sensing.



### **Sensor & Resolution Specifications**

* **Single-Turn Sensor:** MT6701 magnetic encoder.
* *Resolution:* 14-bit (16,384 steps per revolution, or $\approx 0.022^\circ$ per step).


* **Maximum Identifiable Turns:** 7,429 complete rotations.
* *Derivation:* The Least Common Multiple (LCM) of the driven gears ($17 \times 19 \times 23 = 7429$).


* **Multi-Turn Memory Allocation:** 13-bit data structure.
* *Capacity:* $2^{13} = 8192$ discrete values. This safely encapsulates the 7,429 mechanical states while providing an approximate 10% overflow safety margin.

### **Firmware Decoding** (`App/gear_decode.cpp`, config in `App/gear_config.h`)

*   **Encoder Map:** MT6701 #0 reads the input shaft (14-bit fine angle); MT6701 #1..#3 read the 17/19/23-tooth driven gears.
*   **Per-Sample Decode (CRT):** a driven gear turns with the *continuous* input position, so its scaled phase is $13\,(N+\theta)$ in tooth units, where $\theta$ is the within-turn angle the Sun encoder measures. The decoder removes that share first — $q_i = \mathrm{round}(t_i \cdot \mathrm{phase}_i - 13\,\theta_{\mathrm{sun}}) \bmod t_i$ — leaving the whole-turn residue $(13N) \bmod t_i$. The CRT over $(t_1, t_2, t_3)$ gives $Y = 13N \bmod 7429$; one modular multiply by $\mathrm{inv}(13 \bmod 7429)$ recovers $N$ in $0..7428$ (7,429 states). The Sun angle is therefore part of the turn solve, not just the fine-angle output. No counters, no storage — the position is absolute at any instant. The magnets' angular mounting offsets need not be zero: constant offsets shift the reported position as a whole (turns and fine angle together), so the assembly defines an arbitrary zero. Mapping that to a mechanical zero needs one stored offset (host-side; the firmware keeps no calibration), and an offset within sensor noise of a tooth-slot boundary can make the rounded tooth flicker, so production should check the magnets' seating.
*   **Fault Detection is Temporal:** within one sample a misread gear is indistinguishable from a genuine neighbouring state (every residue triple is a valid code word — the tooth slot is the information unit). What a misread always produces is a decoded position jump of a multiple of the other gears' product (≥ 323 turns), which no real shaft makes between samples. The slew guard (`GearMaxTurnsDelta`, default 1 turn per sample at 1 kHz ≈ 1000 turns/s) rejects such jumps: the sample is flagged invalid and tracking stays on the last accepted position (the register map carries the rejected decode, flagged by the valid bit).
*   **Limits:** the 13-bit field covers the 7,429 states with ~10% margin; past 7,429 turns the reading aliases (a mechanical end-stop is assumed). Sensor noise within half a tooth slot (~350–480 counts at 14-bit, in the Sun-de-rotated frame) cannot flip the decode.

### Notes

- Each gear's phase wraps around at a different, non-overlapping interval
- The driven-phase triple repeats every $7429/13 \approx 571.46$ input turns, because it tracks the continuous input position rather than only whole turns; the Sun's within-turn angle selects among the 13 repeats
- The four-sensor system therefore repeats only after exactly 7,429 turns — no sooner, no ambiguity