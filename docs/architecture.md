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

### **Firmware Decoding** (`App/gear_decode.c`, config in `App/gear_config.h`)

*   **Encoder Map:** MT6701 #0 reads the input shaft (14-bit fine angle); MT6701 #1..#3 read the 17/19/23-tooth driven gears.
*   **Per-Sample Decode (CRT):** nearest tooth slot per gear $q_i = \mathrm{round}(t_i \cdot \mathrm{phase}_i)$, residue $r_i = q_i \cdot \mathrm{inv}(13 \bmod t_i) \bmod t_i$, turn count $N$ from the Chinese remainder theorem over $(t_1, t_2, t_3)$ → $0..7429$. No counters, no storage — the position is absolute at any instant.
*   **Fault Detection is Temporal:** within one sample a misread gear is indistinguishable from a genuine neighbouring state (every residue triple is a valid code word — the tooth slot is the information unit). What a misread always produces is a decoded position jump of a multiple of the other gears' product (≥ 323 turns), which no real shaft makes between samples. The slew guard (`GEAR_MAX_TURNS_DELTA`, default 2 turns per sample at 100 Hz ≈ 200 turns/s) rejects such jumps: the last accepted position is held and the sample flagged invalid.
*   **Limits:** the 13-bit field covers the 7,429 states with ~10% margin; past 7,429 turns the reading aliases (a mechanical end-stop is assumed). Sensor noise within half a tooth slot (~350–480 counts at 14-bit) cannot flip the decode.

### Notes

- Each gear's phase wraps around at a different, non-overlapping interval
- The combination of three phases creates a unique fingerprint for every single sun rotation from 0 to 7,428
- The system only repeats after exactly 7,429 turns — no sooner, no ambiguity