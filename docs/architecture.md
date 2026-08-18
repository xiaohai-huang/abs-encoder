### **Core Architecture**

* **Type:** Battery-free, true absolute mechanical encoder.
* **Operating Principle:** Coprime gear phase tracking (mechanical Vernier scale).

### **Mechanical Gear System**

* **Input Shaft (Sun Gear):** 13 teeth (Prime number).
* *Design Note:* Utilizes a positive profile shift to increase tooth root strength, prevent undercutting, and optimize assembly space.


* **Driven Gears:** 17, 19, and 23 teeth.
* *Design Note:* All tooth counts are prime and pairwise coprime. Each gear is embedded with a magnet for individual position sensing.



### **Sensor & Resolution Specifications**

* **Single-Turn Sensor:** MT6701 magnetic encoder.
* *Resolution:* 14-bit (16,384 steps per revolution, or $\approx 0.022^\circ$ per step).


* **Maximum Identifiable Turns:** 7,429 complete rotations.
* *Derivation:* The Least Common Multiple (LCM) of the driven gears ($17 \times 19 \times 23 = 7429$).


* **Multi-Turn Memory Allocation:** 13-bit data structure.
* *Capacity:* $2^{13} = 8192$ discrete values. This safely encapsulates the 7,429 mechanical states while providing an approximate 10% overflow safety margin.