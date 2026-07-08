# Delay Synchronization & Inter-Period Interference in PMCW NTN-ISAC
## Presentation Slides

---

# Slide 1: Title
## Delay Synchronization & Inter-Period Interference in PMCW NTN-ISAC
*   **Topic:** Impact of propagation delay on PMCW joint communication-sensing waveforms.
*   **Focus:** Analyzing the transition from transient single-block errors to steady-state continuous block success.
*   **Presenter:** [Your Name]

---

# Slide 2: The Core Waveform Structure
*   **PMCW Frame (Single Block):**
    $$\text{[ Block } m \text{ ]} = \big[ \text{Sounding Period } (T_{\text{PRI}}) \;\big|\; \text{Data Period } (T_{\text{comm}}) \big]$$
*   **Sounding Half:** Unmodulated Maximum-Length Sequence (MLS) with BPSK symbol $d_m = +1$ (known pilot for channel estimation).
*   **Data Half:** MLS modulated by random data symbol $d_m \in \{+1, -1\}$.
*   **Waveform Constraint:** Both periods use the same MLS code (single-code design), meaning they are not separable in the code domain.

---

# Slide 3: Scenario 1: Single Block (No Guard) — WHY IT IS BAD
### The Setup
*   A single block is transmitted in isolation.
*   The signal experiences propagation delay $\ell_{DU}$ chips.
*   No guard band is used between sounding and data.

### Why it is BAD (The Result)
*   **Transient Leakage:** The first $\ell_{DU}$ chips of the sounding period are missing (received as zeros/noise).
*   **Inter-Period Interference (IPI):** Sounding chips leak into the data period, and data chips leak into the next (empty) period.
*   **Demodulation Failure:** The channel estimate $\hat{h}_{DU}$ is corrupted, and equalization fails.
*   **BER:** Approaches **$0.5$** (equivalent to random guessing), even under zero noise!

---

# Slide 4: Scenario 2: Single Block (With Guard Band) — WHY IT IS GOOD
### The Setup
*   A guard band (zero-padding or cyclic prefix) of duration $T_g \ge \tau_{DU}$ is inserted between periods:
    $$\text{[ Sounding ]} \rightarrow \text{[ Guard Band ]} \rightarrow \text{[ Data ]} \rightarrow \text{[ Guard Band ]}$$

### Why it is GOOD
*   **Perfect Separation:** The guard band absorbs the propagation delay. 
*   **Clean Windows:** Sounding and data windows do not overlap at the receiver.
*   **Optimal Performance:** Correlation peaks are uncorrupted, and the BER matches the theoretical BPSK limit.

### The Catch
*   **Spectral Efficiency Loss:** Guard bands introduce time overhead, reducing data throughput.

---

# Slide 5: Scenario 3: Continuous Block Sequence (No Guard) — THE SMART WORKAROUND
### The Setup
*   A continuous stream of blocks (e.g., $M_{\text{seq}} = 500$) is transmitted back-to-back without guard bands.
*   The receiver aligns the entire continuous stream by shifting it back by $-\ell_{DU}$ chips.

### Why the Overall BER is GOOD
*   **Steady-State Success:** For all blocks $m \ge 2$, the tail of the previous block perfectly fills the delay gap. No zeros or boundary corruptions occur.
*   **Transient Startup Block:** Only the very first block (Block 1) suffers from the startup transient (missing preceding signal).
*   **Minimal Overhead:** The single corrupted block represents only $1 / 500 = \mathbf{0.2\%}$ of the total transmission.
*   **Overall BER:** Remains extremely close to the clean BPSK theoretical limit.

---

# Slide 6: Summary Comparison Table

| Metric / Scenario | Single Block (No Guard) | Single Block (With Guard) | Continuous Sequence (No Guard) |
| :--- | :--- | :--- | :--- |
| **Sounding Quality** | Corrupted (zeros/noise) | Perfect | Block 1: Corrupted <br> Blocks 2+: Perfect |
| **Data Quality** | Corrupted (IPI) | Perfect | Block 1: Corrupted <br> Blocks 2+: Perfect |
| **Throughput (Efficiency)**| High (theoretical) | **Low** (due to guard overhead) | **High** (steady-state efficiency) |
| **Overall BER** | **Failure** (BER $\approx 0.5$) | **Optimal** (theoretical BPSK) | **Optimal** (Block 1 transient is negligible) |

---

# Slide 7: Takeaway
*   **For short/single-frame testing:** Guard bands are mandatory to prevent synchronization failure.
*   **For continuous systems (NTN-ISAC):** Continuous block sequences naturally resolve the delay synchronization issue in steady-state, achieving high spectral efficiency and optimal BER without guard band overhead.
