# Delay Synchronization, Inter-Period Leakage, and Steady-State BER in PMCW NTN-ISAC

This document explains the delay synchronization problem in Phase-Modulated Continuous Wave (PMCW) systems, focusing on how propagation delays cause Inter-Symbol/Inter-Period Interference (ISI/IPI), how guard bands solve this, and why a long continuous sequence of blocks exhibits transient startup errors followed by clean steady-state performance.

---

## 1. The Block Structure and the Delay Problem

In this PMCW system, a single transmit block of duration $T_{\text{block}}$ is divided into two contiguous halves:
$$\text{[ Block } m \text{ ]} = \big[ \text{ Sounding Period } (T_{\text{PRI}}) \;\big|\; \text{ Data Period } (T_{\text{comm}}) \big]$$

Both periods transmit the same Maximum-Length Sequence (MLS) of length $N_D$ chips, but the sounding period is unmodulated ($d_m = +1$) while the data period carries the BPSK information symbol ($d_m = \pm 1$).

### The Inter-Period Leakage (ISI/IPI)
When this signal propagates over the wireless channel, it experiences a delay of $\ell_{DU}$ chips. 

If we simulate a **single block in isolation** (with no preceding or succeeding transmissions), the received signal is linearly shifted. At the receiver, the block is partitioned into two windows of size $N_D$ starting at the nominal boundaries:

```
Transmit:  |-- Sounding (d=1) --|-- Data (d=±1) --|
Received:       |-- Sounding (d=1) --|-- Data (d=±1) --|
                [  Window 1 (S)  ]   [  Window 2 (D)  ]
```

As a result:
1.  **Sounding Window (S):** The first $\ell_{DU}$ chips of the sounding window contain **zeros** (or receiver noise) because the signal hasn't arrived yet. The remaining $N_D - \ell_{DU}$ chips contain the sounding sequence.
2.  **Data Window (D):** The first $\ell_{DU}$ chips of the data window are contaminated by the tail of the sounding period. The remaining $N_D - \ell_{DU}$ chips contain the data sequence.

When the receiver correlates Window 1 with the local MLS replica to estimate the channel:
*   The missing $\ell_{DU}$ chips at the beginning degrade the correlation peak power from $N_D^2$ to $(N_D - \ell_{DU})^2$.
*   For large delays, this severely corrupts the channel estimate $\hat{h}_{DU}$.
*   When this corrupted estimate is used to equalize the data window (which is itself contaminated by sounding chips), BPSK demodulation fails, leading to an extremely high Bit Error Rate (BER $\approx 0.5$) even at infinite SNR.

---

## 2. Simulation A: The Guard Band (Perfect Separation) (clean_syncronization)

To prevent this overlap, the transmitter can insert a **guard band** (either zero-padding or a cyclic prefix) of duration $T_g \ge \tau_{DU}$ between the sounding and data periods:

$$\text{[ Sounding ]} \rightarrow \text{[ Guard Band ]} \rightarrow \text{[ Data ]} \rightarrow \text{[ Guard Band ]}$$

*   **How it works:** The guard band absorbs the propagation delay. Even when the received signal is shifted, the sounding and data periods do not overlap.
*   **Result:** The receiver can window out the clean, non-overlapping portions of the sounding and data periods. The correlation peak remains perfect, and the BER drops to the theoretical BPSK limit.
*   **Drawback:** Inserting guard bands increases the block length and reduces spectral efficiency (throughput).

---

## 3. Simulation B: Continuous Block Sequences (Steady-State Transmission) (long_sequence_gaussian)

Instead of using guard bands, your simulation implements a **continuous block sequence** ($M_{\text{seq}} = 500$ blocks transmitted back-to-back):

$$\dots \text{[ Sounding } m-1 \;\big|\; \text{ Data } m-1 \text{ ]} \text{ [ Sounding } m \;\big|\; \text{ Data } m \text{ ]} \text{ [ Sounding } m+1 \;\big|\; \text{ Data } m+1 \text{ ]} \dots$$

When this continuous stream is received with a delay of $\ell_{DU}$ chips:
*   The first $\ell_{DU}$ chips of the sounding period of block $m$ are filled with the tail of the data period of block $m-1$.
*   The first $\ell_{DU}$ chips of the data period of block $m$ are filled with the tail of the sounding period of block $m$.

Because the blocks are contiguous and the MLS sequence has a periodic correlation property, this delay acts as a shift on a continuous stream. 

At the receiver, the frame synchronizer compensates for this delay by **circularly shifting the entire received continuous chip stream back by $-\ell_{DU}$ chips** before cutting it into blocks:

```matlab
yU_aligned = circshift(yU_full_flat, [-g.ell_DU, 0]);
obj.yU_full = reshape(yU_aligned, [2*p.ND, p.M]);
```

### Why only the first block is corrupted (Transient vs. Steady-State)

#### The First Block (Transient Startup):
At the very start of the transmission ($t = 0$), there is no "block 0" before block 1. 
*   Therefore, the first $\ell_{DU}$ chips of the sounding period of block 1 are **zeros** (in a real linear channel) or wrapped-around noise from the end of the 500-block sequence (due to `circshift` modeling in simulation).
*   Because of this boundary mismatch, the channel estimate for block 1 is corrupted, and the BPSK symbol decision for block 1 is highly likely to be wrong.
*   This is the **startup transient error** (`BER_block1` in your code).

#### Blocks 2 to $M_{\text{seq}}$ (Clean Steady-State):
For any block $m > 1$:
*   The channel has reached a steady-state. The sounding period of block $m$ is preceded by the data period of block $m-1$. 
*   When the continuous stream is shifted back by $-\ell_{DU}$, the chips are perfectly aligned within the receiver's window boundaries for all blocks $m \ge 2$.
*   No zeros or boundary corruptions occur.
*   Thus, blocks 2 to $M_{\text{seq}}$ are **perfectly clean** and achieve the optimal BPSK error rate (`BER_sim` in your code).

### Impact on Overall BER
Since only the first block out of 500 is corrupted, the overall average BER (`BER_all`) is:
$$\text{BER}_{\text{all}} = \frac{1}{M_{\text{seq}}} \text{BER}_{\text{block1}} + \frac{M_{\text{seq}} - 1}{M_{\text{seq}}} \text{BER}_{\text{sim}}$$

For $M_{\text{seq}} = 500$:
$$\text{BER}_{\text{all}} = \frac{1}{500} (0.5) + \frac{499}{500} \text{BER}_{\text{sim}} \approx 0.001 + 0.998 \cdot \text{BER}_{\text{sim}}$$

Because the transient corrupted block represents only **$0.2\%$** of the total transmission, the overall average BER remains extremely close to the clean steady-state BER. This explains why the system performs well in continuous transmission without requiring guard bands.

## Visualization

![](../../Z_backup/long_sequence_gaussian/geometry.png)
(First, check the Theory and Simulation lines with only 2 nodes first, to see if the analysis for the Perfect CSI and estimated CSI correct)
![](../../Z_backup/clean_syncronization/ue_ber_perfect.png)

### 1. Single block at every Monte-Carlo trial - no guard band used

![](../../Z_backup/long_sequence_gaussian/range_doppler_1.png)

![](../../Z_backup/long_sequence_gaussian/ue_ber_1.png)

### 2. Single block at every Monte-Carlo trial - assuming perfect guard band

![](../../Z_backup/clean_syncronization/range_doppler.png)

![](../../Z_backup/clean_syncronization/ue_ber.png)


### 3. Long sequence of  blocks at every Monte-Carlo trial

![](../../Z_backup/long_sequence_gaussian/range_doppler_long.png)

![](../../Z_backup/long_sequence_gaussian/ue_ber_long.png)
