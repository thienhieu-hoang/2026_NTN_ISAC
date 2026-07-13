Listed directory long_sequence_rayleigh_intraFading_globalLoop
Listed directory +ntn
Viewed SystemParams.m:1-92
Viewed MobilityModel.m:1-125
Viewed Geometry.m:1-73
Viewed ChannelModel.m:1-75
Viewed FadingSequence.m:1-94
Viewed coherence_and_resolution_analysis.md:1-130

To evaluate this, let's look at the mathematical parameters configured in your simulation files ([SystemParams.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/long_sequence_rayleigh_intraFading_globalLoop/+ntn/SystemParams.m), [Geometry.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/long_sequence_rayleigh_intraFading_globalLoop/+ntn/Geometry.m), and [FadingSequence.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/long_sequence_rayleigh_intraFading_globalLoop/+ntn/FadingSequence.m)).

---

### 1. Verification of the Current System Parameters

Using the code's configuration, here are the exact physical values:

#### A. Waveform & Block Durations
*   **Chip rate / Bandwidth ($B$):** $50\text{ MHz} \implies$ Chip duration $T_c = 20\text{ ns}$
*   **MLS Code length ($N_D$):** $127$ chips (order $m = 7$)
*   **Cyclic Prefix ($N_{\text{cp}}$):** $34$ chips
*   **Half-block duration ($T_{\text{PRI}}$):** $(127 + 34) \times 20\text{ ns} = 3.22\text{ }\mu\text{s}$
*   **Full-block duration ($T_{\text{block}}$):** $2 \times T_{\text{PRI}} = \mathbf{6.44\text{ }\mu\text{s}}$ (containing `[Sounding | Data]`)

#### B. Velocities & Doppler Shifts
*   **Relative Drone-UE Velocity ($v_{r,DU}$):** $-13.11\text{ m/s}$ (relative speed of $17\text{ m/s}$ projected on the Line-Of-Sight)
*   **Direct Path Doppler ($f_{D,DU}$):** $-(-13.11) / 0.0125\text{ m} \approx \mathbf{1049\text{ Hz}}$
*   **Target Monostatic Doppler ($\nu_{\text{mono}}$):** $\approx \mathbf{3745\text{ Hz}}$
*   **Target Scattered Doppler ($\nu_{DTU}$):** $\approx \mathbf{1463\text{ Hz}}$

#### C. Channel Coherence Time
Using Jakes' model for a 50% correlation threshold ($T_c \approx 0.423 / f_D$):
*   **Direct link ($D \to U$) coherence time:** 
    $$T_{c,DU} \approx \frac{0.423}{1049\text{ Hz}} \approx \mathbf{403\text{ }\mu\text{s}}$$
    *This is equivalent to $\approx \mathbf{62.5}$ block periods ($T_{\text{block}}$).*

---

### 2. Does the System Really Need Channel Prediction under Current Settings?

**No, not for the default $N_D = 127$ configuration.**
*   Under the current settings, a block is very short ($6.44\text{ }\mu\text{s}$) compared to the channel coherence time ($403\text{ }\mu\text{s}$).
*   The phase rotation of the channel from the sounding period to the data period (separated by $3.22\text{ }\mu\text{s}$) is:
    $$\Delta \phi = 2\pi \times f_{D,DU} \times T_{\text{PRI}} = 2\pi \times 1049\text{ Hz} \times 3.22\text{ }\mu\text{s} \approx \mathbf{1.22^\circ}$$
*   A phase drift of $1.22^\circ$ has an entirely negligible impact on BPSK demodulation (less than $0.001\text{ dB}$ SNR loss). Thus, one-shot sounding at every block is perfectly sufficient.

#### However, Channel Prediction is required if:
1.  **You use longer sequences to improve radar resolution:** As shown in [coherence_and_resolution_analysis.md](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/long_sequence_rayleigh_intraFading_globalLoop/coherence_and_resolution_analysis.md), to achieve a fine velocity resolution (e.g., $0.4\text{ m/s}$ to separate birds from drones), you need $N_D = 1512$. This increases $T_{\text{block}}$ to $60.5\text{ }\mu\text{s}$, driving the phase drift up to $22.9^\circ$ (or $87.1^\circ$ at higher velocities), causing communication failure without prediction.
2.  **You want to reduce sounding overhead (as proposed below).**

---

### 3. Proposing a Less Dense Sounding System (Reducing Sounding Overhead)

Your observation is **100% correct and represents a highly valuable research contribution**. 

Currently, every block is structured as `[Sounding | Data]`, which means **50% of the transmission time is wasted on sounding overhead**. Because the channel coherence time ($403\text{ }\mu\text{s}$) is 62 times longer than the block duration ($6.44\text{ }\mu\text{s}$), the physical fading coefficients ($g_{DU}$) change extremely slowly.

#### The Proposal: Sounding Every $K$ Blocks
Instead of sounding every block, we can sound only once every $K$ blocks (e.g., $K = 10$).
*   **Frame Structure:** `[Block 0: Sounding | Data]`, followed by `[Block 1: Data | Data]`, `[Block 2: Data | Data]`, ..., `[Block K-1: Data | Data]`.
*   **Overhead Reduction:** The sounding time overhead drops from **$50\%$** to **$5\%$** (for $K=10$). This immediately increases the communication throughput by **$90\%$**!

#### How Channel Prediction Makes This Possible (The Mathematics)
If you sound at Block $0$ and try to demodulate Block $9$ ($58\text{ }\mu\text{s}$ later) without channel prediction, the phase will have rotated by:
$$\Delta \phi_{\text{drift}} = 2\pi \times 1049\text{ Hz} \times 9 \times 6.44\text{ }\mu\text{s} \approx \mathbf{21.8^\circ}$$
At higher velocities (e.g., relative speed of $50\text{ m/s}$), this drift exceeds $80^\circ$, causing immediate demodulation failure.

With **Channel Prediction**, the UE uses the Doppler shift $\hat{f}_{D,DU}$ estimated at Block $0$ to rotate the channel estimate forward for any block $k \in [0, K-1]$:

$$\hat{h}_{DU}[m+k] = \hat{h}_{DU}[m] \cdot e^{j 2 \pi \hat{f}_{D,DU} \cdot k T_{\text{block}}}$$

Because the channel amplitude and Rayleigh fading coefficient ($g_{DU}$) are highly correlated over $K=10$ blocks ($10 \cdot T_{\text{block}} = 64.4\text{ }\mu\text{s} \ll T_{c,DU} = 403\text{ }\mu\text{s}$), this predicted channel matches the physical channel almost perfectly. This allows the UE to demodulate all $K$ blocks reliably while keeping the sounding overhead minimal.

---

# Assure the generated varying channel in simulation also have similar properties (similar coherence time)

### 1. Is the channel varying smoothly through time (blocks) in the code?
**Yes, the code successfully models a smoothly varying channel.**

The small-scale fading $g[m]$ is generated at a **slot-level** (half-block time steps of $T_{\text{PRI}}$) using a first-order Autoregressive model (AR(1)) in `+ntn/FadingSequence.m`:
$$g[m] = \alpha \cdot g[m-1] + \sqrt{1 - \alpha^2} \cdot w[m]$$

Where:
* The time-step between consecutive slots is $T_{\text{PRI}} = 3.22\text{ }\mu\text{s}$.
* The correlation parameter $\alpha$ is derived from Jakes' model at lag $T_{\text{PRI}}$: 
  $$\alpha = J_0(2\pi f_d T_{\text{PRI}})$$
* For your direct link ($D \to U$) with a relative Doppler of $f_d \approx 1049\text{ Hz}$ and $T_{\text{PRI}} = 3.22\text{ }\mu\text{s}$, the correlation parameter is:
  $$\alpha = J_0(2\pi \times 1049 \times 3.22 \times 10^{-6}) \approx 0.999888$$

Since $\alpha$ is extremely close to $1$, the current fading sample $g[m]$ is dominated by the previous sample $g[m-1]$, meaning the channel transitions very smoothly from one slot to the next.

---

### 2. Does it align with the analysis in `Parameters with channel.md`?
**Only for short lags (small number of blocks). Over long lags (like $K = 64$ blocks), there is a significant discrepancy.**

In `Parameters with channel.md`, the coherence time analysis is based on **Jakes' physical model** ($J_0$ Bessel autocorrelation). Let's compare how the correlation decays over time under the **AR(1) model in the code** versus the **physical Jakes' model** at different block intervals:

| Lag ($K$ blocks / $2K$ slots) | Physical Time ($\tau$) | AR(1) Correlation in Code ($\alpha^{2K}$) | Physical Jakes' Correlation ($J_0(2\pi f_d \tau)$) |
| :--- | :--- | :--- | :--- |
| **$K = 1$ block** (2 slots) | $6.44\text{ }\mu\text{s}$ | $\alpha^2 \approx \mathbf{0.99977}$ | $J_0(0.0424) \approx \mathbf{0.99955}$ |
| **$K = 10$ blocks** (20 slots) | $64.4\text{ }\mu\text{s}$ | $\alpha^{20} \approx \mathbf{0.9978}$ | $J_0(0.424) \approx \mathbf{0.955}$ |
| **$K = 64$ blocks** (128 slots) | $412.2\text{ }\mu\text{s}$ | $\alpha^{128} \approx \mathbf{0.985}$ (Highly Correlated) | $J_0(2.716) \approx \mathbf{-0.15}$ (Uncorrelated) |

#### Why does this happen?
The AR(1) model is a first-order Markov model, meaning its autocorrelation decays **exponentially** ($\rho(N) = \alpha^N$). Because $\alpha$ is so close to $1$, the exponential decay is extremely slow. 

In contrast, the true physical channel follows Jakes' autocorrelation ($J_0(2\pi f_d \tau)$), which decays much faster and eventually oscillates around zero. 

---

### 3. Implications for your Sparse Sounding Simulation
* **For small intervals (e.g., $K = 10$ blocks):** The AR(1) model in the code aligns very well with the analysis in `Parameters with channel.md` ($99.7\%$ vs $95.5\%$). The channel is highly correlated, and sparse sounding will work perfectly.
* **For larger intervals (e.g., $K = 64$ or $128$ blocks):**
  * **In the simulation:** Because the AR(1) model is artificially "too smooth" over long intervals ($\approx 98\%$ correlation at 64 blocks), the UE will still be able to demodulate data with very low BER, even if you only sound every 64 blocks.
  * **In a real physical channel:** Since the channel actually decorrelates completely after $64$ blocks (correlation drops to $-15\%$), a real system would **completely fail** (BER $\approx 0.5$) if you sound only every 64 blocks without applying a Doppler-based channel prediction/rotation.

### Recommendation
If you want to simulate the physical channel's decorrelation more accurately over large block lags, the channel generation should be updated from the simple AR(1) model to a filter-based Jakes' model (e.g., passing white Gaussian noise through a Doppler filter, or summing multiple sinusoidal paths with random phases). 

Under the current AR(1) model, your sparse sounding simulations will show optimistic (better than physical reality) BER results at large sounding intervals because the channel remains artificially correlated.