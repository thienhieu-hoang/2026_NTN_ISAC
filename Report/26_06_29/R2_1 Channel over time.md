# Understanding the Wireless Channel Model and Temporal Evolution

In a wireless communication and radar system, the propagation path between any two nodes (e.g., Drone to UE, Drone to Target) is represented by a time-varying complex channel coefficient $h[m]$, where $m$ is the block index.

This document explains the physical formula of the channel, its component elements, and how the channel evolves smoothly over time using a first-order autoregressive, or AR(1), model.

---

## 1. The Channel Formula

The composite channel coefficient $h[m]$ for block $m$ is decomposed into a large-scale component and a small-scale component:
$$h[m] = \sqrt{\beta[m]} \cdot g[m]$$

Where:
*   **$h[m]$ (Complex Channel Gain):** The overall scaling and phase rotation applied to the transmitted signal.
*   **$\beta[m]$ (Large-Scale Path Gain):** Represents the average attenuation of the signal power due to distance and environmental shadowing.
*   **$g[m]$ (Small-Scale Fading):** Represents rapid fluctuations in the received signal amplitude and phase caused by multipath interference.

---

## 2. Elements of the Channel

### A. Large-Scale Path Gain ($\beta$)
The path gain $\beta[m]$ represents the average signal attenuation over macroscopic distances. It is modeled using a distance-dependent power law:
$$\beta[m] = C_0 \left( \frac{d[m]}{d_0} \right)^{-\alpha}$$

*   **Distance-Dependence:** $\beta[m]$ depends directly on the physical Euclidean distance $d[m]$ between the transmitter and receiver. As the distance increases, the path gain decreases.
*   **Parameters:**
    *   $d_0$: The reference distance (typically $1\text{ m}$).
    *   $C_0$: The reference channel gain at distance $d_0$, given by the free-space gain $\left(\frac{\lambda_D}{4\pi d_0}\right)^2$.
    *   $\alpha$: The path-loss exponent (typically $\alpha = 2$ for free-space line-of-sight propagation).
*   **Temporal Behavior:** Because physical positions change slowly compared to the microsecond-level duration of communication blocks, $\beta[m]$ changes very slowly and is practically constant over short time frames.

### B. Small-Scale Fading ($g$)
The fading coefficient $g[m]$ models the constructive and destructive interference of multiple reflections (multipath) arriving at the receiver.
*   **Rayleigh Distribution:** In environments with many scatterers and no dominant line-of-sight path, $g[m]$ is modeled as a complex Gaussian random variable:
    $$g[m] \sim \mathcal{CN}(0, 1)$$
    *   The real and imaginary parts are independent, zero-mean normal distributions with variance $1/2$: $\text{Re}\{g[m]\}, \text{Im}\{g[m]\} \sim \mathcal{N}(0, 0.5)$.
    *   The magnitude $|g[m]|$ follows a **Rayleigh distribution** representing the envelope fading.
    *   The phase $\angle g[m]$ is uniformly distributed between $0$ and $2\pi$.

---

## 3. Why Fading ($g$) Changes Smoothly Over Time

In basic simulations, $g[m]$ is often generated as an independent, random value for each block $m$. However, this is **physically unrealistic**:
*   A slow-time block in your system (e.g., for $N_D = 127$ or $1023$) is extremely short: $T_{\text{block}} \approx 5.08\text{ }\mu\text{s}$ to $40.92\text{ }\mu\text{s}$.
*   At a relative speed of $v = 17\text{ m/s}$, the distance traveled by a node during one block is:
    $$\Delta d = 17\text{ m/s} \times 40.92\text{ }\mu\text{s} \approx 0.7\text{ mm}$$
*   Since the wavelength is $\lambda_D = 12.5\text{ mm}$ (at $24\text{ GHz}$), the node travels less than **$6\%$ of a wavelength** per block.
*   Because the physical position shifts by a tiny fraction of a wavelength, the phases of the arriving multipath components change by small increments.
*   Therefore, **$g[m]$ must change smoothly from block to block** within each trial, rather than jumping abruptly to a completely random value.

---

## 4. The AR(1) Model for Smooth Channel Evolution

To simulate this smooth variation, we use a **First-Order Autoregressive, or AR(1), model**. 

### The AR(1) Equation
The fading coefficient for block $m$ is calculated from the previous block's coefficient plus a random innovation:
$$g[m] = \alpha \cdot g[m-1] + \sqrt{1 - \alpha^2} \cdot w[m]$$

Where:
*   $g[m-1]$: The fading coefficient of the previous block.
*   $w[m] \sim \mathcal{CN}(0, 1)$: Independent complex Gaussian noise representing the new random environmental changes (innovation).
*   $\alpha$: The **temporal correlation coefficient** between consecutive blocks.

### Jakes' Correlation Coefficient ($\alpha$)
The correlation $\alpha$ depends on the maximum Doppler frequency $f_d$ and the block spacing $T_{\text{block}}$, modeled by Jakes' autocorrelation function:
$$\alpha = J_0(2\pi f_d T_{\text{block}})$$
where $J_0(\cdot)$ is the zeroth-order Bessel function of the first kind (in MATLAB: `besselj(0, ...)`).

*   **Low Speeds ($f_d$ is small):** $\alpha \approx 1$. The previous block dominates ($g[m] \approx g[m-1]$), and the channel changes very slowly and smoothly.
*   **High Speeds ($f_d$ is large):** $\alpha$ is smaller. The memory of the past channel decays faster, and the channel fluctuates more rapidly.

### Why this structure works:
The coefficient scaling in the AR(1) model ($\alpha$ and $\sqrt{1 - \alpha^2}$) is mathematically designed to **preserve the statistical properties of Rayleigh fading**. 

If $g[m-1] \sim \mathcal{CN}(0,1)$ and $w[m] \sim \mathcal{CN}(0,1)$ are independent, then the variance of $g[m]$ remains exactly $1$:
$$\text{Var}(g[m]) = \alpha^2 \text{Var}(g[m-1]) + \left(1 - \alpha^2\right) \text{Var}(w[m]) = \alpha^2(1) + (1 - \alpha^2)(1) = 1$$

This ensures that while the channel coefficient $g[m]$ shifts smoothly within a simulation trial, it maintains the correct physical statistical distribution throughout the entire run.

## Visualization
Run this after calculating `A_DU_m` in `BERAnalysis.m`:

```matlab
% Assumes:
%   - fading: FadingSequence object (contains fading.g_DU)
%   - A_DU_m: 1 x M_seq vector of full channel amplitudes (including path loss)
%   - p     : SystemParams object (contains p.PD)

M_seq = length(fading.g_DU);
block_axis = 1:M_seq;

figure('Name', 'Channel Evolution Over Blocks', 'Position', [100, 100, 800, 600]);

% --- Top Subplot: Small-Scale Fading |g| ---
subplot(2, 1, 1);
plot(block_axis, abs(fading.g_DU), 'b-', 'LineWidth', 1.5);
grid on;
ylabel('|g_{DU}[m]| (Rayleigh Fading)');
xlabel('Block Index (m)');
title('Small-Scale Rayleigh Fading Magnitude Over Blocks');

% --- Bottom Subplot: Full Channel Amplitude |h_DU_m| ---
subplot(2, 1, 2);
plot(block_axis, abs(A_DU_m)./sqrt(p.PD), 'r-', 'LineWidth', 1.5);
grid on;
ylabel('|h_{DU}[m]| (Path Loss + Fading)');
xlabel('Block Index (m)');
title('Full Desired Channel Amplitude Over Blocks');  
```

![](../../Z_backup/long_sequence_rayleigh/+ntn/Pasted%20image%2020260623160645.png)