# Mathematical Analysis of BER in Time-Varying Rayleigh Fading Channels

This document provides a detailed mathematical and physical analysis of the Bit Error Rate (BER) performance of the direct communication link in the NTN-ISAC system. It explains why transitioning from a static channel to a time-varying channel shifts the BER curves from exponential decay (AWGN behavior) to polynomial decay (Rayleigh fading behavior), explains the physical origin of the Rayleigh fading process, and derives the analytical BER expressions for both perfect and noisy channel estimation.

---

## 1. The Core Discrepancy: Static vs. Time-Varying Channels

In simulations, a common point of confusion is why a channel modeled with a complex Gaussian coefficient $g \sim \mathcal{CN}(0, 1)$ can behave like an AWGN channel. The difference lies in the **time-varying** nature of the channel:

1. **Static (Quasi-Static) Channel:** 
   If $g$ is generated once at the start of a simulation run and held constant across all blocks, the receiver estimates this single value and equalizes it:
   $$z_{\text{eq}}[m] = \frac{z_{\text{mf}}[m]}{g}$$
   After equalization, the channel attenuation is perfectly compensated, and the only remaining randomness is the independent additive thermal noise at each block. When normalized by its average received power, this channel behaves identically to a static **AWGN channel**, resulting in an **exponentially decaying** BER curve.
   
2. **Time-Varying Channel:**
   If $g[m]$ varies block-by-block, the channel coefficient acts as a random process. Over the course of the simulation, the receiver experiences the entire probability distribution of $g[m]$. The average BER is the expectation of the AWGN error probability over this distribution. The occurrence of **deep fades** (where $|g[m]| \approx 0$) dominates the average error rate, changing the BER decay from **exponential** to **polynomial** ($1/\text{SNR}$).

---

## 2. Why the Channel Becomes Rayleigh (The Physical Scattering Model)

### A. Multipath Propagation and the Central Limit Theorem
In a wireless propagation environment, the transmitted signal travels along multiple paths due to reflections, scattering, and diffraction from various objects (buildings, trees, terrain, or the ground). 

At the receiver, the composite signal is the superposition of $N_p$ independent scattered wavefronts. The overall channel coefficient $g(t)$ is the sum of these multipath components:
$$g(t) = \sum_{n=1}^{N_p} a_n(t) e^{-j \left( 2\pi f_c \tau_n(t) - \theta_n \right)}$$
where:
* $a_n(t)$ is the real amplitude of the $n$-th path.
* $f_c$ is the carrier frequency.
* $\tau_n(t)$ is the propagation delay of the $n$-th path.
* $\theta_n$ is the phase shift associated with scattering.

If there is no dominant Line-of-Sight (LOS) path (i.e., all paths are scattered and have comparable energy) and the number of paths $N_p$ is large, we can apply the **Central Limit Theorem (CLT)**. The in-phase (real) component $g_I(t) = \text{Re}\{g(t)\}$ and quadrature (imaginary) component $g_Q(t) = \text{Im}\{g(t)\}$ converge to independent, identically distributed (i.e.d.) Gaussian random variables:
$$g_I(t), g_Q(t) \sim \mathcal{N}\left(0, \sigma^2\right)$$
where the total average power is normalized to 1:
$$\mathbb{E}[|g(t)|^2] = \mathbb{E}[g_I(t)^2] + \mathbb{E}[g_Q(t)^2] = 2\sigma^2 = 1 \implies \sigma^2 = \frac{1}{2}$$
Therefore, the complex channel gain is circular symmetric complex Gaussian:
$$g(t) = g_I(t) + j g_Q(t) \sim \mathcal{CN}(0, 1)$$

### B. The Rayleigh Envelope and Uniform Phase
The envelope $R(t) = |g(t)| = \sqrt{g_I(t)^2 + g_Q(t)^2}$ represents the channel amplitude. Since it is the square root of the sum of two squared independent Gaussian variables, its probability density function (PDF) is **Rayleigh distributed**:
$$f_R(r) = \frac{r}{\sigma^2} e^{-\frac{r^2}{2\sigma^2}} = 2r e^{-r^2}, \quad r \ge 0 \quad (\text{for } \sigma^2 = 1/2)$$

The phase $\Theta(t) = \arctan2\left(g_Q(t), g_I(t)\right)$ is **uniformly distributed**:
$$f_\Theta(\theta) = \frac{1}{2\pi}, \quad \theta \in [-\pi, \pi)$$

---

## 3. Dynamic Fading: The Rayleigh Process in Time

When the transmitter or receiver moves, the path delays $\tau_n(t)$ change continuously, causing the phase terms to rotate. This temporal variation turns the random variable $g$ into a time-continuous random process $g(t)$, known as a **Rayleigh fading process**.

### A. Clarke's / Jakes' Autocorrelation Model
Under the assumption of isotropic scattering (where scattered waves arrive from all horizontal directions with equal probability), the autocorrelation function of the channel process $g(t)$ is given by:
$$R_g(\tau) = \mathbb{E}[g(t) g^*(t-\tau)] = J_0(2\pi f_D \tau)$$
where:
* $J_0(\cdot)$ is the zeroth-order Bessel function of the first kind.
* $f_D$ is the maximum Doppler shift, defined as $f_D = v / \lambda$ (where $v$ is the relative speed and $\lambda$ is the wavelength).

### B. The AR(1) Approximation in the Codebase
In the simulation ([FadingSequence.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/long_sequence_rayleigh/+ntn/FadingSequence.m)), the continuous fading process is discretized block-by-block using a first-order Autoregressive (AR(1)) filter:
$$g[m] = \alpha g[m-1] + \sqrt{1 - \alpha^2} w[m]$$
where:
* $m$ is the slow-time block index.
* $w[m] \sim \mathcal{CN}(0,1)$ is i.i.d. complex Gaussian noise.
* $\alpha = J_0(2\pi f_D T_{\text{block}})$ is the block-to-block correlation coefficient.

#### Proof of Marginal Distribution Stationarity
We prove that this AR(1) recurrence relation preserves the marginal distribution $g[m] \sim \mathcal{CN}(0,1)$ for all $m$:
Assume $g[m-1] \sim \mathcal{CN}(0, 1)$, meaning $\mathbb{E}[|g[m-1]|^2] = 1$. The innovation noise $w[m] \sim \mathcal{CN}(0,1)$ is independent of $g[m-1]$. The expected power of $g[m]$ is:
$$\mathbb{E}[|g[m]|^2] = \mathbb{E}\left[ \left( \alpha g[m-1] + \sqrt{1 - \alpha^2} w[m] \right) \left( \alpha g^*[m-1] + \sqrt{1 - \alpha^2} w^*[m] \right) \right]$$
Expanding this expression:
$$\mathbb{E}[|g[m]|^2] = \alpha^2 \mathbb{E}[|g[m-1]|^2] + (1 - \alpha^2) \mathbb{E}[|w[m]|^2] + 2\alpha \sqrt{1 - \alpha^2} \text{Re}\{\mathbb{E}[g[m-1] w^*[m]]\}$$
Since $g[m-1]$ and $w[m]$ are independent and $\mathbb{E}[w[m]] = 0$, the cross-correlation term is zero. Substituting the unit variances:
$$\mathbb{E}[|g[m]|^2] = \alpha^2 (1) + (1 - \alpha^2) (1) + 0 = 1$$
By induction, because the sum of independent Gaussian variables is Gaussian, $g[m]$ remains marginally distributed as $\mathcal{CN}(0,1)$ at every block $m$.

### C. Fading Behavior Limits based on Velocity
* **Stationary / Very Slow Motion ($\alpha \to 1$):**
  When velocity $v \to 0$, $f_D \to 0$, which yields $\alpha = J_0(0) = 1$. The process simplifies to:
  $$g[m] = g[m-1] = \dots = g[1]$$
  This is the static channel limit, where the channel is a single static realization.
* **High-Speed Time-Varying Motion ($\alpha < 1$):**
  As velocity increases, $f_D$ grows, causing $\alpha$ to drop below 1. The channel values decorrelate over time, and the receiver samples the entire Rayleigh envelope, including its deep fades.

---

## 4. Derivation of Bit Error Rate (BER)

We analyze the performance of Binary Phase Shift Keying (BPSK) where data symbols $x \in \{+1, -1\}$.

### A. Static AWGN Channel (Reference)
In a static AWGN channel, the received signal is $y = x + n$, where $n \sim \mathcal{CN}(0, N_0)$. The signal energy per bit is $E_b$. The instantaneous signal-to-noise ratio is $\gamma = E_b / N_0$.
The probability of a bit error is the probability that noise crosses the decision boundary:
$$P_{e, \text{AWGN}}(\gamma) = Q\left(\sqrt{2\gamma}\right) = \frac{1}{2} \operatorname{erfc}\left(\sqrt{\gamma}\right)$$
At high SNR ($\gamma \gg 1$), the $Q$-function decays exponentially:
$$P_{e, \text{AWGN}}(\gamma) \approx \frac{1}{2\sqrt{\pi\gamma}} e^{-\gamma}$$

### B. Time-Varying Rayleigh Fading Channel
In a fading channel, the received signal at block $m$ is $y[m] = g[m] x[m] + n[m]$. The instantaneous SNR varies according to the channel gain:
$$\gamma = |g|^2 \bar{\gamma}$$
where $\bar{\gamma} = \mathbb{E}[\gamma]$ is the average received SNR.

#### 1. PDF of the Instantaneous SNR
Let $X = |g|^2$ represent the channel power. Since $|g|$ is Rayleigh distributed, its square $X$ follows an exponential distribution:
$$f_X(x) = e^{-x}, \quad x \ge 0$$
Using the change of variables $\gamma = x \bar{\gamma}$, we find the PDF of the instantaneous SNR $\gamma$:
$$f_\Gamma(\gamma) = \frac{1}{\bar{\gamma}} f_X\left(\frac{\gamma}{\bar{\gamma}}\right) = \frac{1}{\bar{\gamma}} e^{-\frac{\gamma}{\bar{\gamma}}}, \quad \gamma \ge 0$$

#### 2. Average BER Integration
The average BER in a fading channel, $P_{e, \text{Rayleigh}}$, is the expectation of the AWGN BER over all possible values of $\gamma$:
$$P_{e, \text{Rayleigh}} = \int_{0}^{\infty} P_{e, \text{AWGN}}(\gamma) f_\Gamma(\gamma) d\gamma = \int_{0}^{\infty} \left[ \frac{1}{2} \operatorname{erfc}\left(\sqrt{\gamma}\right) \right] \left[ \frac{1}{\bar{\gamma}} e^{-\frac{\gamma}{\bar{\gamma}}} \right] d\gamma$$

We evaluate this integral using **Integration by Parts**:
$$\int u \, dv = u v - \int v \, du$$
Set:
$$u = \frac{1}{2} \operatorname{erfc}\left(\sqrt{\gamma}\right) \implies du = -\frac{1}{2\sqrt{\pi \gamma}} e^{-\gamma} d\gamma$$
$$dv = \frac{1}{\bar{\gamma}} e^{-\frac{\gamma}{\bar{\gamma}}} d\gamma \implies v = -e^{-\frac{\gamma}{\bar{\gamma}}}$$

Applying the boundary evaluations and substituting $u, v, du, dv$:
$$P_{e, \text{Rayleigh}} = \left[ -\frac{1}{2} \operatorname{erfc}\left(\sqrt{\gamma}\right) e^{-\frac{\gamma}{\bar{\gamma}}} \right]_0^\infty - \int_{0}^{\infty} \left( -e^{-\frac{\gamma}{\bar{\gamma}}} \right) \left( -\frac{1}{2\sqrt{\pi \gamma}} e^{-\gamma} d\gamma \right)$$

Evaluating the first term:
* At $\gamma \to \infty$, $e^{-\infty} = 0$, so the term vanishes.
* At $\gamma = 0$, $\operatorname{erfc}(0) = 1$ and $e^0 = 1$, yielding $-\frac{1}{2}(1)(1) = -1/2$.
* Thus, the boundary term is: $0 - (-1/2) = \frac{1}{2}$.

Now, simplify the remaining integral:
$$P_{e, \text{Rayleigh}} = \frac{1}{2} - \frac{1}{2\sqrt{\pi}} \int_{0}^{\infty} \frac{1}{\sqrt{\gamma}} e^{-\gamma \left(1 + \frac{1}{\bar{\gamma}}\right)} d\gamma$$

To solve the integral, make the substitution $z = \gamma \left( 1 + \frac{1}{\bar{\gamma}} \right) \implies \gamma = \frac{z}{1 + 1/\bar{\gamma}}$ and $d\gamma = \frac{dz}{1 + 1/\bar{\gamma}}$:
$$\int_{0}^{\infty} \frac{1}{\sqrt{\gamma}} e^{-\gamma \left(1 + \frac{1}{\bar{\gamma}}\right)} d\gamma = \int_{0}^{\infty} \sqrt{1 + \frac{1}{\bar{\gamma}}} \frac{1}{\sqrt{z}} e^{-z} \frac{dz}{1 + 1/\bar{\gamma}} = \frac{1}{\sqrt{1 + \frac{1}{\bar{\gamma}}}} \int_{0}^{\infty} z^{-1/2} e^{-z} dz$$

The integral is the standard Gamma function representation for $\Gamma(1/2) = \sqrt{\pi}$:
$$\int_{0}^{\infty} z^{-1/2} e^{-z} dz = \sqrt{\pi}$$

Substitute this back into the equation:
$$P_{e, \text{Rayleigh}} = \frac{1}{2} - \frac{1}{2\sqrt{\pi}} \left( \sqrt{\frac{\bar{\gamma}}{1 + \bar{\gamma}}} \right) \sqrt{\pi} = \frac{1}{2} \left( 1 - \sqrt{\frac{\bar{\gamma}}{1 + \bar{\gamma}}} \right)$$

This is the exact analytical expression for the BPSK BER over a Rayleigh fading channel:
$$P_{e, \text{Rayleigh}} = \frac{1}{2} \left( 1 - \sqrt{\frac{\bar{\gamma}}{1 + \bar{\gamma}}} \right)$$

#### 3. High SNR Approximation and Diversity Order
For high SNR ($\bar{\gamma} \gg 1$), we expand the radical using a Taylor series:
$$\sqrt{\frac{\bar{\gamma}}{1 + \bar{\gamma}}} = \left( 1 + \frac{1}{\bar{\gamma}} \right)^{-1/2} \approx 1 - \frac{1}{2\bar{\gamma}} + \mathcal{O}\left(\frac{1}{\bar{\gamma}^2}\right)$$
Substituting this back into the BER formula:
$$P_{e, \text{Rayleigh}} \approx \frac{1}{2} \left( 1 - \left( 1 - \frac{1}{2\bar{\gamma}} \right) \right) = \frac{1}{4\bar{\gamma}}$$
This approximation shows that the BER decays at a rate of $\bar{\gamma}^{-1}$ (diversity order $d = 1$). A tenfold increase in SNR (10 dB) only decreases the error rate by a factor of 10. In contrast, for an AWGN channel, a 10 dB increase at high SNR decreases the error rate by dozens of orders of magnitude.

---

## 5. The Physical Cause of Degradation: Deep Fades

The reason fading channels perform so poorly compared to AWGN channels is the occurrence of **deep fades**. 

Even if the average SNR $\bar{\gamma}$ is very high, there is a small but non-zero probability that the channel envelope $|g|$ will drop close to zero. When $|g|^2 < 1/\bar{\gamma}$, the instantaneous SNR falls below 0 dB, making bit detection highly error-prone (instantaneous BER $\approx 0.5$). 

The probability of the channel dropping into a deep fade below some threshold $\epsilon \ll 1$ is:
$$P(|g|^2 < \epsilon) = \int_{0}^{\epsilon} e^{-x} dx = 1 - e^{-\epsilon} \approx \epsilon$$
Letting $\epsilon = 1/\bar{\gamma}$, we find that the probability of the channel dropping below 0 dB instantaneous SNR is approximately:
$$P_{\text{fade}} \approx \frac{1}{\bar{\gamma}}$$
Because the error rate during these deep fades is so high, they dominate the average BER calculation. The system performance is limited not by its behavior when the channel is at its average value, but by how often the channel fails completely.

### Quantitative Comparison: AWGN vs. Rayleigh BER
The table below compares the theoretical BPSK BER for AWGN and Rayleigh fading channels at different average SNR ($\bar{\gamma}$) levels:

| SNR ($\bar{\gamma}$) | AWGN Theoretical BER | Rayleigh Theoretical BER |
| :---: | :---: | :---: |
| **0 dB** | $7.86 \times 10^{-2}$ | $1.46 \times 10^{-1}$ |
| **10 dB** | $7.83 \times 10^{-6}$ | $2.33 \times 10^{-2}$ |
| **20 dB** | $3.88 \times 10^{-23}$ | $2.48 \times 10^{-3}$ |
| **30 dB** | $1.11 \times 10^{-218}$ | $2.49 \times 10^{-4}$ |
| **40 dB** | $\approx 0.0$ | $2.50 \times 10^{-5}$ |

At an average SNR of 20 dB, the AWGN channel is practically error-free. In contrast, the Rayleigh fading channel experiences an error rate of about 1 in every 400 bits, which is entirely dominated by the 1% probability of experiencing a deep fade.

---

## 6. Impact of Noisy Channel Estimation

In practical receivers, the channel coefficient $g$ is not known perfectly and must be estimated using pilots (sounding).

### A. Mathematical Model of 1-Tap Pilot Estimation
Let the received pilot symbol be:
$$y_p = g x_p + n_p$$
Assuming the pilot is normalized ($x_p = 1$), the pilot-assisted channel estimate $\hat{g}$ is:
$$\hat{g} = g + n_p$$
where $n_p \sim \mathcal{CN}(0, \sigma_n^2)$ is the estimation noise.
When transmitting a BPSK data symbol $x_d \in \{+1, -1\}$, the received data is:
$$y_d = g x_d + n_d$$
where $n_d \sim \mathcal{CN}(0, \sigma_n^2)$ is the data-phase noise.

### B. Demodulation Decision Statistic
The receiver equalizes the data by multiplying it by the complex conjugate of the channel estimate:
$$\hat{x}_d = \operatorname{Re}\{y_d \hat{g}^*\} = \operatorname{Re}\left\{ (g x_d + n_d)(g + n_p)^* \right\}$$
$$\hat{x}_d = |g|^2 x_d + \operatorname{Re}\{ g x_d n_p^* + g^* n_d + n_d n_p^* \}$$

At reasonable operating SNRs, the second-order noise term $n_d n_p^*$ is negligible. The decision statistic is:
$$\hat{x}_d \approx |g|^2 x_d + v$$
where $v = \text{Re}\{ g x_d n_p^* + g^* n_d \}$ is the composite noise term. 

Conditioned on the channel state $g$, $v$ is a real Gaussian random variable. Its variance is:
$$\sigma_v^2 = \mathbb{E}\left[ \left( \operatorname{Re}\{ g x_d n_p^* + g^* n_d \} \right)^2 \middle| g \right] = \frac{1}{2} |g|^2 \sigma_n^2 + \frac{1}{2} |g|^2 \sigma_n^2 = |g|^2 \sigma_n^2$$

### C. Effective SNR and Fading BER
The effective instantaneous SNR at the decision device is:
$$\gamma_{\text{eff}} = \frac{(\text{mean})^2}{2 \sigma_v^2} = \frac{(|g|^2)^2}{2 |g|^2 \sigma_n^2} = \frac{|g|^2}{2\sigma_n^2} = \frac{1}{2} \gamma$$
where $\gamma = |g|^2 / \sigma_n^2$ is the perfect-CSI instantaneous SNR.

The noisy channel estimate introduces a **3 dB SNR penalty** (cutting the effective SNR in half) because noise corrupts both the channel estimate (pilot phase) and the data symbol (demodulation phase) equally.

Integrating this effective SNR over the Rayleigh fading envelope yields:
$$P_{e, \text{est}} = \frac{1}{2} \left( 1 - \sqrt{\frac{\bar{\gamma}/2}{1 + \bar{\gamma}/2}} \right)$$

At high SNR ($\bar{\gamma} \gg 1$), this simplifies to:
$$P_{e, \text{est}} \approx \frac{1}{2} \left( 1 - \left( 1 - \frac{1}{\bar{\gamma}} \right) \right) = \frac{1}{2\bar{\gamma}}$$
This matches the simulation analysis equation used in the project:
$$P_{e, \text{est}} = \frac{1}{2(1 + \bar{\gamma})}$$
At high SNR, this yields a BER that is exactly **twice** (3 dB worse than) the perfect-CSI fading BER ($1/(4\bar{\gamma})$).

---

## 7. Rician Fading: The Bridge Between AWGN and Rayleigh

If a strong direct Line-of-Sight (LOS) path exists alongside the scattered paths, the channel coefficient $g$ no longer has a zero mean. The small-scale fading is modeled as a **Rician distribution**:
$$g = \sqrt{\frac{K}{K+1}} e^{j\theta} + \sqrt{\frac{1}{K+1}} w$$
where:
* $w \sim \mathcal{CN}(0, 1)$ represents the random scattered components.
* $\theta$ is the phase of the LOS component.
* $K$ is the **Rician K-factor**, defined as the ratio of the power in the LOS component to the power in the scattered paths:
  $$K = \frac{\text{LOS Power}}{\text{Scattered Power}}$$

### Limiting Behaviors of Rician Fading
* **Pure Rayleigh Fading ($K = 0$):**
  If there is no LOS component ($K = 0$), the formula simplifies to $g = w \sim \mathcal{CN}(0, 1)$, which corresponds to pure Rayleigh fading (polynomial decay in BER).
* **Pure AWGN Channel ($K \to \infty$):**
  As the direct path becomes infinitely stronger than the reflections ($K \to \infty$), the scattered component vanishes, leaving a constant channel amplitude:
  $$|g| \to 1$$
  This yields a static channel, which simplifies to the pure AWGN BER curve (exponential decay).
