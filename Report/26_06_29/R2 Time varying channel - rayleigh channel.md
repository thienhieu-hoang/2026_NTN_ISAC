Time-Varying Rayleigh Fading Channels

In short: complex Gaussian coefficient $g \sim \mathcal{CN}(0, 1)$ varying over time, then the envelope $R(t) = |g(t)| = \sqrt{g_I(t)^2 + g_Q(t)^2}$ follows a Rayleigh distribution.
The variation of $g$ over time is not totally random, it varies "smoothly" from time to time.

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
The continuous fading process is discretized block-by-block using a first-order Autoregressive (AR(1)) filter:
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

## Simulation results

### 1. Block-to-Block varying channel

![](../../Z_backup/long_sequence_rayleigh/+ntn/Pasted%20image%2020260623160645.png)

![](../../Z_backup/long_sequence_rayleigh/range_doppler.png)

![](../../Z_backup/long_sequence_rayleigh/ue_ber_perfect_all_blocks.png)


> [!NOTE]
> The simulation and theoretical lines do not match perfectly due to **Inter-Block Interference (IBI)**. Because of the propagation delay ($\tau$), the last $\tau$ chips of the previous block leak into and corrupt the first $\tau$ chips of the current block, distorting the correlation peak.

### 2. Intra-Block varying channel

In 1 block of `[sound, comm]`, the channel varies from ``sound`` period to ``comm`` period.

![](../../long_sequence_rayleigh_intraBlockFading/+ntn/channel_over_time.png)

![](../../long_sequence_rayleigh_intraBlockFading/range_doppler.png)

![](../../long_sequence_rayleigh_intraBlockFading/ue_ber.png)
