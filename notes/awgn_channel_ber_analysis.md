# Mathematical Analysis of BER in AWGN Channels

This document provides a detailed mathematical and physical analysis of the Bit Error Rate (BER) performance of the direct communication link in the NTN-ISAC system under static Additive White Gaussian Noise (AWGN) conditions. It derives the analytical BER expressions for both perfect Channel State Information (CSI) and noisy pilot-assisted channel estimation (using the Moment Generating Function inversion method), and contrasts these results with the Rayleigh fading channel case.

---

## 1. The AWGN Channel Model

In a static (or quasi-static) AWGN channel, the channel coefficient $h$ is assumed to be a constant complex value:
$$h = \sqrt{\beta} e^{j\phi}$$
where:
* $\beta$ is the constant large-scale path gain.
* $\phi$ is a constant phase shift.

Unlike fading channels, there is no time-varying small-scale fading term ($g = 1$). The received signal at the UE during the data period of block $m$, after delay synchronization to the direct-path delay $\ell_{DU}$, is:
$$y_U[n,m] = \sqrt{P_D} h s_{\mathrm{mls}}[n] d_m + n_U[n,m], \quad 0 \le n < N_D$$
where $n_U[n,m] \sim \mathcal{CN}(0, \sigma_U^2)$ is i.i.d. complex white Gaussian noise, and $d_m \in \{+1, -1\}$ is the BPSK data symbol.

---

## 2. Demodulation, Despreading, and Equalization

### A. Matched Filtering (Despreading)
The UE correlates the received data samples with the reference MLS sequence $s_{\mathrm{mls}}$ to compress the energy:
$$z_{\mathrm{mf}}[m] = \sum_{n=0}^{N_D-1} y_U[n,m] s_{\mathrm{mls}}[n] = N_D \sqrt{P_D} h d_m + w_{\mathrm{data}}[m]$$
where the despread noise term is:
$$w_{\mathrm{data}}[m] = \sum_{n=0}^{N_D-1} n_U[n,m] s_{\mathrm{mls}}[n] \sim \mathcal{CN}(0, N_D \sigma_U^2)$$

### B. Equalization and BPSK Slicing
To recover the data, the despread peak is equalized using the channel estimate $\hat{h}$ and sliced:
$$z_{\mathrm{eq}}[m] = \frac{z_{\mathrm{mf}}[m]}{N_D \hat{h}}$$
$$\hat{d}_m = \mathrm{sign}\left(\mathrm{Re}\{z_{\mathrm{eq}}[m]\}\right)$$

---

## 3. Derivation of Theoretical BER with Perfect CSI

With perfect CSI, the channel coefficient $h$ is known exactly ($\hat{h} = h$). 
**Assuming $d_m = +1$ is transmitted**, the decision variable is:
$$D = \mathrm{Re}\left\{ z_{\mathrm{mf}}[m] h^* \right\} = N_D \sqrt{P_D} |h|^2 + v$$
where $v = \mathrm{Re}\{w_{\mathrm{data}}[m] h^*\}$.
(Because
$$\hat{d}_m = \mathrm{sign}\left(\mathrm{Re}\{z_{\mathrm{eq}}[m]\}\right)=\mathrm{sign}(D)$$
)
Since $w_{\mathrm{data}}[m] \sim \mathcal{CN}(0, N_D \sigma_U^2)$, the noise term $v$ is a real Gaussian random variable with zero mean. Its variance is:
$$\sigma_v^2 = \text{Var}\left(\mathrm{Re}\{w_{\mathrm{data}}[m] h^*\}\right) = \frac{1}{2} |h|^2 \text{Var}(w_{\mathrm{data}}[m]) = \frac{1}{2} |h|^2 N_D \sigma_U^2$$

An error occurs if the decision statistic falls below the threshold, i.e., $D < 0$:
$$P_e = P\left(v < -N_D \sqrt{P_D} |h|^2\right) = Q\left( \frac{N_D \sqrt{P_D} |h|^2}{\sigma_v} \right) = Q\left( \frac{N_D \sqrt{P_D} |h|^2}{\sqrt{\frac{1}{2} |h|^2 N_D \sigma_U^2}} \right) = Q\left( \sqrt{\frac{2 N_D P_D |h|^2}{\sigma_U^2}} \right)$$

Let us define the received energy per bit $E_b$ and the noise power spectral density $N_0$:
* $E_b = N_D P_D |h|^2$ (the total power received over $N_D$ chips).
* $N_0 = \sigma_U^2$ (the noise variance per complex sample).

The signal-to-noise ratio is $\gamma = E_b / N_0$. Substituting this yields the standard BPSK AWGN BER formula:
$$P_{e, \text{AWGN}}(\gamma) = Q\left(\sqrt{2\gamma}\right) = \frac{1}{2} \operatorname{erfc}\left(\sqrt{\gamma}\right)$$

---

## 4. Derivation of Theoretical BER with Noisy Pilot Estimation (MGF Inversion)

In a practical system, the channel is estimated using the sounding matched filter over a pilot period of length $N_D$. The channel estimate $\hat{h}$ is:
$$\hat{h} = h + w_{\mathrm{sound}}'$$
where $w_{\mathrm{sound}}' \sim \mathcal{CN}(0, \sigma_0^2)$ is the estimation noise, with variance $\sigma_0^2 = \sigma_U^2 / N_D$.

Let us normalize the despread peak and the channel estimate:
$$X = \frac{z_{\mathrm{mf}}[m]}{N_D} = h d_m + w_{\mathrm{data}}'$$
$$Y = \hat{h} = h + w_{\mathrm{sound}}'$$
where $w_{\mathrm{data}}', w_{\mathrm{sound}}' \sim \mathcal{CN}(0, \sigma_0^2)$ are independent complex Gaussian noises.

---
**Explain
The variance $\sigma_0^2 = \frac{\sigma_U^2}{N_D}$ represents the noise power **after correlation/despreading** over a block of length $N_D$. This reduction in noise variance by a factor of $N_D$ is known as **processing gain** (or integration gain).

Here is the step-by-step mathematical derivation of how this variance arises for both the normalized data noise ($w_{\mathrm{data}}'$) and the pilot estimation noise ($w_{\mathrm{sound}}'$):
##### 1. For the Data Path ($w_{\mathrm{data}}'$)
The matched filter correlates the received signal over $N_D$ chips. The unnormalized noise term is:
$$w_{\mathrm{data}}[m] = \sum_{n=0}^{N_D-1} n_U[n,m] s_{\mathrm{mls}}[n]$$

Since $n_U[n,m] \sim \mathcal{CN}(0, \sigma_U^2)$ are i.i.d. and the sequence chips have unit magnitude ($|s_{\mathrm{mls}}[n]|^2 = 1$), the variance of this sum is:
$$\text{Var}(w_{\mathrm{data}}[m]) = \sum_{n=0}^{N_D-1} |s_{\mathrm{mls}}[n]|^2 \text{Var}(n_U[n,m]) = N_D \sigma_U^2$$

In line 64, we normalize the decision statistic by dividing it by $N_D$ to keep the signal coefficient at $h$ (assuming $P_D = 1$ for simplicity):
$$X = \frac{z_{\mathrm{mf}}[m]}{N_D} = h d_m + w_{\mathrm{data}}'$$
where the normalized noise is $w_{\mathrm{data}}' = \frac{w_{\mathrm{data}}[m]}{N_D}$.

The variance of this normalized noise is:
$$\text{Var}(w_{\mathrm{data}}') = \text{Var}\left( \frac{w_{\mathrm{data}}[m]}{N_D} \right) = \frac{1}{N_D^2} \text{Var}(w_{\mathrm{data}}[m]) = \frac{N_D \sigma_U^2}{N_D^2} = \frac{\sigma_U^2}{N_D}$$

##### 2. For the Pilot/Sounding Path ($w_{\mathrm{sound}}'$)
Similarly, during the pilot (sounding) period, the receiver correlates the received signal over a pilot sequence of length $N_D$ to estimate the channel $h$. 

Assuming a normalized pilot sequence $s_{\mathrm{pilot}}$ of **length $N_D$ with unit-power chips**, the channel estimator correlates the received samples to average out the noise:
$$\hat{h} = \frac{1}{N_D} \sum_{n=0}^{N_D-1} y_{\mathrm{pilot}}[n] s_{\mathrm{pilot}}^*[n] = h + w_{\mathrm{sound}}'$$
where the estimation noise is:
$$w_{\mathrm{sound}}' = \frac{1}{N_D} \sum_{n=0}^{N_D-1} n_{\mathrm{pilot}}[n] s_{\mathrm{pilot}}^*[n]$$

Because we are averaging the uncorrelated noise samples over $N_D$ chips, the variance of this estimator noise is reduced by a factor of $N_D$:
$$\text{Var}(w_{\mathrm{sound}}') = \frac{1}{N_D^2} \sum_{n=0}^{N_D-1} |s_{\mathrm{pilot}}^*[n]|^2 \text{Var}(n_{\mathrm{pilot}}[n]) = \frac{N_D \sigma_U^2}{N_D^2} = \frac{\sigma_U^2}{N_D}$$
**End explain

---

### A. Quadratic Form Representation
Assuming $d_m = +1$, a decision error occurs if $\mathrm{Re}\{X Y^*\} < 0$. We express the decision statistic $D = \mathrm{Re}\{X Y^*\} = \frac{1}{2}(X Y^* + X^* Y)$ as a Hermitian quadratic form:
$$D = \mathbf{v}^H \mathbf{Q} \mathbf{v}$$
where:
$$\mathbf{v} = \begin{bmatrix} X \\ Y \end{bmatrix} \sim \mathcal{CN}(\boldsymbol{\mu}, \mathbf{R}), \quad \boldsymbol{\mu} = \begin{bmatrix} h \\ h \end{bmatrix}, \quad \mathbf{R} = \sigma_0^2 \mathbf{I}_2, \quad \mathbf{Q} = \frac{1}{2} \begin{bmatrix} 0 & 1 \\ 1 & 0 \end{bmatrix}$$

### B. Moment Generating Function (MGF)
The MGF of $D$, defined as $\phi_D(s) = \mathbb{E}[e^{-s D}]$, is given by:
$$\phi_D(s) = \frac{\exp\left( -\boldsymbol{\mu}^H \mathbf{R}^{-1} [(\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} - \mathbf{I}] \boldsymbol{\mu} \right)}{\det(\mathbf{I} + s \mathbf{R} \mathbf{Q})}$$

---
**Explain
Here is the complete, step-by-step algebraic expansion of the derivation. 

#### Step 1: Write down the integration setup
We want to evaluate:
$$\mathbb{E}[e^{-s D}] = \mathbb{E}[e^{-s \mathbf{v}^H \mathbf{Q} \mathbf{v}}]$$

Using the probability density function (PDF) of the complex Gaussian vector $\mathbf{v} \sim \mathcal{CN}(\boldsymbol{\mu}, \mathbf{R})$, this expectation is the multidimensional integral:
$$\mathbb{E}[e^{-s D}] = \int_{\mathbb{C}^N} e^{-s \mathbf{v}^H \mathbf{Q} \mathbf{v}} \cdot \left( \frac{1}{\pi^N \det(\mathbf{R})} \exp\left( -(\mathbf{v} - \boldsymbol{\mu})^H \mathbf{R}^{-1} (\mathbf{v} - \boldsymbol{\mu}) \right) \right) d\mathbf{v}$$

Let's pull the constant scalar $\frac{1}{\pi^N \det(\mathbf{R})}$ outside the integral:
$$\mathbb{E}[e^{-s D}] = \frac{1}{\pi^N \det(\mathbf{R})} \int_{\mathbb{C}^N} \exp\left( -s \mathbf{v}^H \mathbf{Q} \mathbf{v} - (\mathbf{v} - \boldsymbol{\mu})^H \mathbf{R}^{-1} (\mathbf{v} - \boldsymbol{\mu}) \right) d\mathbf{v}$$


#### Step 2: Expand the exponent
Let's focus on the term inside the exponential function:
$$\text{Exponent} = -s \mathbf{v}^H \mathbf{Q} \mathbf{v} - (\mathbf{v} - \boldsymbol{\mu})^H \mathbf{R}^{-1} (\mathbf{v} - \boldsymbol{\mu})$$

First, expand the second quadratic term:
$$(\mathbf{v} - \boldsymbol{\mu})^H \mathbf{R}^{-1} (\mathbf{v} - \boldsymbol{\mu}) = (\mathbf{v}^H - \boldsymbol{\mu}^H) \mathbf{R}^{-1} (\mathbf{v} - \boldsymbol{\mu})$$
$$= \mathbf{v}^H \mathbf{R}^{-1} \mathbf{v} - \mathbf{v}^H \mathbf{R}^{-1} \boldsymbol{\mu} - \boldsymbol{\mu}^H \mathbf{R}^{-1} \mathbf{v} + \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu}$$

Substitute this back into the overall exponent:
$$\text{Exponent} = -s \mathbf{v}^H \mathbf{Q} \mathbf{v} - \left( \mathbf{v}^H \mathbf{R}^{-1} \mathbf{v} - \mathbf{v}^H \mathbf{R}^{-1} \boldsymbol{\mu} - \boldsymbol{\mu}^H \mathbf{R}^{-1} \mathbf{v} + \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu} \right)$$
$$\text{Exponent} = - \mathbf{v}^H \left( \mathbf{R}^{-1} + s \mathbf{Q} \right) \mathbf{v} + \mathbf{v}^H \mathbf{R}^{-1} \boldsymbol{\mu} + \boldsymbol{\mu}^H \mathbf{R}^{-1} \mathbf{v} - \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu}$$

To simplify the math, let's define a new matrix $\tilde{\mathbf{R}}^{-1}$:
$$\tilde{\mathbf{R}}^{-1} = \mathbf{R}^{-1} + s \mathbf{Q}$$

This yields:
$$\text{Exponent} = - \mathbf{v}^H \tilde{\mathbf{R}}^{-1} \mathbf{v} + \mathbf{v}^H \mathbf{R}^{-1} \boldsymbol{\mu} + \boldsymbol{\mu}^H \mathbf{R}^{-1} \mathbf{v} - \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu}$$

#### Step 3: Complete the square
We want to rewrite this exponent in the standard quadratic form with a new mean vector $\tilde{\boldsymbol{\mu}}$ and a constant term $K$:
$$\text{Exponent} = - (\mathbf{v} - \tilde{\boldsymbol{\mu}})^H \tilde{\mathbf{R}}^{-1} (\mathbf{v} - \tilde{\boldsymbol{\mu}}) + K$$

Let's expand this target expression to see what $\tilde{\boldsymbol{\mu}}$ and $K$ must be:
$$- (\mathbf{v} - \tilde{\boldsymbol{\mu}})^H \tilde{\mathbf{R}}^{-1} (\mathbf{v} - \tilde{\boldsymbol{\mu}}) + K = - \mathbf{v}^H \tilde{\mathbf{R}}^{-1} \mathbf{v} + \mathbf{v}^H \tilde{\mathbf{R}}^{-1} \tilde{\boldsymbol{\mu}} + \tilde{\boldsymbol{\mu}}^H \tilde{\mathbf{R}}^{-1} \mathbf{v} - \tilde{\boldsymbol{\mu}}^H \tilde{\mathbf{R}}^{-1} \tilde{\boldsymbol{\mu}} + K$$

By comparing our actual exponent with this target expansion:
1.  **Linear term in $\mathbf{v}^H$:** 
    $$\mathbf{v}^H \tilde{\mathbf{R}}^{-1} \tilde{\boldsymbol{\mu}} = \mathbf{v}^H \mathbf{R}^{-1} \boldsymbol{\mu} \implies \tilde{\mathbf{R}}^{-1} \tilde{\boldsymbol{\mu}} = \mathbf{R}^{-1} \boldsymbol{\mu} \implies \tilde{\boldsymbol{\mu}} = \tilde{\mathbf{R}} \mathbf{R}^{-1} \boldsymbol{\mu}$$
2.  **Constant terms:**
    $$- \tilde{\boldsymbol{\mu}}^H \tilde{\mathbf{R}}^{-1} \tilde{\boldsymbol{\mu}} + K = - \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu} \implies K = \tilde{\boldsymbol{\mu}}^H \tilde{\mathbf{R}}^{-1} \tilde{\boldsymbol{\mu}} - \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu}$$

#### Step 4: Simplify the constant $K$
Substitute $\tilde{\boldsymbol{\mu}} = \tilde{\mathbf{R}} \mathbf{R}^{-1} \boldsymbol{\mu}$ into $K$:
$$K = \left( \tilde{\mathbf{R}} \mathbf{R}^{-1} \boldsymbol{\mu} \right)^H \tilde{\mathbf{R}}^{-1} \left( \tilde{\mathbf{R}} \mathbf{R}^{-1} \boldsymbol{\mu} \right) - \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu}$$
$$K = \boldsymbol{\mu}^H \mathbf{R}^{-1} \tilde{\mathbf{R}} \tilde{\mathbf{R}}^{-1} \tilde{\mathbf{R}} \mathbf{R}^{-1} \boldsymbol{\mu} - \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu}$$

Since $\tilde{\mathbf{R}} \tilde{\mathbf{R}}^{-1} = \mathbf{I}$, this simplifies to:
$$K = \boldsymbol{\mu}^H \mathbf{R}^{-1} \tilde{\mathbf{R}} \mathbf{R}^{-1} \boldsymbol{\mu} - \boldsymbol{\mu}^H \mathbf{R}^{-1} \boldsymbol{\mu}$$
$$K = \boldsymbol{\mu}^H \mathbf{R}^{-1} \left( \tilde{\mathbf{R}} - \mathbf{R} \right) \mathbf{R}^{-1} \boldsymbol{\mu}$$

We can express $\tilde{\mathbf{R}} - \mathbf{R}$ in terms of our known matrices:
$$\tilde{\mathbf{R}} = (\mathbf{R}^{-1} + s \mathbf{Q})^{-1} = \left( \mathbf{R}^{-1}(\mathbf{I} + s \mathbf{R} \mathbf{Q}) \right)^{-1} = (\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} \mathbf{R}$$
$$\implies \tilde{\mathbf{R}} - \mathbf{R} = (\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} \mathbf{R} - \mathbf{R} = \left[ (\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} - \mathbf{I} \right] \mathbf{R}$$

Substitute this back into $K$:
$$K = \boldsymbol{\mu}^H \mathbf{R}^{-1} \left( \left[ (\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} - \mathbf{I} \right] \mathbf{R} \right) \mathbf{R}^{-1} \boldsymbol{\mu}$$

Using $\mathbf{R} \mathbf{R}^{-1} = \mathbf{I}$:
$$K = \boldsymbol{\mu}^H \mathbf{R}^{-1} \left[ (\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} - \mathbf{I} \right] \boldsymbol{\mu}$$

#### Step 5: Evaluate the integral
Now, substitute the completed square exponent back into the multidimensional integral:
$$\mathbb{E}[e^{-s D}] = \frac{1}{\pi^N \det(\mathbf{R})} \int_{\mathbb{C}^N} \exp\left( - (\mathbf{v} - \tilde{\boldsymbol{\mu}})^H \tilde{\mathbf{R}}^{-1} (\mathbf{v} - \tilde{\boldsymbol{\mu}}) + K \right) d\mathbf{v}$$

Since $K$ does not contain the integration variable $\mathbf{v}$, we can factor out $e^K$:
$$\mathbb{E}[e^{-s D}] = \frac{e^K}{\pi^N \det(\mathbf{R})} \int_{\mathbb{C}^N} \exp\left( - (\mathbf{v} - \tilde{\boldsymbol{\mu}})^H \tilde{\mathbf{R}}^{-1} (\mathbf{v} - \tilde{\boldsymbol{\mu}}) \right) d\mathbf{v}$$

The integral is now simply the integral of the core exponential term of a complex Gaussian distribution with covariance $\tilde{\mathbf{R}}$. Since a probability density function must integrate to 1:
$$\int_{\mathbb{C}^N} \exp\left( - (\mathbf{v} - \tilde{\boldsymbol{\mu}})^H \tilde{\mathbf{R}}^{-1} (\mathbf{v} - \tilde{\boldsymbol{\mu}}) \right) d\mathbf{v} = \pi^N \det(\tilde{\mathbf{R}})$$

Substituting this back yields:
$$\mathbb{E}[e^{-s D}] = \frac{e^K}{\pi^N \det(\mathbf{R})} \left( \pi^N \det(\tilde{\mathbf{R}}) \right) = \frac{\det(\tilde{\mathbf{R}})}{\det(\mathbf{R})} e^K$$


#### Step 6: Simplify the determinant ratio
$$\frac{\det(\tilde{\mathbf{R}})}{\det(\mathbf{R})} = \frac{1}{\det(\mathbf{R}) \det(\tilde{\mathbf{R}}^{-1})} = \frac{1}{\det(\mathbf{R} (\mathbf{R}^{-1} + s \mathbf{Q}))} = \frac{1}{\det(\mathbf{I} + s \mathbf{R} \mathbf{Q})}$$


#### Step 7: Final Result
Putting the determinant ratio and $e^K$ together:
$$\mathbb{E}[e^{-s D}] = \frac{\exp\left( \boldsymbol{\mu}^H \mathbf{R}^{-1} \left[ (\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} - \mathbf{I} \right] \boldsymbol{\mu} \right)}{\det(\mathbf{I} + s \mathbf{R} \mathbf{Q})}$$

**End explain

---


Evaluating the matrix terms:
1. $\mathbf{I} + s \mathbf{R} \mathbf{Q} = \begin{bmatrix} 1 & s \frac{\sigma_0^2}{2} \\ s \frac{\sigma_0^2}{2} & 1 \end{bmatrix} \implies \det(\mathbf{I} + s \mathbf{R} \mathbf{Q}) = 1 - s^2 \frac{\sigma_0^4}{4}$
2. The inverse term simplifies to:
   $$\boldsymbol{\mu}^H \mathbf{R}^{-1} [(\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} - \mathbf{I}] \boldsymbol{\mu} = \frac{|h|^2 s}{1 + s \frac{\sigma_0^2}{2}}$$
(Note again: 
$$\mathbf{v} = \begin{bmatrix} X \\ Y \end{bmatrix} \sim \mathcal{CN}(\boldsymbol{\mu}, \mathbf{R}), \quad \boldsymbol{\mu} = \begin{bmatrix} h \\ h \end{bmatrix}, \quad \mathbf{R} = \sigma_0^2 \mathbf{I}_2, \quad \mathbf{Q} = \frac{1}{2} \begin{bmatrix} 0 & 1 \\ 1 & 0 \end{bmatrix}$$
$$(\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} = \frac{1}{1 - s^2 \frac{\sigma_0^4}{4}} \begin{bmatrix} 1 & -s \frac{\sigma_0^2}{2} \\ -s \frac{\sigma_0^2}{2} & 1 \end{bmatrix}$$
)

This yields the MGF:
$$\phi_D(s) = \frac{1}{\left(1 - s \frac{\sigma_0^2}{2}\right)\left(1 + s \frac{\sigma_0^2}{2}\right)} \exp\left( -\frac{|h|^2 s}{1 + s \frac{\sigma_0^2}{2}} \right)$$

### C. Residue Integration
The probability of error is evaluated using the bilateral Laplace transform inversion:
$$P_e = P(D < 0) = \frac{1}{2\pi j} \int_{c - j\infty}^{c + j\infty} \frac{\phi_D(s)}{s} ds$$
where $c$ is a real constant satisfying $0 < c < \frac{2}{\sigma_0^2}$. 

---
**Explain**
The formula is the **inverse Laplace transform** (specifically, the bilateral Laplace inversion formula) applied to calculate a cumulative tail probability.

Here is the explanation of why this formula is used, what $c \pm j\infty$ means, and how we choose the value of $c$:

#### 1. Why do we have this $P_e$ formula?
We want to find the probability that the decision variable $D$ is negative, i.e., the CDF of $D$ evaluated at zero:
$$P_e = P(D < 0) = \int_{-\infty}^{0} f_D(x) \, dx$$

In probability and systems theory:
*   The MGF defined as $\phi_D(s) = \mathbb{E}[e^{-sD}]$ is the **bilateral Laplace transform** of the PDF $f_D(x)$.
*   Integrating a function in the time/domain (to find the area from $-\infty$ to $0$) corresponds to **dividing by $s$** in the Laplace domain.
*   Therefore, the bilateral Laplace transform of the left-tail probability cumulative function is $\frac{\phi_D(s)}{s}$.

To recover the probability $P(D < 0)$, we perform the **inverse Laplace transform** of $\frac{\phi_D(s)}{s}$ evaluated at $x = 0$, which yields the integral:
$$P(D < 0) = \frac{1}{2\pi j} \int_{c-j\infty}^{c+j\infty} \frac{\phi_D(s)}{s} \, ds$$


#### 2. Why $c \pm j\infty$? (The Bromwich Contour)
The path of integration from $c - j\infty$ to $c + j\infty$ is a vertical line in the complex plane ($s$-plane) at a constant real part $\text{Re}\{s\} = c$. This is known as the **Bromwich path**. 

It converts the complex Laplace integral back into a standard line integral along the imaginary direction (where $s = c + j\omega$ and $\omega$ goes from $-\infty$ to $+\infty$). 


#### 3. Why must $c$ satisfy $0 < c < \frac{2}{\sigma_0^2}$?
For the inverse Laplace integral to converge and represent the correct function, the vertical line $s = c$ must lie within the **Region of Convergence (ROC)** of the Laplace transform of the tail probability. The ROC is bounded by the poles of the integrand $\frac{\phi_D(s)}{s}$:

1.  **Poles of the integrand:**
    *   There is a pole at **$s = 0$** (coming from the $s$ in the denominator of $\frac{\phi_D(s)}{s}$).
    *   There is a pole on the positive real axis at **$s = \frac{2}{\sigma_0^2}$** (coming from the term $1 - s\frac{\sigma_0^2}{2}$ in the denominator of the MGF).
    *   There is a pole on the negative real axis at **$s = -\frac{2}{\sigma_0^2}$** (coming from the term $1 + s\frac{\sigma_0^2}{2}$).

2.  **Determining the strip of convergence:**
    To compute the left-tail probability ($D < 0$), the region of convergence for the step-like integration requires the path to lie to the right of the pole at $s = 0$ (so $c > 0$) but to the left of the rightmost pole of the MGF (so $c < \frac{2}{\sigma_0^2}$).

Thus, the vertical line must be placed in the vertical strip:
$$0 < c < \frac{2}{\sigma_0^2}$$

Once the path is placed inside this strip, we can close the integration contour in the Right Half-Plane (RHP). Using the **Residue Theorem**, the integral simplifies to finding the residue of the single enclosed pole in the RHP, which is at $s = \frac{2}{\sigma_0^2}$.

---

Here is the exact mathematical proof that shows how the **probability tail** ($D < 0$ vs. $D > 0$) determines the **side of the pole** ($c > 0$ vs. $c < 0$) in the Laplace domain.


##### Step 1: The Left-Tail Probability $P(D < y)$

Let $f_D(x)$ be the probability density function (PDF) of $D$. We want to find the cumulative distribution function (CDF), which represents the left tail:
$$F_D(y) = P(D < y) = \int_{-\infty}^{y} f_D(x) \, dx$$

Let's find the bilateral Laplace transform of $F_D(y)$ by definition:
$$\mathcal{L}\{F_D(y)\} = \int_{-\infty}^{\infty} F_D(y) e^{-sy} \, dy = \int_{-\infty}^{\infty} \left( \int_{-\infty}^{y} f_D(x) \, dx \right) e^{-sy} \, dy$$

To solve this, we swap the order of integration. The region of integration is $-\infty < x < y < \infty$. Therefore, we can rewrite the double integral as:
$$\mathcal{L}\{F_D(y)\} = \int_{-\infty}^{\infty} f_D(x) \left( \int_{x}^{\infty} e^{-sy} \, dy \right) dx$$

Let's evaluate the inner integral over $y$:
$$\int_{x}^{\infty} e^{-sy} \, dy = \left. \frac{e^{-sy}}{-s} \right|_{x}^{\infty}$$

*   For this to converge at the upper limit ($y \to \infty$), the exponent must decay to zero. This requires the real part of $s$ to be positive: **$\text{Re}\{s\} > 0$**.
*   Under this condition, the integral is:
    $$\left. \frac{e^{-sy}}{-s} \right|_{x}^{\infty} = 0 - \left( -\frac{e^{-sx}}{s} \right) = \frac{e^{-sx}}{s}$$

Substitute this back into the outer integral:
$$\mathcal{L}\{F_D(y)\} = \int_{-\infty}^{\infty} f_D(x) \frac{e^{-sx}}{s} \, dx = \frac{1}{s} \int_{-\infty}^{\infty} f_D(x) e^{-sx} \, dx = \frac{\phi_D(s)}{s}$$

##### Conclusion for the Left Tail:
The Laplace transform of the left tail is $\frac{\phi_D(s)}{s}$ with the convergence condition (ROC):
$$\mathbf{\text{Re}\{s\} > 0 \implies c > 0}$$

##### Step 2: The Right-Tail Probability $P(D > y)$

Now, let's look at the complementary CDF, which represents the right tail:
$$R_D(y) = P(D > y) = \int_{y}^{\infty} f_D(x) \, dx$$

Let's find the bilateral Laplace transform of $R_D(y)$:
$$\mathcal{L}\{R_D(y)\} = \int_{-\infty}^{\infty} R_D(y) e^{-sy} \, dy = \int_{-\infty}^{\infty} \left( \int_{y}^{\infty} f_D(x) \, dx \right) e^{-sy} \, dy$$

We swap the order of integration again. The region of integration is $-\infty < y < x < \infty$. We rewrite it as:
$$\mathcal{L}\{R_D(y)\} = \int_{-\infty}^{\infty} f_D(x) \left( \int_{-\infty}^{x} e^{-sy} \, dy \right) dx$$

Let's evaluate the inner integral over $y$:
$$\int_{-\infty}^{x} e^{-sy} \, dy = \left. \frac{e^{-sy}}{-s} \right|_{-\infty}^{x}$$

*   For this to converge at the lower limit ($y \to -\infty$), the exponent must decay to zero. Since $y$ is negative, this requires the real part of $s$ to be negative: **$\text{Re}\{s\} < 0$**.
*   Under this condition, the integral is:
    $$\left. \frac{e^{-sy}}{-s} \right|_{-\infty}^{x} = -\frac{e^{-sx}}{s} - 0 = -\frac{e^{-sx}}{s}$$

Substitute this back into the outer integral:
$$\mathcal{L}\{R_D(y)\} = \int_{-\infty}^{\infty} f_D(x) \left( -\frac{e^{-sx}}{s} \right) \, dx = -\frac{1}{s} \int_{-\infty}^{\infty} f_D(x) e^{-sx} \, dx = -\frac{\phi_D(s)}{s}$$

##### Conclusion for the Right Tail:
The Laplace transform of the right tail is $-\frac{\phi_D(s)}{s}$ with the convergence condition (ROC):
$$\mathbf{\text{Re}\{s\} < 0 \implies c < 0}$$

##### Summary

| Probability of Interest | Laplace Transform | ROC Condition (Pole at $s=0$) |
| :--- | :--- | :--- |
| **Left Tail ($D < 0$)** | $+\frac{\phi_D(s)}{s}$ | **Right side of pole ($c > 0$)** |
| **Right Tail ($D > 0$)** | $-\frac{\phi_D(s)}{s}$ | **Left side of pole ($c < 0$)** |

This is why, to calculate $P(D < 0)$, we must place our integration line to the right of the pole $s = 0$ ($c > 0$).

**Notes:** This is correct for every case
Because every probability distribution starts at $-\infty$ and ends at $+\infty$:
*   Calculating the **left tail** ($D < \text{threshold}$) always translates to integrating over a right-sided region, which always places the ROC to the **right of $s = 0$** ($c > 0$).
*   Calculating the **right tail** ($D > \text{threshold}$) always translates to integrating over a left-sided region, which always places the ROC to the **left of $s = 0$** ($c < 0$).


**End explain 

---

# 4. (continue section 4 after Explain)
We close the integration contour in the Right Half-Plane (RHP) where the integrand has a single simple pole at $s = \frac{2}{\sigma_0^2}$. Applying the Residue Theorem:
$$P_e = -\text{Res}\left( \frac{\phi_D(s)}{s}, \, s = \frac{2}{\sigma_0^2} \right)$$

Let $a = \sigma_0^2 / 2$. The pole is at $s = 1/a$. We evaluate the residue:
$$\text{Res}\left( \frac{\phi_D(s)}{s}, \, s = \frac{1}{a} \right) = \lim_{s \to 1/a} \left(s - \frac{1}{a}\right) \frac{\exp\left(-\frac{|h|^2 s}{1 + s a}\right)}{s (1 - s a)(1 + s a)}$$
$$\text{Res}\left( \frac{\phi_D(s)}{s}, \, s = \frac{1}{a} \right) = \lim_{s \to 1/a} \left(s - \frac{1}{a}\right) \frac{\exp\left(-\frac{|h|^2 s}{1 + s a}\right)}{s (-a)\left(s - \frac{1}{a}\right)(1 + s a)} = \frac{\exp\left(-\frac{|h|^2 / a}{1 + 1}\right)}{(1/a)(-a)(1 + 1)}$$
$$\text{Res}\left( \frac{\phi_D(s)}{s}, \, s = \frac{1}{a} \right) = \frac{\exp\left(-\frac{|h|^2}{2a}\right)}{-2} = -\frac{1}{2} \exp\left(-\frac{|h|^2}{\sigma_0^2}\right)$$

Substituting the SNR $\gamma = |h|^2 / \sigma_0^2 = N_D P_D |h|^2 / \sigma_U^2$, we obtain:
$$P_{e, \text{noisy}}(\gamma) = \frac{1}{2} \exp(-\gamma)$$

---

**Explain**
##### 1. What is the `Res()` function? (The Residue)
A **pole** is a point where a complex function goes to infinity (like dividing by zero). 
The **Residue**, denoted as $\text{Res}(f(s), s = s_0)$, is a measure of "how fast" the function blows up at that pole.

For a simple pole (a pole of order 1, meaning it is just a term like $\frac{1}{s-s_0}$), the residue can be calculated easily using a limit:
$$\text{Res}(f(s), s = s_0) = \lim_{s \to s_0} (s - s_0) f(s)$$
Multiplying by $(s - s_0)$ cancels out the "division by zero" term, allowing us to evaluate the rest of the function at $s_0$.

---

##### 2. The Residue Theorem (The Integration Shortcut)
The Residue Theorem states that if you integrate a complex function along a **closed loop (contour)**, the integral is simply equal to $2\pi j$ times the sum of the residues of the poles trapped inside the loop:
$$\oint_{\text{loop}} f(s) \, ds = 2\pi j \sum \text{Res}(f(s), \text{poles inside})$$

---

##### 3. How we use it to calculate $P_e$ from line 219

Our line integral goes along the vertical line $\text{Re}\{s\} = c$:
$$P_e = \frac{1}{2\pi j} \int_{c-j\infty}^{c+j\infty} \frac{\phi_D(s)}{s} \, ds$$

To use the Residue Theorem, we must turn this straight line into a closed loop. We do this by adding a giant semi-circle of radius $R \to \infty$ in the **Right Half-Plane (RHP)**:

```
          s-plane (imaginary axis)
             |
             |       Giant closed loop 
      c+jR --+=======)
             |       ║
             |       ║  Pole at s = 2 / σ0^2
             |   c   ║     x
      -------+---+---║-----+-------> real axis
             |   |   ║
             |   |   ║
      c-jR --+=======)
             |
```

1.  **Integral along the semi-circle is 0:**
    Because the MGF $\phi_D(s)$ decays exponentially in the Right Half-Plane, as we make the semi-circle infinitely large ($R \to \infty$), the integral along the curved part goes to exactly $0$.
2.  **The closed loop equals the line integral:**
    Therefore, the integral along the closed loop is exactly equal to our line integral.
3.  **Clockwise Integration (The Minus Sign):**
    When we close the loop in the Right Half-Plane, we are integrating in a **clockwise** direction. By mathematical convention, the Residue Theorem assumes counter-clockwise integration. Integrating clockwise introduces a **negative sign**:
    $$\int_{c-j\infty}^{c+j\infty} \frac{\phi_D(s)}{s} \, ds = -2\pi j \sum \text{Res}\left( \frac{\phi_D(s)}{s}, \text{RHP poles} \right)$$
4.  **Substitute into the $P_e$ formula:**
    $$P_e = \frac{1}{2\pi j} \left( -2\pi j \sum \text{Res}\left( \frac{\phi_D(s)}{s}, \text{RHP poles} \right) \right) = -\sum \text{Res}\left( \frac{\phi_D(s)}{s}, \text{RHP poles} \right)$$

---

##### 4. Evaluating the final step (Line 338-339)

Inside our closed loop in the Right Half-Plane ($\text{Re}\{s\} > c > 0$), there is only **one** pole: the pole at $s = \frac{2}{\sigma_0^2}$ (which we denote as $s = 1/a$, where $a = \sigma_0^2/2$).

So the sum of residues has only one term:
$$P_e = -\text{Res}\left( \frac{\phi_D(s)}{s}, \, s = \frac{1}{a} \right)$$

In line 338, we calculate this residue using the limit formula:
$$\text{Res} = \lim_{s \to 1/a} \left(s - \frac{1}{a}\right) \frac{\exp\left(-\frac{|h|^2 s}{1 + s a}\right)}{s (1 - s a)(1 + s a)}$$

*   We rewrite the term $(1 - sa)$ in the denominator as $-a(s - 1/a)$.
*   This allows us to cancel the $(s - 1/a)$ term in both the numerator and the denominator, resolving the "division by zero" division:
    $$\text{Res} = \lim_{s \to 1/a} \cancel{\left(s - \frac{1}{a}\right)} \frac{\exp\left(-\frac{|h|^2 s}{1 + s a}\right)}{s (-a)\cancel{\left(s - \frac{1}{a}\right)}(1 + s a)}$$
*   Now we can safely substitute $s = 1/a$ into the remaining terms:
    $$\text{Res} = \frac{\exp\left(-\frac{|h|^2 / a}{1 + 1}\right)}{(1/a)(-a)(1 + 1)} = \frac{\exp\left(-\frac{|h|^2}{2a}\right)}{-2} = -\frac{1}{2} \exp\left(-\frac{|h|^2}{\sigma_0^2}\right)$$

Finally, substituting this back into $P_e = -\text{Res}$ gives:
$$P_e = -\left( -\frac{1}{2} \exp\left(-\frac{|h|^2}{\sigma_0^2}\right) \right) = \frac{1}{2} \exp\left(-\frac{|h|^2}{\sigma_0^2}\right)$$

This is how we get the final Bit Error Rate without ever having to solve a difficult complex integration!

---

**The Residue Theorem only works for a closed loop**. It cannot be used directly on an open line.

To get around this, we use a limiting process where we **force** the open line to become a closed loop by "completing the contour at infinity."

Here is the exact mathematical step of how we do this:

##### Step 1: Define a Closed Loop of Finite Radius $R$

Instead of integrating to infinity immediately, we define a closed loop $C$ with a finite radius $R$ (where $R$ is large enough to enclose our pole at $s = 1/a$).

This closed loop $C$ consists of two distinct segments:
1.  **The Straight Line Segment ($I_{\text{line}}$):** A straight line from $c - jR$ to $c + jR$.
2.  **The Semi-Circular Arc ($I_{\text{arc}}$):** A giant circular arc of radius $R$ in the Right Half-Plane that loops from $c + jR$ back to $c - jR$.

```
          s-plane
             |
             |       Arc of radius R
      c+jR --+-------*
             |        \
             |         \
             |   c      *  Pole (1/a)
      -------+---+------|------> real axis
             |   |      *
             |   |     /
      c-jR --+-------*
             |
```

Since this is a closed loop, the Residue Theorem **does** apply:
$$\oint_{C} f(s) \, ds = 2\pi j \sum \text{Res}(f(s))$$

Since the loop is made of the line and the arc, we can split the integral:
$$\int_{c-jR}^{c+jR} f(s) \, ds + \int_{\text{arc}} f(s) \, ds = -2\pi j \cdot \text{Res}\left(f(s), s=\frac{1}{a}\right)$$
*(The minus sign is because we are traversing the loop clockwise).*

##### Step 2: Take the limit as $R \to \infty$

Now, we take the limit of both sides as the radius $R$ goes to infinity:
$$\lim_{R \to \infty} \left[ \int_{c-jR}^{c+jR} f(s) \, ds + \int_{\text{arc}} f(s) \, ds \right] = -2\pi j \cdot \text{Res}\left(f(s), s=\frac{1}{a}\right)$$
$$\int_{c-j\infty}^{c+j\infty} f(s) \, ds + \lim_{R \to \infty} \left[ \int_{\text{arc}} f(s) \, ds \right] = -2\pi j \cdot \text{Res}\left(f(s), s=\frac{1}{a}\right)$$


##### Step 3: Show that the Arc Integral vanishes to $0$

As $R \to \infty$, the arc gets pushed out to infinity in the Right Half-Plane (where the real part of $s$ becomes extremely large and positive). 

Because our MGF $\phi_D(s)$ contains the term $\exp\left(-\frac{|h|^2 s}{1 + sa}\right)$, as the real part of $s$ grows, the value of the function decays exponentially to zero. 

According to a standard theorem in complex analysis (Jordan's Lemma), if a function decays fast enough, the integral over the circular arc at infinity is exactly zero:
$$\lim_{R \to \infty} \left[ \int_{\text{arc}} f(s) \, ds \right] = 0$$


##### Step 4: The Final Equivalence

Since the arc term disappears, we are left with:
$$\int_{c-j\infty}^{c+j\infty} f(s) \, ds + 0 = -2\pi j \cdot \text{Res}\left(f(s), s=\frac{1}{a}\right)$$
$$\implies \frac{1}{2\pi j} \int_{c-j\infty}^{c+j\infty} f(s) \, ds = -\text{Res}\left(f(s), s=\frac{1}{a}\right)$$

So, we are not "assuming" it is a closed loop of infinite radius; we are **rigorously proving** that the limit of the closed loop integral is mathematically identical to the open line integral. This is why we are allowed to use the Residue Theorem to evaluate the open line integral!



**End explain 

---

## 5. Comparison: Perfect CSI vs. Noisy Channel Estimation

Comparing the two analytical formulas for BPSK in an AWGN channel:
* **Perfect CSI:** $P_{e, \text{perfect}}(\gamma) = Q(\sqrt{2\gamma}) \approx \frac{1}{2\sqrt{\pi\gamma}} e^{-\gamma}$
* **Noisy Channel Estimation:** $P_{e, \text{noisy}}(\gamma) = \frac{1}{2} e^{-\gamma}$

Both curves decay exponentially with the same dominant exponential term ($e^{-\gamma}$). However, the noisy channel estimate introduces a pre-exponential penalty. The ratio of the error probabilities at high SNR is:
$$\frac{P_{e, \text{noisy}}(\gamma)}{P_{e, \text{perfect}}(\gamma)} \approx \sqrt{\pi\gamma}$$

### The SNR Shift vs. BER Discrepancy
Because this ratio grows with $\gamma$, there is indeed a **large vertical discrepancy** in the absolute BER values between the two curves:
* At $\gamma = 10$ ($10$ dB), the ratio is $\sqrt{10\pi} \approx 5.6$. The BER with noisy channel estimation is nearly **6 times larger** than the perfect CSI case ($2.27 \times 10^{-5}$ vs $3.87 \times 10^{-6}$).

However, in terms of **horizontal SNR shift** (the additional SNR in dB required by the noisy channel estimate to achieve the same target BER), the penalty actually **decreases** as SNR increases:
* **Low SNR (Target BER $10^{-1}$):** SNR shift is **$2.92$ dB** (close to the 3 dB mark).
* **Moderate SNR (Target BER $10^{-3}$):** SNR shift is **$1.14$ dB**.
* **High SNR (Target BER $10^{-5}$):** SNR shift is **$0.75$ dB**.
* **Very High SNR (Target BER $10^{-9}$):** SNR shift is **$0.47$ dB**.

As $\gamma \to \infty$, the SNR shift in dB approaches **0 dB**. 

> [!NOTE]
> The common literature claim that noisy channel estimation introduces a constant **3 dB SNR penalty** is a simplification based on the **Gaussian approximation** of the decision variable (which assumes the noise variance simply doubles, yielding $Q(\sqrt{\gamma})$). This approximation is highly pessimistic at high SNR because the decision variable (the product of two Gaussian variables) is not Gaussian. 


