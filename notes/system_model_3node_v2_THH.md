# System Model: Three-Node NTN-ISAC System (Single-Code PMCW)

> *Transmit and received signal mathematics for a single-waveform (single-code) PMCW joint radar–communication design.*

The three-node system consists of:

- **Drone (D):** A mid-air platform with one transmit and one receive antenna. It acts simultaneously as the Joint Radar–Communication (JRC) transmitter, a communication access point for the UE, and a monostatic radar for sensing the moving object.
- **Ground User Equipment (UE):** A single-antenna mobile receiver that receives the downlink communication signal from the drone, and whose position and velocity are estimated by the drone.
- **Moving Object (T):** A passive aerial target (e.g., a drone or UAV) detected and tracked through radar backscatter.

The drone transmits a **single-code PMCW waveform** shared by both the communication and the radar-sensing functions (see [Section 3](#3-transmit-signal)).

---

## 1. Notation and Geometry

Let $d_{DU}$, $d_{DT}$, and $d_{TU}$ denote the instantaneous Euclidean distances between the drone and the UE, between the drone and the object, and between the object and the UE, respectively. The corresponding one-way propagation delays are

$$
\tau_{DU} = \frac{d_{DU}}{c_0}, \qquad \tau_{DT} = \frac{d_{DT}}{c_0}, \qquad \tau_{TU} = \frac{d_{TU}}{c_0},
$$

where $c_0$ is the speed of light. The round-trip monostatic sensing delay for the moving object is

$$
\tau_{\mathrm{mono}} = \frac{2\,d_{DT}}{c_0},
$$

and the total propagation delay for the two-hop scattering path $D\!\to\!T\!\to\!U$ is

$$
\tau_{DTU} = \tau_{DT} + \tau_{TU} = \frac{d_{DT} + d_{TU}}{c_0}.
$$

Let $\mathbf{v}_D$, $\mathbf{v}_U$, and $\mathbf{v}_T$ be the velocity vectors of the drone, UE, and moving object. The radial velocities along each link are projected onto the unit line-of-sight (LOS) vector $\hat{\mathbf{u}}_{ij}$ from node $i$ to node $j$:

$$
v_{r,DU} = \bigl(\mathbf{v}_U - \mathbf{v}_D\bigr) \cdot \hat{\mathbf{u}}_{DU}, \qquad
v_{r,DT} = \bigl(\mathbf{v}_T - \mathbf{v}_D\bigr) \cdot \hat{\mathbf{u}}_{DT}, \qquad
v_{r,TU} = \bigl(\mathbf{v}_U - \mathbf{v}_T\bigr) \cdot \hat{\mathbf{u}}_{TU}.
$$

With drone carrier wavelength $\lambda_D = c_0 / f_{c,D}$, the resulting Doppler shifts are

$$
f_{D,DU} = -\frac{v_{r,DU}}{\lambda_D}, \qquad f_{D,DT} = -\frac{v_{r,DT}}{\lambda_D}, \qquad f_{D,TU} = -\frac{v_{r,TU}}{\lambda_D}.
$$

The two-way monostatic Doppler shift for target sensing is $\nu_{\mathrm{mono}} = 2\,f_{D,DT}$, and the combined Doppler shift accumulated over the two-hop $D\!\to\!T\!\to\!U$ scattering path is

$$
\nu_{DTU} = f_{D,DT} + f_{D,TU}.
$$

---

## 2. Channel Model

All drone-to-node and node-to-drone links are SISO channels. The channel coefficient between nodes $i$ and $j$ is decomposed as

$$
h_{ij} = \sqrt{\beta_{ij}}\,g_{ij},
$$

where $\beta_{ij}$ is the large-scale power gain (path loss and shadowing) and $g_{ij}$ is the complex small-scale fading coefficient. A distance-dependent path-loss model gives

$$
\beta_{ij} = C_{ij}\!\left(\frac{d_{ij}}{d_0}\right)^{-\alpha_{ij}},
$$

with reference channel gain $C_{ij} = G_i G_j \!\left(\lambda_D / (4\pi d_0)\right)^2$ at distance $d_0$, and $\alpha_{ij}$ the path-loss exponent.

The time-varying channels (including the Doppler phase rotation) are modeled as

$$
h_{DU}(t) = \sqrt{\beta_{DU}}\,g_{DU}\,e^{j2\pi f_{D,DU}\,t}, \qquad
h_{DT}(t) = \sqrt{\beta_{DT}}\,g_{DT}\,e^{j2\pi f_{D,DT}\,t}, \qquad
h_{TD}(t) = \sqrt{\beta_{TD}}\,g_{TD}\,e^{j2\pi f_{D,DT}\,t}.
$$

Under channel reciprocity, $\beta_{TD}=\beta_{DT}$ and $g_{TD}=g_{DT}$.

To account for the scattering path $D\!\to\!T\!\to\!U$ (Section 4), an additional channel between the moving object and the UE is introduced:

$$
h_{TU}(t) = \sqrt{\beta_{TU}}\,g_{TU}\,e^{j2\pi f_{D,TU}\,t},
$$

where $\beta_{TU}$ is the large-scale path gain and $g_{TU}\sim\mathcal{CN}(0,1)$ is the small-scale fading of the $T\!\to\!U$ link.

> **$h_{TU}$ is an unknown channel.** The object's position $d_{TU}$, velocity $\mathbf{v}_T$, and scattering pattern toward the UE are all unknown a priori (they are precisely what the radar is trying to estimate), so $\beta_{TU}$, $g_{TU}$, and $f_{D,TU}$ cannot be predicted by either the drone or the UE. The $D\!\to\!T\!\to\!U$ interference therefore cannot be cancelled by pre-coding or Doppler pre-compensation at the transmitter.

---

## 3. Transmit Signal

The drone transmits a Phase-Modulated Continuous Wave (PMCW) waveform built from a single Maximum-Length Sequence (MLS) of length $N_D$, shared by both the communication and the radar-sensing functions.

**Binary chips and data bits.** The raw MLS sequence is generated in binary (logical) form, and the communication payload on pulse $m$ is a raw bit:

$$
c[n] \in \{0,1\},\quad 0 \le n < N_D; \qquad\qquad b_m \in \{0,1\},\quad 0 \le m < M.
$$

**Polar (BPSK) mapping.** Both the chips and the data bit are mapped from the logical alphabet $\{0,1\}$ to the antipodal **polar** alphabet $\{+1,-1\}$ via the BPSK rule $x \mapsto 1 - 2x$:

$$
s_{\mathrm{mls}}[n] = 1 - 2\,c[n] \;\in\; \{+1,-1\},
\qquad
d_m = 1 - 2\,b_m \;\in\; \{+1,-1\}.
$$

In particular $d_m^2 = 1$, a property exploited for radar data stripping (Section 5.1).

**Frame structure (preamble + data).** Each slow-time block $m$ is divided into two consecutive periods, each of $N_D$ chips and duration $T_m = N_D T_c$, giving a total block duration of $T_{\mathrm{block}} = 2\,T_m = 2\,T_{\mathrm{PRI}}$:

| Period | Duration | Content | Purpose |
|---|---|---|---|
| **Sounding (preamble)** | $T_m$ | Pure MLS $s_{\mathrm{mls}}[n]$ (no data modulation, $d_m = +1$) | Channel estimation at UE |
| **Data** | $T_m$ | MLS $\times$ polar data symbol $d_m$ | Communication + sensing |

The continuous-time baseband transmitted signal for block $m$ is therefore:

$$
x_D(t) = \sqrt{P_D}\Bigl[
  \underbrace{s_{\mathrm{mls}}(t - m\,T_{\mathrm{block}})}_{\text{sounding period}}
  + d_m\,\underbrace{s_{\mathrm{mls}}(t - m\,T_{\mathrm{block}} - T_m)}_{\text{data period}}
\Bigr],
$$

where $s_{\mathrm{mls}}(t) = \sum_{n=0}^{N_D-1} s_{\mathrm{mls}}[n]\,p(t - nT_c)$ and the rectangular chip pulse is

$$
p(t) = \begin{cases} 1, & 0 \le t < T_c, \\ 0, & \text{otherwise.} \end{cases}
$$

In discrete time, the two periods are indexed separately:

$$
x_{D,\mathrm{sound}}[n] = \sqrt{P_D}\,s_{\mathrm{mls}}[n], \qquad
x_{D,\mathrm{data}}[n,m] = \sqrt{P_D}\,s_{\mathrm{mls}}[n]\,d_m,
\quad 0 \le n < N_D.
$$

> **Single-code consequence.** Because one code serves both functions, the UE signal and the target echo are **not separable in the code domain**. At the drone receiver they share one matched-filter output and are distinguished only by their **delay and Doppler** — requiring successive interference cancellation (Section 5.1) to keep the strong UE echo from masking the weak target.

---

## 4. Received Signal at the UE

Besides the direct drone-to-UE link, the UE also receives a **scattered contribution** over the two-hop path $D\!\to\!T\!\to\!U$: the transmitted signal hits the moving object and is re-radiated toward the UE, acting as uncontrolled interference. The composite received signal is

$$
y_U[n,m]
= \underbrace{h_{DU}\, x_D\!\bigl[(n - \ell_{DU})_{N_D},\,m\bigr]\,
  e^{j2\pi f_{D,DU}\,m\,T_{\mathrm{PRI}}}}_{ \text{direct } D\to U \text{ link}}
+ \underbrace{y_{DTU}[n,m]}_{\text{scattered } D\to T\to U}
+ n_U[n,m],
$$

where $\ell_{DU} = \mathrm{round}(\tau_{DU}\,f_s)$ is the integer chip delay, $(\cdot)_{N_D}$ denotes modulo-$N_D$, $f_s = 1/T_c$, and $n_U[n,m]\sim\mathcal{CN}(0,\sigma_U^2)$ is AWGN.

The scattered term propagates from the drone to the object (delay $\tau_{DT}$, Doppler $f_{D,DT}$, channel $h_{DT}$), is scattered with complex reflection coefficient $\eta_T$, then propagates from the object to the UE (delay $\tau_{TU}$, Doppler $f_{D,TU}$, channel $h_{TU}$):

$$
y_{DTU}[n,m]
= \eta_T\,\sqrt{\beta_{DT}\,\beta_{TU}}\,g_{DT}\,g_{TU}\,
  \sqrt{P_D}\,
  s_{\mathrm{mls}}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr]\,
  d_m\,e^{j2\pi\nu_{DTU}\,m\,T_{\mathrm{PRI}}},
$$

with $\ell_{DTU} = \mathrm{round}(\tau_{DTU}\,f_s)$ the total two-hop chip delay and $\nu_{DTU} = f_{D,DT} + f_{D,TU}$ the cumulative Doppler. Substituting the transmit signal, the full received signal is

$$
y_U[n,m]
= \underbrace{\sqrt{P_D}\,h_{DU}\,
  s_{\mathrm{mls}}\!\bigl[(n-\ell_{DU})_{N_D}\bigr]\,
  d_m\,e^{j2\pi f_{D,DU}\,m\,T_{\mathrm{PRI}}}}_{\text{desired direct link}}
+ \underbrace{\eta_T\,\sqrt{\beta_{DT}\beta_{TU}}\,g_{DT}\,g_{TU}\,\sqrt{P_D}\,
  s_{\mathrm{mls}}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr]\,
  d_m\,e^{j2\pi\nu_{DTU}\,m\,T_{\mathrm{PRI}}}}_{\text{scattered interference (unknown channel)}}
+ n_U[n,m].
$$

The data $d_m$ is recovered by correlating with $s_{\mathrm{mls}}$. The scattered term carries the **same** code delayed by $\ell_{DTU}$ and Doppler-shifted by $\nu_{DTU}$; since $h_{TU}$ is unknown it cannot be pre-cancelled and is treated as residual structured noise.

### 4.1 Data Demodulation at the UE

The UE samples the received signal at chip rate $f_s = 1/T_c$, yielding $N_D$ samples for the sounding period and $N_D$ samples for the data period within each block $m$.

---

**Step 1 — Delay Estimation and Channel Estimation via Sounding Matched Filter.**

During the sounding period of block $m$, the drone transmits the unmodulated MLS ($d_m = +1$, no data), so the received samples at the UE are

$$
y_{U,\mathrm{sound}}[n,m] = h_{DU}\,\sqrt{P_D}\,s_{\mathrm{mls}}[n - \ell_{DU}]\,e^{j2\pi f_{D,DU}\,m\,T_{\mathrm{block}}} + n_{U,\mathrm{sound}}[n,m],
$$

where $\ell_{DU} = \mathrm{round}(\tau_{DU}\,f_s)$ is the true chip delay and $T_{\mathrm{block}} = 2\,T_{\mathrm{PRI}}$.

Instead of performing frequency-domain deconvolution (least-squares division $\hat{H} = Y / S$), which suffers from severe **noise enhancement** at the DC bin (since $|S_{\mathrm{mls}}[0]|^2 = 1$ is $21\text{ dB}$ weaker than the AC bins where $|S_{\mathrm{mls}}[k_f]|^2 = N_D + 1 = 128$), the UE uses a **Matched Filter (cross-correlation)** in the frequency domain:

$$
R_{\mathrm{sound}}[k_f, m] = Y_{U,\mathrm{sound}}[k_f,m] \cdot S_{\mathrm{mls}}^*[k_f],
$$

where $S_{\mathrm{mls}}[k_f] = \mathrm{DFT}\{s_{\mathrm{mls}}[n]\}$. Converting back to the time domain:

$$
r_{\mathrm{sound}}[n, m] = \mathrm{IDFT}_{k_f}\{ R_{\mathrm{sound}}[k_f, m] \}.
$$

*   **Delay Estimation (Timing Synchronization):** The UE does not know the true delay $\ell_{DU}$ in advance. It estimates it by locating the peak of the average cross-correlation power:
    
    $$
    \hat{\ell}_{DU} = \arg\max_{n} \frac{1}{M}\sum_{m=0}^{M-1} |r_{\mathrm{sound}}[n, m]|^2.
    $$

*   **Channel Estimation:** At the estimated peak $\hat{\ell}_{DU}$, the correlation output represents the coherent sum over the $N_D$ chips:
    
    $$
    r_{\mathrm{sound}}[\hat{\ell}_{DU}, m] \approx N_D \cdot h_{DU} \sqrt{P_D} e^{j2\pi f_{D,DU} m T_{\mathrm{block}}} + \tilde{w}_{\mathrm{sound}}[m].
    $$
    
    Dividing by the code length $N_D$ yields the scalar channel estimate for block $m$ (which naturally captures the path loss, fading, and the slow-time Doppler phase):
    
    $$
    \hat{h}_{DU}[m] = \frac{r_{\mathrm{sound}}[\hat{\ell}_{DU}, m]}{N_D} \approx h_{DU} \sqrt{P_D} e^{j2\pi f_{D,DU} m T_{\mathrm{block}}}.
    $$

**Why Matched Filtering (Conjugate Multiplication) is Preferred over Least-Squares (Division)

If the UE estimates the channel response using frequency-domain division (Least-Squares deconvolution):

$$
\hat{H}_{\mathrm{LS}}[k_f, m] = \frac{Y_{U,\mathrm{sound}}[k_f, m]}{S_{\mathrm{mls}}[k_f]} = H_{DU}[k_f, m] + \frac{W_{\mathrm{sound}}[k_f, m]}{S_{\mathrm{mls}}[k_f]}
$$

Taking the IDFT to return to the time domain yields the estimate $\hat{h}_{\mathrm{LS}}[n, m]$. Let's compare the noise variance in the time-domain peak bin for both estimators:

1.  **Matched Filter Estimator (Conjugate Multiplication):**
    The matched filter output in the time domain is:
    
    $$
    r_{\mathrm{sound}}[n, m] = \sum_{j=0}^{N_D-1} y_{U,\mathrm{sound}}[j, m] \cdot s_{\mathrm{mls}}^*[(j - n)_{N_D}]
    $$
    
    Dividing the peak at the true delay $\ell_{DU}$ by $N_D$ yields the estimate $\hat{h}_{\mathrm{MF}}[m] = h_{DU}\sqrt{P_D}e^{j2\pi f_{D,DU} m T_{\mathrm{block}}} + \tilde{w}_{\mathrm{MF}}[m]$. Since the time-domain noise samples $n_U[n]$ are i.i.d. with variance $\sigma_U^2$, and $|s_{\mathrm{mls}}[j]|^2 = 1$, the noise variance of the matched-filter estimate is:
    
    $$
    \text{Var}\left(\tilde{w}_{\mathrm{MF}}[m]\right) = \text{Var}\left( \frac{1}{N_D} \sum_{j=0}^{N_D-1} n_U[j] s_{\mathrm{mls}}^*[j-\ell_{DU}] \right) = \frac{1}{N_D^2} \sum_{j=0}^{N_D-1} \sigma_U^2 = \frac{\sigma_U^2}{N_D}
    $$

2.  **Least-Squares Estimator (Division):**
    For the LS deconvolution, the frequency-domain noise term is $W_{\mathrm{sound}}[k_f] / S_{\mathrm{mls}}[k_f]$. By Parseval's theorem, the noise variance in the time domain is the average of the frequency-domain variances:
    
    $$
    \text{Var}\left(\tilde{w}_{\mathrm{LS}}[m]\right) = \frac{1}{N_D^2} \sum_{k_f=0}^{N_D-1} \frac{\text{Var}(W_{\mathrm{sound}}[k_f, m])}{|S_{\mathrm{mls}}[k_f]|^2} = \frac{1}{N_D^2} \left[ \frac{N_D \sigma_U^2}{1} + (N_D-1)\frac{N_D\sigma_U^2}{N_D+1} \right] \approx \frac{2\sigma_U^2}{N_D}
    $$

This shows that the Least-Squares deconvolution (division) estimator has **twice the noise variance** (a $3\text{ dB}$ SNR penalty) compared to the Matched Filter estimator, because division amplifies the noise at the weak DC frequency bin where $|S_{\mathrm{mls}}[0]|^2 = 1 \ll 128$. For this reason, matched filtering is mathematically the optimal SNR estimator.

---

**Step 2 — Matched Filtering of the Data Period.**

During the data period of block $m$, the received signal (ignoring the scattered interference for clarity) is

$$
y_{U,\mathrm{data}}[n,m] = h_{DU}\,\sqrt{P_D}\,s_{\mathrm{mls}}[n-\ell_{DU}]\,d_m\,e^{j2\pi f_{D,DU}\,m\,T_{\mathrm{block}}} + n_{U,\mathrm{data}}[n,m].
$$

The UE correlates the received data signal with the reference code $S_{\mathrm{mls}}$ to compress the energy:

$$
r_{\mathrm{data}}[n, m] = \mathrm{IDFT}_{k_f}\left\{ Y_{U,\mathrm{data}}[k_f, m] \cdot S_{\mathrm{mls}}^*[k_f] \right\}.
$$

Slicing this matrix at the estimated delay bin $\hat{\ell}_{DU}$ yields the despread peak:

$$
z_{\mathrm{mf}}[m] = r_{\mathrm{data}}[\hat{\ell}_{DU}, m] \approx N_D \cdot h_{DU} \sqrt{P_D} d_m e^{j2\pi f_{D,DU} m T_{\mathrm{block}}} + \tilde{w}_{\mathrm{data}}[m].
$$

---

**Step 3 — Scalar Channel Equalization.**

To cancel the channel gain $h_{DU}$, the transmit power scaling, and the slow-time Doppler phase rotation simultaneously, the despread data peak is divided by the channel estimate $\hat{h}_{DU}[m]$:

$$
z_{\mathrm{eq}}[m] = \frac{z_{\mathrm{mf}}[m]}{\hat{h}_{DU}[m]} \approx N_D \cdot d_m + \tilde{w}_{\mathrm{eq}}[m].
$$

---

**Step 4 — BPSK Symbol Decision.**

The polar symbol estimate is obtained by slicing the real part of the equalized peak:

$$
\hat{d}_m = \mathrm{sign}\!\left(\mathrm{Re}\!\left\{ z_{\mathrm{eq}}[m] \right\}\right) \in \{+1,-1\}.
$$

The decoded binary bit is then recovered via the inverse BPSK map:

$$
\hat{b}_m = \frac{1 - \hat{d}_m}{2} \in \{0,1\}.
$$

The resulting **post-equalization SINR** at the UE is

$$
\mathrm{SINR}_U
= \frac{P_D\,\beta_{DU}\,|g_{DU}|^2}
       {P_D\,|\eta_T|^2\,\beta_{DT}\,\beta_{TU}\,|g_{DT}|^2\,|g_{TU}|^2\,|\rho(\ell_{DTU})|^2
       + \dfrac{\sigma_U^2}{N_D}},
$$

where $\rho(\ell) = \frac{1}{N_D}\sum_{n} s_{\mathrm{mls}}[(n-\ell)_{N_D}]\,s_{\mathrm{mls}}[n]$ is the MLS autocorrelation ($|\rho(\ell)| = 1/N_D$ for $\ell \ne 0$).

- **Numerator:** desired despread signal power.
- **First denominator term:** residual scattered $D\!\to\!T\!\to\!U$ interference, suppressed by $|\rho|^2 \approx 1/N_D^2$ for $\ell_{DTU} \ne \ell_{DU}$. If $\ell_{DTU} = \ell_{DU}$ (target very close to drone), $\rho(0)=1$ and the term becomes a co-delay co-Doppler jammer — the worst case, not mitigatable by despreading alone.
- **Second denominator term:** thermal noise, reduced by the processing gain $N_D$ through coherent despreading.

### 4.2 Analytical Derivation of the Theoretical BER (MGF Inversion Method)

To compute the theoretical BER of BPSK under noisy pilot-based channel estimation, we analyze the decision rule from Step 4. Let the despread data peak and the channel estimate be normalized as:

$$
X = \frac{z_{\mathrm{mf}}[m]}{N_D} = S d_m + w_{\mathrm{data}}[m]
$$

$$
Y = \hat{h}_{DU}[m] = S + w_{\mathrm{sound}}[m]
$$

where:
- $S = h_{DU} \sqrt{P_D}$ is the complex channel gain.
- $w_{\mathrm{data}}[m], w_{\mathrm{sound}}[m] \sim \mathcal{CN}\left(0, \sigma_0^2\right)$ are independent complex Gaussian noises with variance $\sigma_0^2 = \sigma_U^2 / N_D$.

Assuming $d_m = +1$ is transmitted, an error occurs if $\mathrm{Re}\{z_{\mathrm{eq}}[m]\} < 0$, which is equivalent to the real part of the product of two complex Gaussians being negative:

$$
P_e = P\left(\mathrm{Re}\left\{X Y^*\right\} < 0\right)
$$

We define the decision statistic $D = \mathrm{Re}\{X Y^*\} = \frac{1}{2}(X Y^* + X^* Y)$, which can be expressed in Hermitian quadratic form as:

$$
D = \mathbf{v}^H \mathbf{Q} \mathbf{v}
$$

where:

$$
\mathbf{v} = \begin{bmatrix} X \\ Y \end{bmatrix} \sim \mathcal{CN}(\boldsymbol{\mu}, \mathbf{R}), \quad \boldsymbol{\mu} = \begin{bmatrix} S \\ S \end{bmatrix}, \quad \mathbf{R} = \sigma_0^2 \mathbf{I}_2, \quad \mathbf{Q} = \frac{1}{2} \begin{bmatrix} 0 & 1 \\ 1 & 0 \end{bmatrix}
$$

The Moment Generating Function (MGF) of $D$ is:

$$
\phi_D(s) = E[e^{-s D}] = \frac{\exp\left( -\boldsymbol{\mu}^H \mathbf{R}^{-1} [(\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} - \mathbf{I}] \boldsymbol{\mu} \right)}{\det(\mathbf{I} + s \mathbf{R} \mathbf{Q})}
$$

Substituting $\mathbf{R}\mathbf{Q} = \frac{\sigma_0^2}{2}\begin{bmatrix} 0 & 1 \\ 1 & 0 \end{bmatrix}$ yields:
1. $\det(\mathbf{I} + s \mathbf{R} \mathbf{Q}) = 1 - s^2 \frac{\sigma_0^4}{4}$
2. $\boldsymbol{\mu}^H \mathbf{R}^{-1} [(\mathbf{I} + s \mathbf{R} \mathbf{Q})^{-1} - \mathbf{I}] \boldsymbol{\mu} = \frac{|S|^2 s}{1 + s \frac{\sigma_0^2}{2}}$

This simplifies the MGF to:

$$
\phi_D(s) = \frac{1}{\left(1 - s \frac{\sigma_0^2}{2}\right)\left(1 + s \frac{\sigma_0^2}{2}\right)} \exp\left( -\frac{|S|^2 s}{1 + s \frac{\sigma_0^2}{2}} \right)
$$

The error probability is evaluated using bilateral Laplace inversion:

$$
P_e = \frac{1}{2\pi j} \int_{c - j\infty}^{c + j\infty} \frac{\phi_D(s)}{s} ds
$$

where $0 < c < 2/\sigma_0^2$. Closing the contour in the right half-plane (RHP), the integrand is analytic except for a simple pole at $s = 2/\sigma_0^2$ (since the essential singularity is in the left half-plane at $s = -2/\sigma_0^2$).

Applying the Residue Theorem:

$$
P_e = -\text{Res}\left( \frac{\phi_D(s)}{s}, \, s = \frac{2}{\sigma_0^2} \right) = \frac{1}{2} \exp\left( -\frac{|S|^2}{\sigma_0^2} \right) = \frac{1}{2} \exp\left( -\gamma_{\mathrm{eff}} \right)
$$

where the effective SNR is $\gamma_{\mathrm{eff}} = \frac{P_D \beta_{DU} |g_{DU}|^2}{\sigma_U^2 / N_D}$. This is the standard DBPSK-like performance curve resulting from independent, equal-noise pilot and data matched filtering.

### 4.3 Practical BER Estimation (Statistical Method of Moments)
In practice, if the receiver does not have access to the exact channel gain $S$ or noise variance $\sigma_0^2$, they can be estimated directly from the received samples $y[n] = A d[n] + w[n]$ using the Method of Moments. By matching the sample moments to the theoretical moments:
- **Second Moment:** $M_2 = E[y^2] = A^2 + \sigma^2$
- **Fourth Moment:** $M_4 = E[y^4] = A^4 + 6A^2\sigma^2 + 3\sigma^4$

Solving this system yields the estimators:

$$
\sigma^2 = M_2 - \sqrt{\frac{3M_2^2 - M_4}{2}}, \qquad A^2 = \sqrt{\frac{3M_2^2 - M_4}{2}}
$$

The estimated SNR $\hat{\gamma} = A^2/\sigma^2$ is then used to estimate the BER:

$$
\hat{P}_e = Q\left(\sqrt{\hat{\gamma}}\right)
$$

The resulting BER curves from the simulation are shown below, illustrating the match between the simulated BER and the analytical curve derived via the MGF method:

![UE BER Curves](ue_ber.png)

---

## 5. Received Signal at the Drone Receiver (Sensing)

The drone's receive chain collects the backscatter from both the UE (cooperative echo) and the moving object:

$$
y_D[n,m]
= y_{D,\mathrm{UE}}[n,m]
+ y_{D,\mathrm{T}}[n,m]
+ y_{\mathrm{3hop}}[n,m]
+ n_D[n,m],
$$

where $n_D[n,m]\sim\mathcal{CN}(0,\sigma_D^2)$ is receiver noise and $y_{\mathrm{3hop}} = y_{DTUD} + y_{DUTD}$ collects the two reciprocal three-hop echoes (defined below).

**UE-related echo ($D\!\to\!U\!\to\!D$):**

$$
y_{D,\mathrm{UE}}[n,m]
= \eta_U\,\beta_{DU}\,g_{DU}^2\,
  \sqrt{P_D}\,s_{\mathrm{mls}}\!\bigl[(n-\ell_{2DU})_{N_D}\bigr]
  \,d_m\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI}}},
$$

where $\eta_U$ is the UE reflection coefficient, $\ell_{2DU}=\mathrm{round}(2\tau_{DU}\,f_s)$ is the round-trip chip delay, and $\nu_{DU}=2f_{D,DU}$ the round-trip Doppler.

**Moving-object echo ($D\!\to\!T\!\to\!D$):**

$$
y_{D,\mathrm{T}}[n,m]
= \eta_T\,\beta_{DT}\,g_{DT}^2\,
  \sqrt{P_D}\,s_{\mathrm{mls}}\!\bigl[(n-\ell_{\mathrm{mono}})_{N_D}\bigr]
  \,d_m\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI}}},
$$

where $\eta_T$ is the complex target reflection coefficient (related to the Radar Cross Section $\sigma_T$), $\ell_{\mathrm{mono}}=\mathrm{round}(\tau_{\mathrm{mono}}\,f_s)$, and $\nu_{\mathrm{mono}}=2f_{D,DT}$.

**Three-hop echoes ($D\!\to\!T\!\to\!U\!\to\!D$ and $D\!\to\!U\!\to\!T\!\to\!D$).** Under reciprocity both share the cumulative delay $\ell_{\mathrm{3hop}} = \mathrm{round}((\tau_{DT} + \tau_{TU} + \tau_{DU})\,f_s)$ and Doppler $\nu_{\mathrm{3hop}} = f_{D,DT} + f_{D,TU} + f_{D,DU}$:

$$
y_{\mathrm{3hop}}[n,m]
= 2\,\eta_T\,\eta_U\,\sqrt{\beta_{DT}\,\beta_{TU}\,\beta_{DU}}\,g_{DT}\,g_{TU}\,g_{DU}\,
  \sqrt{P_D}\,
  s_{\mathrm{mls}}\!\bigl[(n-\ell_{\mathrm{3hop}})_{N_D}\bigr]\,
  d_m\,e^{j2\pi\nu_{\mathrm{3hop}}\,m\,T_{\mathrm{PRI}}}.
$$

These are **neglected**: they suffer triple free-space path loss ($\propto d^{-6}$) and double scattering loss ($|\eta_T|^2 |\eta_U|^2 \ll 1$), placing them $\sim\!70$–$80\text{ dB}$ below the noise floor — far weaker than the monostatic target echo ($\propto d^{-4}$, single reflection). Dropping them, the working received signal is

$$
y_D[n,m]
= \underbrace{
    \eta_T\,\beta_{DT}\,g_{DT}^2\,\sqrt{P_D}\,
    s_{\mathrm{mls}}\!\bigl[(n-\ell_{\mathrm{mono}})_{N_D}\bigr]
    \,d_m\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI}}}
  }_{\text{object echo}}
+ \underbrace{
    \eta_U\,\beta_{DU}\,g_{DU}^2\,\sqrt{P_D}\,
    s_{\mathrm{mls}}\!\bigl[(n-\ell_{2DU})_{N_D}\bigr]
    \,d_m\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI}}}
  }_{\text{UE echo (interference for sensing)}}
+ n_D[n,m].
$$

Both echoes are spread by the **same** code, distinguished only by delay ($\ell_{\mathrm{mono}}$ vs. $\ell_{2DU}$) and Doppler ($\nu_{\mathrm{mono}}$ vs. $\nu_{DU}$). Since the cooperative UE echo is typically $30$–$40\text{ dB}$ stronger than the target backscatter, its range sidelobes can mask the weak target — handled by SIC below.

### 5.1 Sensing Signal Processing at the Drone

A **single-branch** pipeline: one matched filter yields a single joint Range–Doppler map carrying both reflectors; SIC then unmasks the weak target before CFAR detection.

**Step 1 — BPSK data stripping.** The transmitter knows $d_m$, and both echoes carry it. Since $d_m^2 = 1$, pre-multiplying strips the data modulation, enabling coherent Doppler integration:

$$
y'_D[n,m] = y_D[n,m] \cdot d_m.
$$

**Step 2 — Single matched filtering.** Correlate the stripped signal with $s_{\mathrm{mls}}$ (efficiently via the FFT):

$$
\mathcal{R}[p,m]
= \frac{1}{N_D}\sum_{n=0}^{N_D-1} y'_D[n,m]\,s_{\mathrm{mls}}\!\bigl[(n-p)_{N_D}\bigr]^*
= \mathrm{IDFT}_{k_f}\!\bigl\{ Y'_D[k_f,m]\,S_{\mathrm{mls}}^*[k_f] \bigr\},
$$

with $Y'_D[k_f,m] = \mathrm{DFT}_n\{y'_D[n,m]\}$ and $S_{\mathrm{mls}}[k_f] = \mathrm{DFT}_n\{s_{\mathrm{mls}}[n]\}$. The MLS periodic autocorrelation is $R_{cc}[\Delta] = N_D$ at $\Delta=0$ and $-1$ otherwise, so this single filter produces **two peaks** plus mutual sidelobe leakage. At the target bin $p = \ell_{\mathrm{mono}}$:

$$
\mathcal{R}[\ell_{\mathrm{mono}}, m]
\approx \underbrace{\eta_T\,\beta_{DT}\,g_{DT}^2\,\sqrt{P_D}\,N_D\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI}}}}_{\text{target peak}}
+ \underbrace{\eta_U\,\beta_{DU}\,g_{DU}^2\,\sqrt{P_D}\,R_{cc}[\ell_{\mathrm{mono}}-\ell_{2DU}]\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI}}}}_{\text{UE sidelobe leakage}}
+ \tilde{w}_D[\ell_{\mathrm{mono}},m],
$$

and at the UE bin $p = \ell_{2DU}$ (roles swapped):

$$
\mathcal{R}[\ell_{2DU}, m]
\approx \eta_U\,\beta_{DU}\,g_{DU}^2\,\sqrt{P_D}\,N_D\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI}}}
+ \eta_T\,\beta_{DT}\,g_{DT}^2\,\sqrt{P_D}\,R_{cc}[\ell_{2DU}-\ell_{\mathrm{mono}}]\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI}}}
+ \tilde{w}_D[\ell_{2DU},m],
$$

where $\tilde{w}_D \sim \mathcal{CN}(0, \sigma_D^2/N_D)$ and $|R_{cc}[\Delta]| = 1$ for $\Delta \ne 0$. The MLS sidelobe is only $1$ relative to the peak $N_D$, but because the UE echo is $30$–$40\text{ dB}$ stronger, its sidelobe at the target bin can still rival or bury the target peak — so matched filtering alone cannot separate the two, motivating SIC (Step 5).

**Step 3 — Range estimation.** Each peak's delay bin $p_{\mathrm{peak}}$ gives the reflector range, with resolution and maximum unambiguous range:

$$
R = \frac{c_0\,p_{\mathrm{peak}}}{2\,f_s}, \qquad
\Delta R = \frac{c_0\,T_c}{2}, \qquad
R_{\max} = \frac{c_0\,N_D\,T_c}{2} = \frac{c_0\,T_{\mathrm{PRI}}}{2}.
$$

**Step 4 — Doppler FFT (joint Range–Doppler map).** A windowed slow-time DFT over the $M$ blocks at each range bin $p$ gives a **single joint map**:

$$
\mathcal{X}_{\mathrm{RD}}[p,q]
= \sum_{m=0}^{M-1} \mathcal{R}[p,m]\,w_{\mathrm{slow}}[m]\,e^{-j\frac{2\pi}{M}qm},
$$

with $w_{\mathrm{slow}}[m]$ a window (e.g., Hann) to suppress Doppler sidelobes. The map shows **two peaks**, at $(\ell_{\mathrm{mono}}, q_{\mathrm{mono}})$ for the target and $(\ell_{2DU}, q_{2DU})$ for the UE, with peak amplitudes and Doppler

$$
A_T = \eta_T\,\beta_{DT}\,g_{DT}^2\,\sqrt{P_D}\,N_D, \quad
A_U = \eta_U\,\beta_{DU}\,g_{DU}^2\,\sqrt{P_D}\,N_D, \quad
\nu_T = 2f_{D,DT}, \quad
\nu_U = 2f_{D,DU}.
$$

The radial velocity of reflector $i \in \{T, U\}$, the velocity resolution, and the maximum unambiguous velocity are

$$
v_{r, i} = \frac{\nu_i\,\lambda_D}{2}, \qquad
\nu_i = \frac{q_{\mathrm{peak}, i} - M/2}{M\,T_{\mathrm{PRI}}}, \qquad
\Delta v = \frac{\lambda_D}{2\,M\,T_{\mathrm{PRI}}}, \qquad
v_{\max} = \pm\frac{\lambda_D}{4\,T_{\mathrm{PRI}}}.
$$

> [!WARNING]
> **Target masking on the joint map.** Both reflectors share one heatmap, so the strong UE peak and its sidelobes sit alongside the weak target. If the UE peak — or its sidelobe pedestal — falls in the CFAR training cells of the target's cell-under-test, it inflates the local noise estimate and raises the adaptive threshold, hiding the target (**target masking** / **signal choking**). This is the fundamental cost of the single-code design, mitigated by SIC.

**Step 5 — Successive Interference Cancellation (SIC).** Before detecting the weak target:

1. **Detect the dominant UE peak** in $\mathcal{X}_{\mathrm{RD}}[p,q]$ and estimate its delay $\hat{\ell}_{2DU}$, Doppler $\hat{\nu}_{DU}$, and complex amplitude $\hat{A}_U$.
2. **Reconstruct** the data-stripped UE echo $\hat{y}_{D,\mathrm{UE}}'[n,m]$ from $(\hat{\ell}_{2DU}, \hat{\nu}_{DU}, \hat{A}_U)$ and the known code $s_{\mathrm{mls}}$.
3. **Subtract and re-filter:** $y''_D[n,m] = y'_D[n,m] - \hat{y}_{D,\mathrm{UE}}'[n,m]$, then re-correlate with $s_{\mathrm{mls}}$ to obtain a cleaned map $\mathcal{X}'_{\mathrm{RD}}[p,q]$ in which the UE peak and sidelobes are largely removed, revealing the target.

The resulting joint Range-Doppler maps before and after SIC are shown below:

![Range-Doppler Map](range_doppler.png)

**Reflector identification.** Since both peaks live on one map, the drone cannot label them by code. It cross-references the estimated peak coordinates with the UE's communication telemetry / Doppler feedback (Section 4.1) to identify the UE trajectory, classifying the remaining peak as the passive object.

**Step 6 — 2D CA-CFAR detection.** On the cleaned map $\mathcal{X}'_{\mathrm{RD}}[p,q]$, Cell-Averaging CFAR tests each pixel against an adaptive threshold:

$$
\text{Decision}(p,q)
= \begin{cases}
    1 & |\mathcal{X}'_{\mathrm{RD}}[p,q]|^2 > V_{\mathrm{th}}(p,q), \\
    0 & \text{otherwise},
  \end{cases}
\qquad
V_{\mathrm{th}}(p,q) = T \cdot Z(p,q).
$$

- $Z(p,q) = \frac{1}{N_{\mathrm{train}}}\sum_{(p',q') \in \mathcal{W}_{\mathrm{train}}} |\mathcal{X}'_{\mathrm{RD}}[p',q']|^2$ — local noise power, averaged over the $N_{\mathrm{train}}$ training cells in the outer ring (guard cells excluded).
- $T = N_{\mathrm{train}}\!\left(P_{\mathrm{fa}}^{-1/N_{\mathrm{train}}} - 1\right)$ — threshold multiplier for the target **Probability of False Alarm** $P_{\mathrm{fa}}$ (exponential noise model).
- **Guard cells** buffer the CUT so its own power does not leak into $Z(p,q)$ (self-masking).

```
  T  T  T  T  T  T  T
  T  T  T  T  T  T  T
  T  T  G  G  G  T  T
  T  T  G  C  G  T  T   ← CUT (C)
  T  T  G  G  G  T  T
  T  T  T  T  T  T  T
  T  T  T  T  T  T  T

  C = Cell Under Test | G = Guard Cells | T = Training Cells
```

The **post-processing SNR** at the CUT for reflector $i \in \{T, U\}$ is

$$
\mathrm{SNR}_{\mathrm{CUT}, i}
= \frac{|A_i|^2\,M^2}{\sigma_D^2\,N_D\,M}
= \frac{|\eta_i|^2\,\beta_{Di}^2\,|g_{Di}|^4\,P_D\,N_D\,M}{\sigma_D^2},
$$

where $M$ is the coherent Doppler integration gain and $N_D$ the despreading range processing gain. For the target ($i=T$) this holds after SIC; any residual UE cancellation error adds to the effective noise floor and degrades $\mathrm{SNR}_{\mathrm{CUT}, T}$.
