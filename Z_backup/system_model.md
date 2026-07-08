# System Model: Transmit and Received Signal Mathematics

> *Converted from `system_model.tex` — 3-Node and 4-Node NTN-ISAC Systems*

---

## 1. Three-Node System

> **Figure:** Three-node system: drone (D), ground user equipment (UE), and moving object (T).
> *(See `fig/sys_3node.pdf`)*

As illustrated in the figure, the three-node system consists of:

- **Drone (D):** A mid-air platform equipped with one transmit and one receive antenna. It acts simultaneously as the Joint Radar-Communication (JRC) transmitter, a communication access point for the UE, and a monostatic radar for sensing the moving object.
- **Ground User Equipment (UE):** A single-antenna mobile receiver that receives the downlink communication signal from the drone, and whose position and velocity can be estimated by the drone.
- **Moving Object (T):** A passive aerial target (e.g., a drone or UAV) that is detected and tracked through radar backscatter.

The drone transmits a PMCW waveform. It employs two quasi-orthogonal Maximum-Length Sequence (MLS) spreading codes. The details are explained in [Section 1.3](#13-transmit-signal).

---

### 1.1 Notation and Geometry

Let $d_{DU}$, $d_{DT}$, and $d_{TU}$ denote the instantaneous Euclidean distances between the drone and the UE, between the drone and the object, and between the object and the UE, respectively. The corresponding one-way propagation delays are

$$
\tau_{DU} = \frac{d_{DU}}{c_0}, \qquad \tau_{DT} = \frac{d_{DT}}{c_0}, \qquad \tau_{TU} = \frac{d_{TU}}{c_0},
$$

where $c_0$ is the speed of light. The round-trip monostatic sensing delay for the moving object is

$$
\tau_{\mathrm{mono}} = \frac{2\,d_{DT}}{c_0}.
$$

The total propagation delay for the two-hop scattering path $D\!\to\!T\!\to\!U$ is

$$
\tau_{DTU} = \tau_{DT} + \tau_{TU} = \frac{d_{DT} + d_{TU}}{c_0}.
$$

Let $\mathbf{v}_D$, $\mathbf{v}_U$, and $\mathbf{v}_T$ be the velocity vectors of the drone, UE, and moving object, respectively. The radial velocities along each link are projected onto the unit line-of-sight (LOS) vector $\hat{\mathbf{u}}_{ij}$ from node $i$ to node $j$:

$$
v_{r,DU} = \bigl(\mathbf{v}_U - \mathbf{v}_D\bigr) \cdot \hat{\mathbf{u}}_{DU}, \qquad
v_{r,DT} = \bigl(\mathbf{v}_T - \mathbf{v}_D\bigr) \cdot \hat{\mathbf{u}}_{DT}, \qquad
v_{r,TU} = \bigl(\mathbf{v}_U - \mathbf{v}_T\bigr) \cdot \hat{\mathbf{u}}_{TU}.
$$

The carrier wavelength for the drone PMCW link is $\lambda_D = c_0 / f_{c,D}$. The resulting Doppler shifts are

$$
f_{D,DU} = -\frac{v_{r,DU}}{\lambda_D}, \qquad f_{D,DT} = -\frac{v_{r,DT}}{\lambda_D}, \qquad f_{D,TU} = -\frac{v_{r,TU}}{\lambda_D},
$$

the two-way monostatic Doppler shift for target sensing is $\nu_{\mathrm{mono}} = 2\,f_{D,DT}$, and the combined Doppler shift accumulated over the two-hop $D\!\to\!T\!\to\!U$ scattering path is

$$
\nu_{DTU} = f_{D,DT} + f_{D,TU}.
$$

---

### 1.2 Channel Model

All drone-to-node and node-to-drone links in the three-node system are SISO channels. Following the generic narrowband link model, the channel coefficient between nodes $i$ and $j$ is decomposed as

$$
h_{ij} = \sqrt{\beta_{ij}}\,g_{ij},
$$

where $\beta_{ij}$ is the large-scale power gain (path loss and shadowing) and $g_{ij}$ is the complex small-scale fading coefficient. A distance-dependent path-loss model gives

$$
\beta_{ij} = C_{ij}\!\left(\frac{d_{ij}}{d_0}\right)^{-\alpha_{ij}},
$$

with $C_{ij} = G_i G_j \!\left(\lambda_D / (4\pi d_0)\right)^2$ being the reference channel gain at distance $d_0$, and $\alpha_{ij}$ the path-loss exponent.

The time-varying drone-to-UE channel (including the Doppler phase rotation) is modeled as

$$
h_{DU}(t) = \sqrt{\beta_{DU}}\,g_{DU}\,e^{j2\pi f_{D,DU}\,t},
$$

and similarly for the drone-to-target and target-to-drone channels:

$$
h_{DT}(t) = \sqrt{\beta_{DT}}\,g_{DT}\,e^{j2\pi f_{D,DT}\,t}, \qquad
h_{TD}(t) = \sqrt{\beta_{TD}}\,g_{TD}\,e^{j2\pi f_{D,DT}\,t}.
$$

Under channel reciprocity, $\beta_{TD}=\beta_{DT}$ and $g_{TD}=g_{DT}$.

To account for the scattering path $D\!\to\!T\!\to\!U$ (see Section 1.4), an additional channel between the moving object and the UE must be introduced:

$$
h_{TU}(t) = \sqrt{\beta_{TU}}\,g_{TU}\,e^{j2\pi f_{D,TU}\,t},
$$

where $\beta_{TU}$ is the large-scale path gain and $g_{TU}\sim\mathcal{CN}(0,1)$ is the complex small-scale fading coefficient of the $T\!\to\!U$ link. 

 **$h_{TU}$ is an unknown channel.** Since the object's position $d_{TU}$, velocity $\mathbf{v}_T$, and scattering pattern toward the UE are all unknown a priori (they are precisely what the radar is trying to estimate), the parameters $\beta_{TU}$, $g_{TU}$, and $f_{D,TU}$ cannot be predicted by either the drone or the UE. This makes the $D\!\to\!T\!\to\!U$ interference fundamentally different from the direct $D\!\to\!U$ link: it cannot be cancelled by pre-coding or Doppler pre-compensation at the transmitter.

---

### 1.3 Transmit Signal

The drone transmits a Phase-Modulated Continuous Wave (PMCW) waveform. It employs two quasi-orthogonal Maximum-Length Sequence (MLS) spreading codes of length $N_D$: $s_{\mathrm{mls},U}[n]\in\{+1,-1\}$ directed toward the UE, and $s_{\mathrm{mls},T}[n]\in\{+1,-1\}$ directed toward the moving object. The discrete-time baseband transmit signal during pulse $m$ (fast-time chip index $n$, $0 \le n < N_D$, $0 \le m < M$) is

$$
x_D[n,m]
= \sqrt{P_D}\,\Bigl(
    s_{\mathrm{mls},U}[n]
    + s_{\mathrm{mls},T}[n]
  \Bigr)\,b_m,
$$

where $P_D$ is the drone transmit power and $b_m \in \{+1,-1\}$ is the BPSK communication symbol carried on pulse $m$. The two codes are designed to be quasi-orthogonal so that code-domain separation can suppress cross-interference at the respective receivers. The chip duration is $T_c$, giving a Pulse Repetition Interval (PRI) of $T_{\mathrm{PRI}} = N_D\,T_c$.

---

### 1.4 Received Signal at the UE

In addition to the direct drone-to-UE link, the UE also receives a **scattered contribution** from the two-hop path $D\!\to\!T\!\to\!U$: the drone's transmitted signal impinges on the moving object and is re-radiated (diffracted/reflected) toward the UE. This scattered path acts as an uncontrolled interference on the communication link. The composite received signal at the UE is therefore:

$$
y_U[n,m]
= \underbrace{h_{DU}\, x_D\!\bigl[(n - \ell_{DU})_{N_D},\,m\bigr]\,
  e^{j2\pi f_{D,DU}\,m\,T_{\mathrm{PRI}}}}_{ \text{direct } D\to U \text{ link}}
+ \underbrace{y_{DTU}[n,m]}_{\text{scattered } D\to T\to U \text{ interference}}
+ n_U[n,m],
$$

where $\ell_{DU} = \mathrm{round}(\tau_{DU}\,f_s)$ is the integer chip delay, $(\cdot)_{N_D}$ denotes modulo-$N_D$, $f_s = 1/T_c$ is the chip-rate sampling frequency, and $n_U[n,m]\sim\mathcal{CN}(0,\sigma_U^2)$ is additive white Gaussian noise.

**Scattered Interference Term $y_{DTU}$.**
The signal travels from the drone to the moving object (delay $\tau_{DT}$, Doppler $f_{D,DT}$, channel $h_{DT}$), is scattered with complex reflection coefficient $\eta_T$, and then propagates from the object to the UE (delay $\tau_{TU}$, Doppler $f_{D,TU}$, channel $h_{TU}$):

$$
y_{DTU}[n,m]
= \eta_T\,\sqrt{\beta_{DT}\,\beta_{TU}}\,g_{DT}\,g_{TU}\,
  \sqrt{P_D}\,
  \Bigl(
    s_{\mathrm{mls},U}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr]
    + s_{\mathrm{mls},T}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr]
  \Bigr)
  b_m\,e^{j2\pi\nu_{DTU}\,m\,T_{\mathrm{PRI}}},
$$

where $\ell_{DTU} = \mathrm{round}(\tau_{DTU}\,f_s)$ is the total two-hop chip delay, and $\nu_{DTU} = f_{D,DT} + f_{D,TU}$ is the cumulative Doppler shift over the $D\!\to\!T\!\to\!U$ path.

Substituting the direct path and substituting the transmit signal, the full explicit received signal at the UE is:

$$
y_U[n,m]
= \underbrace{\sqrt{P_D}\,h_{DU}
  \Bigl(s_{\mathrm{mls},U}\!\bigl[(n-\ell_{DU})_{N_D}\bigr]
  + s_{\mathrm{mls},T}\!\bigl[(n-\ell_{DU})_{N_D}\bigr]\Bigr)
  b_m\,e^{j2\pi f_{D,DU}\,m\,T_{\mathrm{PRI}}}}_{\text{desired direct link}}
+ \underbrace{\eta_T\,\sqrt{\beta_{DT}\beta_{TU}}\,g_{DT}\,g_{TU}\,\sqrt{P_D}\,
  \Bigl(
    s_{\mathrm{mls},U}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr]
    + s_{\mathrm{mls},T}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr]
  \Bigr)
  b_m\,e^{j2\pi\nu_{DTU}\,m\,T_{\mathrm{PRI}}}}_{\text{scattered } D\to T\to U \text{ interference (unknown channel)}}
+ n_U[n,m].
$$

The communication data $b_m$ is recovered by correlating with the UE code $s_{\mathrm{mls},U}$. The sensing code $s_{\mathrm{mls},T}$ in the direct path is suppressed by quasi-orthogonality with gain $N_D$. The scattered interference term, however, carries **both** codes delayed by $\ell_{DTU}$ and shifted in Doppler by $\nu_{DTU}$; since $h_{TU}$ is unknown, this interference cannot be pre-cancelled and must be treated as residual structured noise at the UE receiver.

#### 1.4.1 Data Demodulation at the UE

The UE demodulates the data symbol $b_m$ through the following three steps.

**Step 1 — Doppler Compensation.**
Before despreading, the UE must compensate for the slow-time Doppler phase rotation on the direct link. Using an estimated Doppler $\hat{f}_{D,DU}$ (obtained from a pilot-aided frequency estimator or ephemeris data), a phase correction is applied chip-by-chip in the slow-time dimension:

$$
\tilde{y}_U[n,m] = y_U[n,m]\,e^{-j2\pi \hat{f}_{D,DU}\,m\,T_{\mathrm{PRI}}}.
$$

Assuming perfect Doppler estimation ($\hat{f}_{D,DU} = f_{D,DU}$), the direct-link term loses its Doppler phase, while the interference term acquires a residual Doppler $\Delta\nu = \nu_{DTU} - f_{D,DU} = f_{D,DT} + f_{D,TU} - f_{D,DU}$.

**Step 2 — Despreading (Correlation with $s_{\mathrm{mls},U}$).**
The UE correlates the Doppler-compensated signal over the $N_D$ chips of pulse $m$ with the known UE spreading code $s_{\mathrm{mls},U}$:

$$
z_U[m]
= \frac{1}{N_D}\sum_{n=0}^{N_D-1} \tilde{y}_U[n,m]\,s_{\mathrm{mls},U}[n].
$$

Substituting $\tilde{y}_U[n,m]$ and evaluating each term:

$$
z_U[m]
= \underbrace{\sqrt{P_D}\,h_{DU}\,b_m}_{{\text{desired signal}}}
+ \underbrace{\sqrt{P_D}\,h_{DU}\,\frac{1}{N_D}\sum_{n=0}^{N_D-1} s_{\mathrm{mls},T}\!\bigl[(n-\ell_{DU})_{N_D}\bigr]\,s_{\mathrm{mls},U}[n]\,b_m}_{\approx\,0\;\text{(sensing code suppressed by quasi-orthogonality)}}
+ \underbrace{I_{\mathrm{scat}}[m]}_{\text{scattered interference}}
+ \underbrace{\tilde{n}_U[m]}_{\text{noise}},
$$

where $\tilde{n}_U[m] = \frac{1}{N_D}\sum_{n} n_U[n,m]\,s_{\mathrm{mls},U}[n] \sim \mathcal{CN}(0,\sigma_U^2/N_D)$ is the post-despreading noise (reduced by the processing gain $N_D$), and the scattered interference after despreading is:

$$
I_{\mathrm{scat}}[m]
= \eta_T\,\sqrt{\beta_{DT}\beta_{TU}}\,g_{DT}\,g_{TU}\,\sqrt{P_D}
  \underbrace{\left[\frac{1}{N_D}\sum_{n=0}^{N_D-1}
    \Bigl( s_{\mathrm{mls},U}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr] + s_{\mathrm{mls},T}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr] \Bigr)\,s_{\mathrm{mls},U}[n]
  \right]}_{\rho_U(\ell_{DTU}) + \rho_{TU}(\ell_{DTU})\;\text{(residual correlation at delay }\ell_{DTU}\text{)}}
  b_m\,e^{j2\pi\Delta\nu\,m\,T_{\mathrm{PRI}}}.
$$

Here $\rho_U(\ell_{DTU})$ is the **aperiodic auto-correlation** of $s_{\mathrm{mls},U}$ at delay $\ell_{DTU}$. For an MLS code, this takes the value $-1/N_D$ for all non-zero delays, i.e., $|\rho_U(\ell)| = 1/N_D$ for $\ell \ne 0$, so the scattered interference is also suppressed by the same processing gain $N_D$ — provided the two-hop delay $\ell_{DTU} \ne 0$.

If the scattering path is very short (i.e., the target is very close to both the drone and the UE), it is possible that $\ell_{DTU} = \ell_{DU}$ (same delay bin as the direct path). In this case, $\rho_U(0) = 1$ and the scattered term causes **maximum interference** — equivalent to a co-delay co-Doppler jammer. This is the worst-case scenario and cannot be mitigated by code despreading alone; spatial filtering or Doppler discrimination would be needed.

**Step 3 — BPSK Decision.**
After despreading, the sufficient statistic for the BPSK data symbol $b_m \in \{+1,-1\}$ is the real part of the Doppler-equalized despread sample:

$$
\hat{b}_m = \mathrm{sign}\!\left(\mathrm{Re}\!\left\{\frac{z_U[m]}{h_{DU}\,\sqrt{P_D}}\right\}\right),
$$

where $h_{DU}$ is estimated using a pilot sequence (known symbols $b_m$) inserted periodically by the drone. The resulting **post-despreading SINR** at the UE is:

$$
\mathrm{SINR}_U
= \frac{P_D\,\beta_{DU}\,|g_{DU}|^2}
       {P_D\,|\eta_T|^2\,\beta_{DT}\,\beta_{TU}\,|g_{DT}|^2\,|g_{TU}|^2\,|\rho_U(\ell_{DTU}) + \rho_{TU}(\ell_{DTU})|^2
       + \dfrac{\sigma_U^2}{N_D}}.
$$

The three terms in the denominator are:
- **Numerator:** Power of the desired despread signal.
- **First denominator term:** Residual power of the $D\!\to\!T\!\to\!U$ scattered interference after despreading (suppressed by $|\rho_U|^2 \approx 1/N_D^2$ for $\ell_{DTU} \ne 0$).
- **Second denominator term:** Thermal noise power, reduced by the processing gain $N_D$ through despreading.

---

### 1.5 Received Signal at the Drone Receiver (Sensing)

The drone's receive chain collects the backscatter from both the UE (cooperative echo / passive reflection) and the moving object. In the general case, the composite received signal at the drone is

$$
y_D[n,m]
= y_{D,\mathrm{UE}}[n,m]
+ y_{D,\mathrm{T}}[n,m]
+ y_{\mathrm{3hop}}[n,m]
+ n_D[n,m],
$$

where $y_{\mathrm{3hop}}[n,m] = y_{DTUD}[n,m] + y_{DUTD}[n,m]$ represents the sum of the two reciprocal three-hop scattered echoes ($D\!\to\!T\!\to\!U\!\to\!D$ and $D\!\to\!U\!\to\!T\!\to\!D$), and $n_D[n,m]\sim\mathcal{CN}(0,\sigma_D^2)$ is receiver noise.

**Reciprocal Three-Hop Scattered Echoes $y_{\mathrm{3hop}}$.**
The two three-hop scattering paths travel in reciprocal directions: Drone-to-Target-to-UE-to-Drone ($D\!\to\!T\!\to\!U\!\to\!D$) and Drone-to-UE-to-Target-to-Drone ($D\!\to\!U\!\to\!T\!\to\!D$). Under channel reciprocity, both paths share the same cumulative propagation delay $\ell_{\mathrm{3hop}} = \mathrm{round}((\tau_{DT} + \tau_{TU} + \tau_{DU})\,f_s)$ and cumulative Doppler shift $\nu_{\mathrm{3hop}} = f_{D,DT} + f_{D,TU} + f_{D,DU}$. The sum of these two paths is modeled as:

$$
y_{\mathrm{3hop}}[n,m]
= 2\,\eta_T\,\eta_U\,\sqrt{\beta_{DT}\,\beta_{TU}\,\beta_{DU}}\,g_{DT}\,g_{TU}\,g_{DU}\,
  \sqrt{P_D}\,
  \Bigl(
    s_{\mathrm{mls},U}\!\bigl[(n-\ell_{\mathrm{3hop}})_{N_D}\bigr]
    + s_{\mathrm{mls},T}\!\bigl[(n-\ell_{\mathrm{3hop}})_{N_D}\bigr]
  \Bigr)
  b_m\,e^{j2\pi\nu_{\mathrm{3hop}}\,m\,T_{\mathrm{PRI}}},
$$

where $\ell_{\mathrm{3hop}}$ and $\nu_{\mathrm{3hop}}$ represent the cumulative delay and Doppler shift, respectively.

**Neglect of $y_{\mathrm{3hop}}$ in Practice:** Although thermal noise $n_D[n,m]$ is a limiting factor in detection, these three-hop echoes are safely neglected because their power is **physically negligible** (typically $70$ to $80\text{ dB}$ below the thermal noise floor). Specifically, they suffer from **triple free-space path loss** ($\propto d^{-6}$) and **double scattering loss** (scaling with the product of target and UE reflection coefficients, $|\eta_T|^2 |\eta_U|^2 \ll 1$), whereas the monostatic target echo $y_{D,\mathrm{T}}$ suffers only two-way path loss ($\propto d^{-4}$) and a single reflection. Thus, even after processing gains (despreading and Doppler integration), $y_{\mathrm{3hop}}$ remains far below the noise floor and can be omitted.

**UE-Related Echo.**
The echo returning from the UE path ($D\!\to\!U\!\to\!D$) is modeled as

$$
y_{D,\mathrm{UE}}[n,m]
= \eta_U\,\beta_{DU}\,g_{DU}^2\,
  \sqrt{P_D}\,s_{\mathrm{mls},U}\!\bigl[(n-\ell_{2DU})_{N_D}\bigr]
  \,b_m\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI}}},
$$

where $\eta_U$ is the UE reflection coefficient, $\ell_{2DU}=\mathrm{round}(2\tau_{DU}\,f_s)$ is the round-trip chip delay, and $\nu_{DU}=2f_{D,DU}$ is the round-trip Doppler shift.

**Moving-Object Echo.**
The monostatic target echo from the object ($D\!\to\!T\!\to\!D$) is modeled as

$$
y_{D,\mathrm{T}}[n,m]
= \eta_T\,\beta_{DT}\,g_{DT}^2\,
  \sqrt{P_D}\,s_{\mathrm{mls},T}\!\bigl[(n-\ell_{\mathrm{mono}})_{N_D}\bigr]
  \,b_m\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI}}},
$$

where $\eta_T$ is the complex target reflection coefficient (related to the Radar Cross Section, $\sigma_T$), $\ell_{\mathrm{mono}}=\mathrm{round}(\tau_{\mathrm{mono}}\,f_s)$, and $\nu_{\mathrm{mono}}=2f_{D,DT}$ is the round-trip Doppler. Under channel reciprocity, $\beta_{TD}=\beta_{DT}$ and $g_{TD}=g_{DT}$.

The full monostatic received signal at the drone (omitting the negligible three-hop terms $y_{\mathrm{3hop}}$) is therefore:

$$
y_D[n,m]
= \underbrace{
    \eta_T\,\beta_{DT}\,g_{DT}^2\,\sqrt{P_D}\,
    s_{\mathrm{mls},T}\!\bigl[(n-\ell_{\mathrm{mono}})_{N_D}\bigr]
    \,b_m\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI}}}
  }_{\text{object sensing echo}}
+ \underbrace{
    \eta_U\,\beta_{DU}\,g_{DU}^2\,\sqrt{P_D}\,
    s_{\mathrm{mls},U}\!\bigl[(n-\ell_{2DU})_{N_D}\bigr]
    \,b_m\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI}}}
  }_{\text{UE-related echo (interference for object sensing)}}
+ n_D[n,m].
$$

The object sensing echo is isolated by correlating $y_D[n,m]$ with $s_{\mathrm{mls},T}$; the UE echo term is suppressed by code quasi-orthogonality with gain $N_D$.

#### 1.5.1 Sensing Signal Processing at the Drone

The drone processes the received signal $y_D[n,m]$ through the following pipeline to estimate the range and velocity of each reflector (UE and moving object) separately, and then applies CFAR-based detection.

---

**Step 1 — BPSK Data Stripping.**
The drone's transmitter knows the transmitted data symbol $b_m \in \{+1,-1\}$ for every block $m$. Since the transmit signal modulates both the UE and target codes by $b_m$, both the UE-related echo and the moving-object target echo carry the BPSK symbol $b_m$. Since $b_m^2 = 1$, the drone pre-multiplies the received signal by the known symbol $b_m$ to strip the data modulation from both echo branches simultaneously:

$$
y'_D[n,m] = y_D[n,m] \cdot b_m.
$$

This operation yields a data-free signal $y'_D[n,m]$ for both the UE and target code branches, enabling coherent Doppler integration across the $M$ slow-time blocks.

---

**Step 2 — Separate Despreading for Each Target.**
The drone correlates the stripped received signal independently with each of the two MLS codes to separate the two echo branches.

**Despreading for object sensing (correlate with $s_{\mathrm{mls},T}$):**

$$
\mathcal{R}_T[p,m]
= \frac{1}{N_D}\sum_{n=0}^{N_D-1} y'_D[n,m]\,s_{\mathrm{mls},T}\!\bigl[(n-p)_{N_D}\bigr]^*,
$$

which gives a peak at the chip-delay bin $p = \ell_{\mathrm{mono}}$ corresponding to the object range, and suppresses the UE echo with gain $N_D$ by quasi-orthogonality.

**Despreading for UE sensing (correlate with $s_{\mathrm{mls},U}$):**

$$
\mathcal{R}_U[p,m]
= \frac{1}{N_D}\sum_{n=0}^{N_D-1} y'_D[n,m]\,s_{\mathrm{mls},U}\!\bigl[(n-p)_{N_D}\bigr]^*,
$$

which gives a peak at $p = \ell_{2DU}$ corresponding to the UE round-trip range, and suppresses the object echo with gain $N_D$.

In the frequency domain (for computational efficiency), both operations are implemented as:
$$
\mathcal{R}_i[p,m] = \mathrm{IDFT}_{k_f}\!\bigl\{ Y'_D[k_f,m]\,S_{\mathrm{mls},i}^*[k_f] \bigr\}, \quad i \in \{T, U\},
$$
where $Y'_D[k_f,m] = \mathrm{DFT}_n\{y'_D[n,m]\}$ and $S_{\mathrm{mls},i}[k_f] = \mathrm{DFT}_n\{s_{\mathrm{mls},i}[n]\}$.

Since the MLS code has a near-ideal circular autocorrelation ($R_{cc}[p] \approx N_D\,\delta[p]$), the matched filter output at the correct delay bins simplifies to:

**For the object sensing branch peak ($p = \ell_{\mathrm{mono}}$):**
$$
\mathcal{R}_T[\ell_{\mathrm{mono}}, m]
\approx \eta_T\,\beta_{DT}\,g_{DT}^2\,\sqrt{P_D}\,N_D\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI}}}
+ \tilde{w}_D[\ell_{\mathrm{mono}},m],
$$

**For the UE sensing branch peak ($p = \ell_{2DU}$):**
$$
\mathcal{R}_U[\ell_{2DU}, m]
\approx \eta_U\,\beta_{DU}\,g_{DU}^2\,\sqrt{P_D}\,N_D\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI}}}
+ \tilde{w}_U[\ell_{2DU},m],
$$

where $\tilde{w}_D, \tilde{w}_U \sim \mathcal{CN}(0, \sigma_D^2/N_D)$ are the post-despreading noise terms.

For any delay bin $p$ that does not match the true delay of the respective branch (i.e., $p \ne \ell_{\mathrm{mono}}$ for the target, and $p \ne \ell_{2DU}$ for the UE), the matched filter output is approximately zero (noise-like). In particular, at the cross-delay bins (evaluating the target branch at the UE delay $p = \ell_{2DU}$, or vice versa), the unwanted echo is suppressed by the quasi-orthogonality of the two codes:
$$
\mathcal{R}_T[p,m] \approx \tilde{w}_D[p,m], \quad \forall p \ne \ell_{\mathrm{mono}},
$$
$$
\mathcal{R}_U[p,m] \approx \tilde{w}_U[p,m], \quad \forall p \ne \ell_{2DU}.
$$

---

**Step 3 — Range Estimation.**
The range-delay bin $p_{\mathrm{peak}}$ of the correlation peak directly yields the range of each reflector:

$$
R = \frac{c_0\,p_{\mathrm{peak}}\,T_c}{2} = \frac{c_0\,p_{\mathrm{peak}}}{2\,f_s}.
$$

The **range resolution** is set by the chip rate:
$$
\Delta R = \frac{c_0}{2\,f_s} = \frac{c_0\,T_c}{2}.
$$

The **maximum unambiguous range** is limited by the code length:
$$
R_{\max} = \frac{c_0\,N_D\,T_c}{2} = \frac{c_0\,T_{\mathrm{PRI}}}{2}.
$$

---

**Step 4 — Doppler FFT (Range–Doppler Map).**
Fixing the range bin at $p_{\mathrm{peak}, i}$ for each branch $i \in \{T, U\}$ (where $p_{\mathrm{peak}, T} = \ell_{\mathrm{mono}}$ for target sensing, and $p_{\mathrm{peak}, U} = \ell_{2DU}$ for UE sensing), the slow-time signal across the $M$ blocks is a pure complex exponential:

$$
s_{\mathrm{slow}, i}[m]
= \mathcal{R}_i[p_{\mathrm{peak}, i},m]
\approx A_i\,e^{j2\pi\nu_i\,m\,T_{\mathrm{PRI}}},
$$

where the complex amplitudes are $A_T = \eta_T\,\beta_{DT}\,g_{DT}^2\,\sqrt{P_D}\,N_D$ and $A_U = \eta_U\,\beta_{DU}\,g_{DU}^2\,\sqrt{P_D}\,N_D$, and the Doppler shifts are $\nu_T = \nu_{\mathrm{mono}} = 2f_{D,DT}$ and $\nu_U = \nu_{DU} = 2f_{D,DU}$.

To estimate the Doppler frequency $\nu_i$, a windowed DFT of size $M$ is applied across slow-time for each range bin $p$:

$$
\mathcal{X}_{\mathrm{RD}, i}[p,q]
= \sum_{m=0}^{M-1} \mathcal{R}_i[p,m]\,w_{\mathrm{slow}}[m]\,e^{-j\frac{2\pi}{M}qm},
$$

where $w_{\mathrm{slow}}[m]$ is a weighting window (e.g., Hann) applied to reduce Doppler sidelobes. This produces two independent **2D Range–Doppler maps** $\mathcal{X}_{\mathrm{RD}, T}[p,q]$ and $\mathcal{X}_{\mathrm{RD}, U}[p,q]$ with peaks at $(p_{\mathrm{peak}, T}, q_{\mathrm{peak}, T})$ and $(p_{\mathrm{peak}, U}, q_{\mathrm{peak}, U})$, respectively.

The **radial velocity** of reflector/target $i$ is recovered from the Doppler peak bin:
$$
\nu_i = \frac{(q_{\mathrm{peak}, i} - M/2)}{M\,T_{\mathrm{PRI}}}, \qquad
v_{r, i} = \frac{\nu_i\,\lambda_D}{2}.
$$

The **velocity resolution** and **maximum unambiguous velocity** for both branches are:
$$
\Delta v = \frac{\lambda_D}{2\,M\,T_{\mathrm{PRI}}}, \qquad
v_{\max} = \pm\frac{\lambda_D}{4\,T_{\mathrm{PRI}}}.
$$

---

**Step 5 — 2D CA-CFAR Target Detection.**
The 2D Range–Doppler maps $|\mathcal{X}_{\mathrm{RD}, i}[p,q]|^2$ contain the target/reflector power peaks embedded in noise and clutter. The **Cell-Averaging CFAR (CA-CFAR)** algorithm evaluates every pixel $(p,q)$ of each map $i \in \{T, U\}$ independently using a sliding window:

$$
\text{Decision}_i(p,q)
= \begin{cases}
    1\;\text{(Target Detected)}, & |\mathcal{X}_{\mathrm{RD}, i}[p,q]|^2 > V_{\mathrm{th}, i}(p,q), \\
    0\;\text{(Noise)}, & \text{otherwise},
  \end{cases}
$$

where the **adaptive threshold** at each cell under test (CUT) is:
$$
V_{\mathrm{th}, i}(p,q) = T \cdot Z_i(p,q).
$$

Here:
- $Z_i(p,q) = \frac{1}{N_{\mathrm{train}}}\sum_{(p',q') \in \mathcal{W}_{\mathrm{train}}} |\mathcal{X}_{\mathrm{RD}, i}[p',q']|^2$ is the **local noise power estimate** averaged over the $N_{\mathrm{train}}$ training cells in the outer ring of the CFAR window (excluding guard cells that buffer the CUT from its own sidelobes).
- $T = N_{\mathrm{train}}\!\left(P_{\mathrm{fa}}^{-1/N_{\mathrm{train}}} - 1\right)$ is the **threshold multiplier**, derived analytically from the exponential noise power distribution to maintain the desired **Probability of False Alarm** $P_{\mathrm{fa}}$.
- **Guard cells** surround the CUT to prevent the target's own power from leaking into $Z_i(p,q)$ and inflating the threshold (self-masking).

The CFAR window structure for each CUT is:

```
  T  T  T  T  T  T  T
  T  T  T  T  T  T  T
  T  T  G  G  G  T  T
  T  T  G  C  G  T  T   ← CUT  (C)
  T  T  G  G  G  T  T
  T  T  T  T  T  T  T
  T  T  T  T  T  T  T

  C = Cell Under Test | G = Guard Cells | T = Training Cells
```

The **post-processing SNR** at the CUT for branch $i$ is:
$$
\mathrm{SNR}_{\mathrm{CUT}, i}
= \frac{|A_i|^2\,M^2}{\sigma_D^2\,N_D\,M}
= \frac{|\eta_i|^2\,\beta_{Di}^2\,|g_{Di}|^4\,P_D\,N_D\,M}{\sigma_D^2},
$$
where $M$ is the coherent Doppler integration gain (DFT over $M$ blocks) and $N_D$ is the despreading range processing gain.

> [!NOTE]
> **Prevention of Target Masking via CDM:**
> Evaluating CA-CFAR independently on two separate maps ($\mathcal{X}_{\mathrm{RD}, T}$ and $\mathcal{X}_{\mathrm{RD}, U}$) is a major advantage of the two-code (CDM) design. 
> In a single-code system where both targets appear in a single joint heatmap, a strong reflector (such as the cooperative UE) located in the training cells ($\mathcal{W}_{\mathrm{train}}$) of a weak reflector (the object target) would inflate the local noise estimate $Z(p,q)$ of that cell. This dramatically raises the adaptive threshold $V_{\mathrm{th}}(p,q)$, causing the weak target to go undetected (a phenomenon known as **target masking** or **signal choking**). Code-domain separation avoids this entirely because each node's echo is processed on its own separate heatmap, keeping the noise estimates $Z_i(p,q)$ clean.

#### 1.5.2 Alternative: Single-Waveform (Single-Code) Joint Sensing

If the drone transmits a single JRC waveform shared by both the communication and radar sensing functions (rather than employing Code-Division Multiplexing with two distinct MLS codes), the transmit signal simplifies to:

$$
x_D[n,m] = \sqrt{P_D}\,s_{\mathrm{mls}}[n]\,b_m,
$$

where $s_{\mathrm{mls}}[n]$ is the single shared Maximum-Length Sequence. Under this setup, the composite received signal at the drone receiver becomes:

$$
y_D[n,m] = \sqrt{P_D}\left( \eta_T\,\beta_{DT}\,g_{DT}^2\,s_{\mathrm{mls}}\!\bigl[(n-\ell_{\mathrm{mono}})_{N_D}\bigr]\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI}}} + \eta_U\,\beta_{DU}\,g_{DU}^2\,s_{\mathrm{mls}}\!\bigl[(n-\ell_{2DU})_{N_D}\bigr]\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI}}} \right) b_m + n_D[n,m].
$$

To extract the delay and Doppler parameters of both the UE and the moving object, the receiver executes a single-branch processing pipeline:

1. **BPSK Data Stripping:** The drone multiplies the composite received signal by the known BPSK sequence $b_m$ (since $b_m^2 = 1$):
   $$
   y'_D[n,m] = y_D[n,m] \cdot b_m.
   $$
2. **Single Matched Filtering:** The stripped signal is correlated with the single reference sequence $s_{\mathrm{mls}}[n]$:
   $$
   \mathcal{R}[p,m] = \frac{1}{N_D}\sum_{n=0}^{N_D-1} y'_D[n,m]\,s_{\mathrm{mls}}\!\bigl[(n-p)_{N_D}\bigr]^*.
   $$
3. **Joint Range-Doppler Map:** Applying the slow-time FFT across blocks $m$ yields a **single joint Range-Doppler heatmap** $\mathcal{X}_{\mathrm{RD}}[p,q]$. This single map displays **two distinct power peaks** corresponding to the coordinates of the target $(\ell_{\mathrm{mono}}, q_{\mathrm{mono}})$ and the UE $(\ell_{2DU}, q_{2DU})$ simultaneously.

##### Implementation Challenges and Solutions

While a single-waveform setup simplifies transmitter design and reduces spectral footprint, it introduces several JRC processing challenges:

* **Target Masking (The Dynamic Range Problem):** Since the UE-related echo is a cooperative reflection, its power is typically $30$ to $40\text{ dB}$ higher than the weak radar backscatter from the moving object. Under a single-code scheme, the range sidelobes of the strong UE peak will bleed across the delay bins, completely drowning out the weak target peak.
  * *Solution:* The drone must implement **Successive Interference Cancellation (SIC)**. The receiver first detects the dominant UE peak, estimates its complex channel gain and delay, reconstructs the corresponding UE echo waveform, and subtracts it from $y'_D[n,m]$. It then correlates the residual signal with $s_{\mathrm{mls}}$ to reveal the weaker target peak.
* **Reflector Peak Ambiguity (Identification):** Because both peaks appear in the same heatmap, the drone cannot immediately distinguish between the UE and the target based on the code branch.
  * *Solution:* The drone must cross-reference the estimated peak coordinates with the communication telemetry or estimated Doppler feedback from the UE's demodulation loop to identify the UE's trajectory, allowing it to classify the remaining peak as the passive object.

---

## 2. Four-Node System

> **Figure:** Four-node system: LEO satellite (S), mid-air drone (D), ground user equipment (UE), and moving target (T).
> *(See `fig/sys_4node.pdf`)*

The four-node system extends the three-node architecture by introducing a **LEO Satellite (S)** as a high-power, wide-area transmitter that serves two simultaneous roles: (1) it is the primary downlink communication source for the UE, and (2) it acts as a bistatic radar illuminator for the moving target. The four nodes are:

- **Satellite (S):** A multi-antenna LEO satellite with $N_s$ transmit elements. It transmits a joint communication-and-sensing waveform (OTFS) toward the ground. Its echoes off the target are received by the drone for bistatic sensing.
- **Drone (D):** A single transmit/receive antenna aerial platform. It simultaneously acts as a PMCW monostatic radar (sensing the target and UE directly), a **decode-and-forward relay** (receiving the satellite's signal and retransmitting it to the UE via a PMCW link), and a bistatic echo collector for the satellite-illuminated target.
- **Ground UE (U):** A single-antenna mobile receiver. It receives downlink data from both the satellite directly ($S\!\to\!U$) and from the drone relay ($S\!\to\!D\!\to\!U$), and its position and velocity are estimated by the drone's radar.
- **Moving Target (T):** A passive aerial object with radar cross section $\sigma_T$, sensed through multiple radar paths: monostatic drone echo ($D\!\to\!T\!\to\!D$) and bistatic satellite-illuminated echo ($S\!\to\!T\!\to\!D$).

---

### 2.1 System Architecture: Communication and Sensing Roles

#### 2.1.1 Downlink Communication Streams

The UE receives downlink data via two parallel paths:

| Stream | Path | Waveform | Data Rate |
|---|---|---|---|
| **Direct** | $S\!\to\!U$ | OTFS ($f_{c,S}$, $B_S \sim 100\,\text{MHz}$) | $\sim\!100\,\text{Mbps}$ |
| **Relayed** | $S\!\to\!D\!\to\!U$ | OTFS ($S\!\to\!D$) + PMCW ($D\!\to\!U$) | $\sim\!10\text{–}100\,\text{kbps}$ |

**Same data or different data?**
In this system, both streams carry **the same payload data** (cooperative diversity / soft-combining relay). The two streams act as independently faded copies of the same bits: the direct satellite path and the relayed drone path experience different channel realizations, so the UE can apply Maximum Ratio Combining (MRC) across both received copies to improve reliability. This is especially beneficial in blockage-prone environments where the direct $S\!\to\!U$ link may be obstructed (e.g., terrain, foliage, urban canyons).

**Data rate mismatch and synchronization.** A key practical challenge is that the OTFS satellite link ($\sim\!100\,\text{Mbps}$) is several orders of magnitude faster than the PMCW relay link ($\sim\!10\text{–}100\,\text{kbps}$), since the PMCW link carries only one BPSK symbol $b_{D,m}$ per PRI ($T_{\mathrm{PRI},D} = N_D\,T_c$). Therefore the drone relay link is **not** used to relay the full satellite data payload. Instead, it carries:
- **HARQ feedback and retransmission bits:** When the UE fails to decode a satellite packet, the drone retransmits only the failed coded bits at low rate via PMCW, acting as a low-throughput HARQ helper.
- **Control and synchronization:** Ranging results, beam-management commands, timing reference signals.
 - **Cooperative combining pilots:** Known pilot symbols that allow the UE to estimate the drone-UE channel $h_{DU}$ and phase-align the relay signal for MRC combining with the direct satellite signal.
 For timing synchronization, the satellite broadcasts a global epoch reference (GPS-derived) inside every OTFS pilot frame. The drone aligns its PMCW PRI clock to the same reference, pre-compensating for the $S\!\to\!D$ propagation delay ($\approx\!2\,\text{ms}$ for LEO at $h\!=\!600\,\text{km}$) so that the relayed bits arrive at the UE co-temporally with the direct satellite packets.

#### 2.1.2 Active Sensing Paths

The table below lists all physically meaningful sensing paths in the four-node system and whether they are exploited or negligible:

| Path                                                                          | Type                   | Receiver      | Status                    | Purpose                                            |
| ----------------------------------------------------------------------------- | ---------------------- | ------------- | ------------------------- | -------------------------------------------------- |
| $D\!\to\!T\!\to\!D$                                                           | Monostatic             | Drone $D$     | **Active**                | High-resolution target range + velocity            |
| $S\!\to\!T\!\to\!D$                                                           | Bistatic               | Drone $D$     | **Active**                | Additional target geometry from bistatic geometry  |
| $D\!\to\!U\!\to\!D$                                                           | Cooperative echo       | Drone $D$     | **Active**                | UE range + velocity estimation                     |
| $S\!\to\!U\!\to\!D$                                                           | Bistatic UE reflection | Drone $D$     | **Negligible**            | Weak reflection off ground UE; below noise floor   |
| $S\!\to\!T\!\to\!S$                                                           | Echo to satellite      | Satellite $S$ | **Negligible** (see note) | —                                                  |
| $S\!\to\!U\!\to\!S$                                                           | Echo to satellite      | Satellite $S$ | **Negligible** (see note) | —                                                  |
| $S\!\to\!T\!\to\!U$                                                           | Scattered interference | UE $U$        | **Interference**          | Treated as residual noise at UE                    |
| $S\!\to\!U\!\to\!T$                                                           | Multi-bounce path      | None ($T$)    | **Negligible**            | No active receiver at target; power is negligible  |
| $D\!\to\!T\!\to\!U$                                                           | Bistatic target echo   | UE $U$        | **Interference**          | Treated as negligible co-channel interference      |
| $D\!\to\!U\!\to\!T$                                                           | Multi-bounce path      | None ($T$)    | **Negligible**            | No active receiver at target; power is negligible  |
| Other 4 hops such as $S\!\to\!T\!\to\!U\!\to\!D$, $S\!\to\!U\!\to\!T\!\to\!D$ | Triple-bounce paths    | Drone $D$     | **Negligible**            | Combined path loss of multiple bounces is too high |

 **Why echoes returning to the satellite are negligible.** The round-trip path loss for a signal originating at the LEO satellite ($h \approx 600\,\text{km}$, $f_{c,S} = 20\,\text{GHz}$) that scatters off a target and returns to the satellite is:
 $$L_\mathrm{bi} \approx \left(\frac{4\pi}{\lambda_S}\right)^2 d_{ST}^2 \approx 200\,\text{dB}$$compared to the drone monostatic path loss at $d_{DT} \approx 200\,\text{m}$:
$$L_\mathrm{mono} \approx \left(\frac{4\pi}{\lambda_D}\right)^2 d_{DT}^2 \approx 110\,\text{dB}$$
 The satellite-return echo is therefore approximately $90\,\text{dB}$ weaker than the drone monostatic echo, placing it $\sim\!90\,\text{dB}$ below the satellite receiver noise floor. It is physically undetectable. The satellite acts only as a **transmitter of opportunity** (passive bistatic illuminator); all echo collection is done at the drone $D$.

#### 2.1.3 Signal Separation Strategy

Three orthogonality mechanisms are combined to separate the signals at each receiver:

- **Waveform-domain separation:** Satellite links ($S\!\to\!U$, $S\!\to\!T$, $S\!\to\!D$) use OTFS to handle the large Doppler and delay spreads of LEO channels. Drone links ($D\!\to\!U$, $D\!\to\!T\!\to\!D$) use PMCW with MLS code $s_{\mathrm{mls},D}[n]$.
- **Code-domain orthogonality:** The quasi-orthogonality of $s_{\mathrm{mls},S}$ and $s_{\mathrm{mls},D}$ suppresses cross-code leakage at the drone receiver with a processing gain of $\min(N_S, N_D)$.
- **Delay-Doppler separation:** The direct-path satellite interference $y_{SD}$ at the drone appears at delay bin $\ell_{SD}$ and Doppler bin $f_{D,SD}$, which are generally well separated from the target bistatic echo at $(\ell_{\mathrm{bi}}, \nu_{\mathrm{bi}})$. Residual direct-path interference is mitigated by spatial nulling via the satellite beamforming vector $\mathbf{w}_S$ or by software cancellation in the bistatic processing branch.

---

### 2.2 Notation and Geometry

The instantaneous inter-node Euclidean distances are:

$$
d_{SU}(t) = \|\mathbf{p}_S(t) - \mathbf{p}_U(t)\|, \quad
d_{SD}(t) = \|\mathbf{p}_S(t) - \mathbf{p}_D(t)\|, \quad
d_{DU}(t) = \|\mathbf{p}_D(t) - \mathbf{p}_U(t)\|,
$$
$$
d_{ST}(t) = \|\mathbf{p}_S(t) - \mathbf{p}_T(t)\|, \quad
d_{DT}(t) = \|\mathbf{p}_D(t) - \mathbf{p}_T(t)\|,
$$

where $\mathbf{p}_i(t)\in\mathbb{R}^3$ is the position vector of node $i$ at time $t$.

The propagation delays for all active paths are:

$$
\tau_{SU}(t) = d_{SU}(t)/c_0, \qquad
\tau_{SD}(t) = d_{SD}(t)/c_0, \qquad
\tau_{DU}(t) = d_{DU}(t)/c_0,
$$
$$
\tau_{\mathrm{mono}}(t) = 2\,d_{DT}(t)/c_0, \qquad
\tau_{\mathrm{bi}}(t) = \bigl(d_{ST}(t)+d_{DT}(t)\bigr)/c_0, \qquad
\tau_{2DU}(t) = 2\,d_{DU}(t)/c_0.
$$

The Doppler shifts on each propagation link are obtained from the radial velocity projections:

$$
f_{D,SU}(t) = -\frac{(\mathbf{v}_U - \mathbf{v}_S)\cdot\hat{\mathbf{u}}_{SU}}{\lambda_S}, \qquad
f_{D,SD}(t) = -\frac{(\mathbf{v}_D - \mathbf{v}_S)\cdot\hat{\mathbf{u}}_{SD}}{\lambda_S},
$$
$$
f_{D,DU}(t) = -\frac{(\mathbf{v}_U - \mathbf{v}_D)\cdot\hat{\mathbf{u}}_{DU}}{\lambda_D}, \qquad
f_{D,ST}(t) = -\frac{(\mathbf{v}_T - \mathbf{v}_S)\cdot\hat{\mathbf{u}}_{ST}}{\lambda_S},
$$
$$
f_{D,DT}(t) = -\frac{(\mathbf{v}_T - \mathbf{v}_D)\cdot\hat{\mathbf{u}}_{DT}}{\lambda_D},
$$

where $\lambda_S = c_0/f_{c,S}$ and $\lambda_D = c_0/f_{c,D}$ are the satellite and drone carrier wavelengths, respectively. The cumulative two-way Doppler shifts for the sensing paths are:

$$
\nu_{\mathrm{mono}}(t) = 2\,f_{D,DT}(t), \qquad
\nu_{\mathrm{bi}}(t) = f_{D,ST}(t) + f_{D,DT}(t), \qquad
\nu_{DU}(t) = 2\,f_{D,DU}(t).
$$

---

### 2.3 Satellite Antenna Model

The satellite is equipped with a Uniform Linear Array (ULA) of $N_s$ elements with half-wavelength spacing $d = \lambda_S/2$. The transmit spatial steering vector toward direction $\vartheta$ is

$$
\mathbf{a}_{N_s}(\vartheta)
= \bigl[1,\;e^{j\pi\sin\vartheta},\;\dots,\;
         e^{j\pi(N_s-1)\sin\vartheta}\bigr]^T
\in \mathbb{C}^{N_s \times 1}.
$$

A transmit beamforming vector $\mathbf{w}_S \in \mathbb{C}^{N_s\times 1}$ (with $\|\mathbf{w}_S\|=1$) is applied to focus energy toward the UE and simultaneously illuminate the target.

---

### 2.4 Transmit Signals

**Satellite Transmit Signal.**
The satellite modulates the BPSK communication symbol sequence $\{b_m\}$, $b_m\in\{+1,-1\}$, onto a length-$N_S$ MLS spreading code $s_{\mathrm{mls},S}[n]\in\{+1,-1\}$ and applies spatial beamforming. The discrete-time baseband transmit signal vector is

$$
\mathbf{x}_S[n,m]
= \sqrt{P_S}\,\mathbf{w}_S\,s_{\mathrm{mls},S}[n]\,b_m
\;\in\;\mathbb{C}^{N_s\times 1},
$$

where $P_S$ is the satellite transmit power and the PRI is $T_{\mathrm{PRI},S} = N_S\,T_c$.

**Drone Transmit Signal.**
The drone transmits a PMCW JRC waveform using a distinct quasi-orthogonal MLS code $s_{\mathrm{mls},D}[n]\in\{+1,-1\}$ of length $N_D$, modulated by its own BPSK data symbol $b_{D,m}\in\{+1,-1\}$:

$$
x_D[n,m]
= \sqrt{P_D}\,s_{\mathrm{mls},D}[n]\,b_{D,m},
$$

where $P_D$ is the drone transmit power and the PRI is $T_{\mathrm{PRI},D} = N_D\,T_c$. The symbol $b_{D,m}$ carries the relayed data bits (decoded from the satellite's signal) to the UE, and also serves as the known BPSK sequence for radar data-stripping at the drone receiver.

---

### 2.5 Channel Models

**Space-to-Ground/Mid-Air Channels (MISO Rician).**
Due to the high LEO altitude, all satellite links ($S\!\to\!U$, $S\!\to\!T$, $S\!\to\!D$) experience a high Rician $K$-factor. The MISO channel row vector $\mathbf{h}_{Sj}^H\in\mathbb{C}^{1\times N_s}$ is modeled as

$$
\mathbf{h}_{Sj}^H
= \sqrt{\beta_{Sj}}\,e^{j\phi_{Sj}}\,e^{j2\pi f_{D,Sj}\,t}
  \!\bigg(
    \sqrt{\tfrac{K_{Sj}}{K_{Sj}+1}}\,\mathbf{a}_{N_s}^H(\vartheta_S^{j})
    + \sqrt{\tfrac{1}{K_{Sj}+1}}\,\mathbf{g}_{Sj}^H
  \bigg),
$$

where $\beta_{Sj}$ is the large-scale path gain, $\phi_{Sj}$ is a random initial phase, $K_{Sj}$ is the Rician factor, $\vartheta_S^{j}$ is the Angle of Departure (AoD) from the satellite toward node $j$, and $\mathbf{g}_{Sj}^H\sim\mathcal{CN}(\mathbf{0},\mathbf{I}_{N_s})$ represents the scattered NLoS components. Specifically:

| Symbol | Description |
|---|---|
| $\mathbf{h}_{SU}^H$ | Satellite-to-UE channel |
| $\mathbf{h}_{ST}^H$ | Satellite-to-target channel |
| $\mathbf{h}_{SD}^H$ | Satellite-to-drone direct channel |

**Aerial/Terrestrial Channels (SISO).**
All links between the drone and ground/aerial nodes ($D\!\to\!U$, $D\!\to\!T$, $T\!\to\!D$) are SISO channels modeled as in Section 1.2:

$$
h_{DU}(t) = \sqrt{\beta_{DU}}\,g_{DU}\,e^{j2\pi f_{D,DU}\,t}, \qquad
h_{DT}(t) = \sqrt{\beta_{DT}}\,g_{DT}\,e^{j2\pi f_{D,DT}\,t},
$$
$$
h_{TD}(t) = \sqrt{\beta_{TD}}\,g_{TD}\,e^{j2\pi f_{D,DT}\,t}.
$$

Under channel reciprocity, $\beta_{TD}=\beta_{DT}$ and $g_{TD}=g_{DT}$.

---

### 2.6 Received Signal at the UE

The UE receives the superposition of the direct satellite downlink signal and the drone relay signal:

$$
y_U[n,m]
= \underbrace{
    \mathbf{h}_{SU}^H\,\mathbf{x}_S[n-\ell_{SU},\,m]\,
    e^{j2\pi f_{D,SU}\,m\,T_{\mathrm{PRI},S}}
  }_{\text{direct satellite downlink } (S\to U)}
+ \underbrace{
    h_{DU}\,x_D[n-\ell_{DU},\,m]\,
    e^{j2\pi f_{D,DU}\,m\,T_{\mathrm{PRI},D}}
  }_{\text{drone relay signal } (S\to D\to U)}
+ n_U[n,m],
$$

where $\ell_{SU}=\mathrm{round}(\tau_{SU}\,f_s)$, $\ell_{DU}=\mathrm{round}(\tau_{DU}\,f_s)$, and $n_U[n,m]\sim\mathcal{CN}(0,\sigma_U^2)$ is the UE receiver noise. Substituting the transmit signals yields:

$$
y_U[n,m]
= \underbrace{\sqrt{P_S}\,\bigl(\mathbf{h}_{SU}^H\mathbf{w}_S\bigr)\,
      s_{\mathrm{mls},S}\!\bigl[(n-\ell_{SU})_{N_S}\bigr]\,b_m\,
      e^{j2\pi f_{D,SU}\,m\,T_{\mathrm{PRI},S}}}_{\text{direct satellite stream}}
+ \underbrace{\sqrt{P_D}\,h_{DU}\,
      s_{\mathrm{mls},D}\!\bigl[(n-\ell_{DU})_{N_D}\bigr]\,b_{D,m}\,
      e^{j2\pi f_{D,DU}\,m\,T_{\mathrm{PRI},D}}}_{\text{drone relay stream}}
+ n_U[n,m].
$$

Since both streams carry the same payload data ($b_m \equiv b_{D,m}$ after the drone decodes and re-encodes), the UE applies MRC combining after independently despreading each stream: the satellite stream is despread by correlating with $s_{\mathrm{mls},S}$, and the drone relay stream by correlating with $s_{\mathrm{mls},D}$. The two codes are quasi-orthogonal, so each despreading step suppresses the other stream's leakage by a processing gain of $\min(N_S, N_D)$.

---

### 2.7 Received Signal at the Drone Receiver (Sensing)

The drone receiver collects four signal contributions: the monostatic target echo ($D\!\to\!T\!\to\!D$), the cooperative UE echo ($D\!\to\!U\!\to\!D$), the bistatic satellite-illuminated target echo ($S\!\to\!T\!\to\!D$), and the direct-path satellite interference ($S\!\to\!D$):

$$
y_D[n,m]
= y_{DTD}[n,m]
+ y_{DUD}[n,m]
+ y_{STD}[n,m]
+ y_{SD}[n,m]
+ n_D[n,m],
$$

where $n_D[n,m]\sim\mathcal{CN}(0,\sigma_D^2)$ is the drone receiver noise.

**Monostatic Target Echo ($D\!\to\!T\!\to\!D$).**
The drone's own PMCW waveform reflects from the moving target and returns to the drone. Under channel reciprocity ($\beta_{TD}=\beta_{DT}$, $g_{TD}=g_{DT}$):

$$
y_{DTD}[n,m]
= \eta_T\,\beta_{DT}\,g_{DT}^2\,
  \sqrt{P_D}\,s_{\mathrm{mls},D}\!\bigl[(n-\ell_{\mathrm{mono}})_{N_D}\bigr]
  \,b_{D,m}\,e^{j2\pi \nu_{\mathrm{mono}} \,m\,T_{\mathrm{PRI},D}},
$$

where $\ell_{\mathrm{mono}}=\mathrm{round}(\tau_{\mathrm{mono}}\,f_s)$ and $\nu_{\mathrm{mono}} = 2f_{D,DT}$ is the two-way monostatic Doppler shift.

**Cooperative UE Echo ($D\!\to\!U\!\to\!D$).**
The drone's PMCW waveform reflects off the UE and returns to the drone, enabling UE localization:

$$
y_{DUD}[n,m]
= \eta_U\,\beta_{DU}\,g_{DU}^2\,
  \sqrt{P_D}\,s_{\mathrm{mls},D}\!\bigl[(n-\ell_{2DU})_{N_D}\bigr]
  \,b_{D,m}\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI},D}},
$$

where $\ell_{2DU}=\mathrm{round}(2\tau_{DU}\,f_s)$ and $\nu_{DU} = 2f_{D,DU}$ is the round-trip Doppler shift.

**Bistatic Target Echo ($S\!\to\!T\!\to\!D$).**
The satellite signal illuminates the target; the bistatic reflected echo is collected by the drone. Since the drone knows the satellite's transmitted waveform (from the $S\!\to\!D$ direct path), it can perform bistatic matched filtering:

$$
y_{STD}[n,m]
= \eta_T\,\sqrt{\beta_{ST}\,\beta_{DT}}\,g_{ST}\,g_{DT}\,
  \sqrt{P_S}\,\bigl(\mathbf{h}_{ST}^H\mathbf{w}_S\bigr)
  \,s_{\mathrm{mls},S}\!\bigl[(n-\ell_{\mathrm{bi}})_{N_S}\bigr]\,b_m\,
  e^{j2\pi\nu_{\mathrm{bi}}\,m\,T_{\mathrm{PRI},S}},
$$

where $\ell_{\mathrm{bi}}=\mathrm{round}(\tau_{\mathrm{bi}}\,f_s)$ and $\nu_{\mathrm{bi}} = f_{D,ST} + f_{D,DT}$ is the bistatic Doppler shift.

**Direct-Path Satellite Signal (Interference at Drone).**
The satellite's downlink transmission is received directly at the drone, constituting strong direct-path interference for the bistatic sensing branch:

$$
y_{SD}[n,m]
= \sqrt{P_S}\,\bigl(\mathbf{h}_{SD}^H\mathbf{w}_S\bigr)\,
  s_{\mathrm{mls},S}\!\bigl[(n-\ell_{SD})_{N_S}\bigr]
  \,b_m\,
  e^{j2\pi f_{D,SD}\,m\,T_{\mathrm{PRI},S}},
$$

where $\ell_{SD}=\mathrm{round}(\tau_{SD}\,f_s)$.

Combining all contributions, the complete received signal at the drone is:

$$
y_D[n,m]
= \underbrace{
    \eta_T\,\beta_{DT}\,g_{DT}^2\,\sqrt{P_D}\,
    s_{\mathrm{mls},D}\!\bigl[(n-\ell_{\mathrm{mono}})_{N_D}\bigr]
    \,b_{D,m}\,e^{j2\pi\nu_{\mathrm{mono}}\,m\,T_{\mathrm{PRI},D}}
  }_{D\to T\to D\text{ monostatic echo}}
+ \underbrace{
    \eta_U\,\beta_{DU}\,g_{DU}^2\,\sqrt{P_D}\,
    s_{\mathrm{mls},D}\!\bigl[(n-\ell_{2DU})_{N_D}\bigr]
    \,b_{D,m}\,e^{j2\pi\nu_{DU}\,m\,T_{\mathrm{PRI},D}}
  }_{D\to U\to D\text{ cooperative UE echo}}
+ \underbrace{
    \eta_T\sqrt{\beta_{ST}\beta_{DT}}\,g_{ST}g_{DT}\,\sqrt{P_S}\,
    (\mathbf{h}_{ST}^H\mathbf{w}_S)\,b_m
    \,s_{\mathrm{mls},S}\!\bigl[(n-\ell_{\mathrm{bi}})_{N_S}\bigr]
    \,e^{j2\pi\nu_{\mathrm{bi}}\,m\,T_{\mathrm{PRI},S}}
  }_{S\to T\to D\text{ bistatic echo}}
+ \underbrace{
    \sqrt{P_S}\,(\mathbf{h}_{SD}^H\mathbf{w}_S)\,
    s_{\mathrm{mls},S}\!\bigl[(n-\ell_{SD})_{N_S}\bigr]\,b_m\,
    e^{j2\pi f_{D,SD}\,m\,T_{\mathrm{PRI},S}}
  }_{S\to D\text{ direct-path interference}}
+ n_D[n,m].
$$

 **Sensing signal separation at the drone.** The drone receiver separates the four contributions using two processing branches:
 - **PMCW branch** (correlate with $s_{\mathrm{mls},D}$, after multiplying by known $b_{D,m}$): extracts the monostatic target echo $y_{DTD}$ and the cooperative UE echo $y_{DUD}$. These two echoes share the same code but are separated in the delay-Doppler domain at bins $(\ell_{\mathrm{mono}}, \nu_{\mathrm{mono}})$ and $(\ell_{2DU}, \nu_{DU})$, respectively.
 - **OTFS/satellite branch** (correlate with $s_{\mathrm{mls},S}$, after multiplying by known $b_m$): extracts the bistatic echo $y_{STD}$ and also reveals the direct-path interference $y_{SD}$ at a known delay-Doppler location $(\ell_{SD}, f_{D,SD})$, which is then subtracted via software cancellation.
