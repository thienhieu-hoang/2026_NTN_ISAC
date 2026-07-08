This is an excellent question that touches on the core physical and signal-processing limitations of joint radar-communication systems. Let's break down the mathematics of the received signal, detail the drone's feedback overhead, explain how the UE uses it, and address why the standard cross-correlation method fails to resolve the two taps on its own.

---

### 1. Why Can’t the UE Simply Extract the Taps from standard Cross-Correlation?

In theory, the cross-correlation profile of the sounding period should show two peaks corresponding to the two paths. In practice, however, the UE cannot resolve the second (scattered) path due to two major physical bottlenecks:

#### Bottleneck A: The "Near-Far" Dynamic Range Problem
*   The direct path ($D \to U$) is a **1-hop propagation link**: path loss is proportional to $1/d_{DU}^2$.
*   The scattered path ($D \to T \to U$) is a **2-hop bistatic radar link**: path loss is proportional to $1/(d_{DT}^2 \cdot d_{TU}^2)$.
*   Because of the 2-hop propagation and the target's radar cross-section (RCS) scattering loss (as detailed in [Reflecting parameters.md](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/Reflecting%20parameters.md)), the scattered path is typically **$20\text{ dB}$ to $40\text{ dB}$ weaker** than the direct path at the UE.
*   A Maximum-Length Sequence (MLS/PRBS) of length $N_D$ has an auto-correlation sidelobe level of approximately $-10\log_{10}(N_D)\text{ dB}$ (e.g., $\approx -21\text{ dB}$ for $N_D = 127$). 
*   **The consequence:** The peak of the weak scattered path is completely buried under the **auto-correlation sidelobes of the strong direct path**. The UE cannot see it as a distinct tap in the raw correlation profile.

#### Bottleneck B: Doppler Smearing
*   Under high-speed drone/target scenarios, the cumulative Doppler shift of the scattered path ($\nu_{DTU} = f_{D,DT} + f_{D,TU}$) is large. 
*   MLS/PRBS codes are sensitive to Doppler shifts. Without prior Doppler compensation, a large Doppler shift rotates the phase of the chips across the sequence, destroying the correlation peak and spreading its energy into noise-like sidelobes across all delay bins.

---

### 2. Received Signal at the UE (Mathematics)

During the **sounding period** of block $m$, the drone transmits the unmodulated sequence $s_{\mathrm{mls}}[n]$ ($d_m = +1$). The received signal at the UE is:

$$y_U[n,m] = A_{DU} \cdot s_{\mathrm{mls}}\!\bigl[(n-\ell_{DU})_{N_D}\bigr] \cdot e^{j2\pi f_{D,DU} \cdot m T_{\mathrm{block}}} + A_{DTU} \cdot s_{\mathrm{mls}}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr] \cdot e^{j2\pi \nu_{DTU} \cdot m T_{\mathrm{block}}} + n_U[n,m]$$

Where:
*   $s_{\mathrm{mls}}[n]$ is the reference sequence of length $N_D$.
*   $A_{DU} = \sqrt{P_D \beta_{DU}} g_{DU}$ is the complex amplitude of the direct path.
*   $A_{DTU} = \eta_T \sqrt{P_D \beta_{DT} \beta_{TU}} g_{DT} g_{TU}$ is the complex amplitude of the scattered path (typically $|A_{DTU}| \ll |A_{DU}|$).
*   $\ell_{DU}$ and $\ell_{DTU}$ are the integer chip delays of the two paths.
*   $f_{D,DU}$ is the direct-path Doppler, and $\nu_{DTU} = f_{D,DT} + f_{D,TU}$ is the scattered-path Doppler.

---

### 3. Drone Feedback (Light Overhead)

The Drone transmits just two scalar parameters to the UE (e.g., in a downlink control packet sent at a much slower rate than the symbol/frame rate):
1.  **$d_{DT}$:** The drone-to-target distance (sensed monostatically).
2.  **$f_{D,DT}$:** The target's Doppler shift relative to the drone.

---

### 4. What the UE Does with This Information (Mathematics)

Armed with $d_{DT}$ and $f_{D,DT}$, the UE can execute a targeted multi-stage estimation and cancellation process:

#### Step 4.1: Subtracting the Dominant Direct Path (First-Stage SIC)
Since the direct path is strong, the UE easily estimates its parameters ($\hat{A}_{DU}$, $\hat{\ell}_{DU}$, $\hat{f}_{D,DU}$) from the primary correlation peak. It then reconstructs and subtracts it to expose the weak scattered path:

$$z[n,m] = y_U[n,m] - \hat{A}_{DU} \cdot s_{\mathrm{mls}}\!\bigl[(n-\hat{\ell}_{DU})_{N_D}\bigr] \cdot e^{j2\pi \hat{f}_{D,DU} \cdot m T_{\mathrm{block}}}$$

This removes the direct path's peak and, crucially, **its high-level correlation sidelobes**, revealing the residual signal:

$$z[n,m] \approx A_{DTU} \cdot s_{\mathrm{mls}}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr] \cdot e^{j2\pi \nu_{DTU} \cdot m T_{\mathrm{block}}} + n_U[n,m]$$

#### Step 4.2: Doppler Pre-Compensation
To prevent Doppler smearing, the UE uses the drone's feedback $f_{D,DT}$ to de-rotate the residual signal:

$$z_{\mathrm{comp}}[n,m] = z[n,m] \cdot e^{-j2\pi f_{D,DT} \cdot m T_{\mathrm{block}}} \approx A_{DTU} \cdot s_{\mathrm{mls}}\!\bigl[(n-\ell_{DTU})_{N_D}\bigr] \cdot e^{j2\pi f_{D,TU} \cdot m T_{\mathrm{block}}} + \tilde{n}_U[n,m]$$

Because the target-to-drone Doppler component ($f_{D,DT}$) has been compensated, the remaining Doppler rotation is only $f_{D,TU}$, which is significantly smaller.

#### Step 4.3: Bounded Delay-Doppler Correlation
Instead of searching over the entire code length $N_D$, the UE bounds the search window for $\ell_{DTU}$ using the triangle inequality:
$$d_{DTU} = d_{DT} + d_{TU}$$

Since $d_{DT}$ is known from drone feedback and $d_{TU}$ must satisfy $|d_{DU} - d_{DT}| \le d_{TU} \le d_{DU} + d_{DT}$, the search is confined to a tiny range of chips:
$$\ell_{DTU} \in \left[ \text{round}\left(\frac{d_{DT} + |d_{DU} - d_{DT}|}{c_0} f_s\right), \; \text{round}\left(\frac{d_{DT} + d_{DU} + d_{DT}}{c_0} f_s\right) \right]$$

The UE then correlates within this narrow window to extract the exact delay $\hat{\ell}_{DTU}$ and the remaining Doppler $\hat{f}_{D,TU}$:

$$R(\ell, f) = \sum_{m} \sum_{n} z_{\text{comp}}[n,m] \cdot s_{\mathrm{mls}}^*\!\bigl[(n - \ell)_{N_D}\bigr] \cdot e^{-j 2 \pi f \cdot m T_{\mathrm{block}}}$$

Once the peak is identified, the UE obtains:
*   The exact scattered delay: $\hat{\ell}_{DTU}$
*   The total scattered Doppler: $\hat{\nu}_{DTU} = f_{D,DT} + \hat{f}_{D,TU}$
*   The scattered path amplitude: $\hat{A}_{DTU}$

#### Step 4.4: Active Scatter Path Suppression (During Data Period)
During the data block when symbols $d_m \in \{+1, -1\}$ are transmitted, the UE performs Successive Interference Cancellation (SIC):
1.  It makes a tentative bit decision $\hat{d}_m$ based on the direct path.
2.  It reconstructs the scattered interference:
    $$\hat{y}_{DTU}[n,m] = \hat{A}_{DTU} \cdot s_{\mathrm{mls}}\!\bigl[(n-\hat{\ell}_{DTU})_{N_D}\bigr] \cdot \hat{d}_m \cdot e^{j2\pi \hat{\nu}_{DTU} \cdot m T_{\mathrm{block}}}$$
3.  It subtracts the reconstructed interference:
    $$y_{\text{cleaned}}[n,m] = y_{U,\text{data}}[n,m] - \hat{y}_{DTU}[n,m]$$
4.  It performs the final, clean demodulation of the communication payload from $y_{\text{cleaned}}[n,m]$.