For your PMCW joint radar-communication problem, you should use **Scattering Parameters linked to the Radar Cross Section (RCS)**. 

Here is the physical justification and the exact values you should set in your code.

---

### Why Scattering Parameters Linked to RCS?
Your system model defines the one-way path loss as the dimensionless Friis transmission equation:
$$\beta_{ij} = \left( \frac{\lambda_D}{4\pi d_{ij}} \right)^2$$

When modeling the monostatic radar path ($D \to T \to D$), the signal goes through two hops, resulting in a propagation loss of $\beta_{DT}^2 \propto \frac{\lambda_D^4}{d_{DT}^4}$. 

To make this physically match the standard **Radar Range Equation** (which relates to the physical target area $\sigma_T$ in $\text{m}^2$ and has a $\lambda_D^2$ dependency):
$$P_{r,\text{radar}} = \frac{P_D \lambda_D^2 \sigma_T}{(4\pi)^3 d_{DT}^4}$$

The reflection/scattering parameter $\eta_T$ must act as a scaling factor that converts the communication-style path loss into the radar equation. This requires:
$$|\eta_T|^2 = \frac{4\pi \sigma_T}{\lambda_D^2}$$

This mathematical conversion works **perfectly** for all three links:
1.  **Monostatic Target Sensing ($D \to T \to D$):** Yields the standard radar range equation.
2.  **Monostatic UE Scattering ($D \to U \to D$):** Yields the radar range equation for the UE backscatter.
3.  **Bistatic Target Scattering to UE ($D \to T \to U$):** Yields the standard **bistatic radar equation**, which models the scattered interference power at the UE.

If you used surface reflection coefficients ($\Gamma \le 1$), the model would violate the physics of radar propagation, and your targets would behave like microscopic dust particles ($10^{-11}\text{ m}^2$) instead of physical objects.

---

### Recommended Values to Use

At $24\text{ GHz}$ ($\lambda_D = 12.5\text{ mm}$), here are the recommended physical values based on electromagnetic measurements in the literature:

#### 1. Target (Moving Object T)
*   **Option 1: A Bird (Most Common)**
    *   **Physical RCS ($\sigma_T$):** $-30\text{ to } -20\text{ dBsm}$ ($10^{-3} \text{ to } 10^{-2}\text{ m}^2$).
    *   **Value to use ($|\eta_T|$):** **$\approx 9.0$** (for a pigeon-sized bird at $10^{-3}\text{ m}^2$).
*   **Option 2: A Small Drone/UAV**
    *   **Physical RCS ($\sigma_T$):** $-20\text{ to } -10\text{ dBsm}$ ($10^{-2} \text{ to } 10^{-1}\text{ m}^2$).
    *   **Value to use ($|\eta_T|$):** **$\approx 28.4$** (for a standard hobbyist drone at $10^{-2}\text{ m}^2$).

#### 2. UE (User Equipment + Human User)
*   **Physical RCS ($\sigma_U$):** $-10\text{ to } 0\text{ dBsm}$ ($10^{-1} \text{ to } 1\text{ m}^2$). Because the user is holding the phone, the human body's surface area dominates the radar backscatter.
*   **Value to use ($|\eta_U|$):** **$\approx 89.7$** (for a low-end estimation of $10^{-1}\text{ m}^2$) or **$\approx 283.6$** (for a full-body reflection of $1.0\text{ m}^2$).

---

### Suggested Code Implementation
You can replace the definition of `eta_T` and `eta_U` in [+ntn/ChannelModel.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/clean_syncronization/+ntn/ChannelModel.m#L53-L54) with:

```matlab
            % Set physical RCS values (in m^2)
            sigma_T = 1e-2;     % Target RCS: -20 dBsm (Small UAV/Drone)
            sigma_U = 1e-1;     % UE RCS: -10 dBsm (Phone + User Hand/Body)
            
            % Compute corresponding scattering parameters
            obj.eta_T = sqrt(4 * pi * sigma_T) / p.lamD * exp(1j * 2 * pi * rand);
            obj.eta_U = sqrt(4 * pi * sigma_U) / p.lamD * exp(1j * 2 * pi * rand);
```

### Impact on Your Simulation
1.  **Target SNR:** Because your code scales the receiver noise `sigD` relative to `aT` (the target amplitude), the target's post-processing SNR will remain at your configured $18\text{ dB}$.
2.  **UE-to-Target Interference Ratio:** The power ratio at the drone receiver will become:
    $$\frac{P_{r,U}}{P_{r,T}} = \frac{\sigma_U}{\sigma_T} \left( \frac{d_{DT}}{d_{DU}} \right)^4 \approx \frac{0.1}{0.01} \left( \frac{126.9}{205.9} \right)^4 \approx 1.44 \text{ or } +1.6\text{ dB}$$
    This ratio is much more physically realistic than the artificial $+41.1\text{ dB}$ ratio. In this case, the target signal is much stronger relative to the UE interference, making detection and Successive Interference Cancellation (SIC) easier.