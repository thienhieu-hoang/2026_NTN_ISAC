*What updated from v7_2*
# Sounding Configuration [a b] & Doppler-based Channel Prediction at UE

This  details the implementation of a flexible sounding configuration `[a b]` for sparse sounding in the NTN-ISAC simulation. It implements a Doppler prediction scheme at the UE using two sounding blocks to estimate the Doppler frequency, and then predicts the channel coefficients for the subsequent non-sounding blocks. Finally, it Sweeps and compares the Bit Error Rate (BER) performance for configurations `[2, 9]`, `[2, 17]`, and `[2, 33]` under both Jakes and AR(1) fading models against the benchmark curves.

---

### Component: System Parameters and Signal Construction

We will modify `SystemParams.m` to include the sounding configuration parameter `sounding_config` and update the signal generation and demodulation code to use it.

#### [MODIFY] [SystemParams.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/main/+ntn/SystemParams.m)
- Add property `sounding_config = [2, 9]` (default value).
- Keep `L_sound` as a fallback or remove it if not needed, but keeping it for backward compatibility is safer.

#### [MODIFY] [TransmitSignal.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/main/+ntn/TransmitSignal.m)
- Change slot determination logic to use `sounding_config = [a b]`.
- Specifically, block `m` is a sounding block if `mod(m-1, b-1) == 0` or `mod(m-1, b-1) == a-1`. If it is a sounding block, the first slot (sounding slot) is populated with a pilot symbol (no data modulation). All other slots are communication slots.

#### [MODIFY] [Demodulator.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/main/+ntn/+comms/Demodulator.m)
- Update slot identification logic to match the new definition of sounding blocks using `sounding_config`.

#### [MODIFY] [DroneReceiver.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/main/+ntn/+sensing/DroneReceiver.m)
- Update the modulation stripping logic (which determines sounding slots) to use `sounding_config` to avoid any syntax or functional errors if this class is ever instantiated.

---

### Component: Channel Estimation and Prediction

#### [MODIFY] [ChannelEstimator.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/main/+ntn/+comms/ChannelEstimator.m)
- Replace the zero-order hold channel estimation with the Doppler-based channel prediction:
  1. Identify sounding blocks: $S_k = (k-1)(b-1)+1$ and $S'_k = (k-1)(b-1)+a$.
  2. For each period $k$:
     - Use channel estimates $h_1 = h(S_k)$ and $h_2 = h(S'_k)$.
     - Compute the phase difference $\Delta \theta = \angle(h_2 \cdot h_1^*)$.
     - Compute the block-to-block phase rotation $\psi = \Delta \theta / (a - 1)$.
     - For any blocks $m$ between $S_k$ and $S'_k$ (if any), interpolate/rotate:
       $$\hat{h}(m) = \left( |h_1| + (|h_2| - |h_1|) \frac{m - S_k}{a - 1} \right) \exp\left(j (\angle h_1 + \psi (m - S_k))\right)$$
     - For blocks $m$ in the prediction range $S'_k < m \le \text{end\_of\_period}$, extrapolate phase:
       $$\hat{h}(m) = h_2 \exp\left(j \psi (m - S'_k)\right)$$
     - Correctly handle the boundary conditions at the end of the block sequence $M$.

---

### Component: Simulation Driver

#### [MODIFY] [run_simulation_sparse_sounding.m](file:///c:/Users/AT30890/polymtl/Tri%20Nhu%20Do%20-%20NTN_ISAC/main/run_simulation_sparse_sounding.m)
- Update the driver to sweep through configurations `[2, 9]`, `[2, 17]`, and `[2, 33]`.
- Add support for running both `ar1` and `jakes` fading models.
- Save and plot the simulated BER results for all configurations and fading models alongside the perfect CSI theory and sounding-every-block benchmarks.
- Output comparison plots as PDF/PNG files.

