# NTN-ISAC PMCW Simulation: Running & Configuration Guide

---

## Quick Reference Summary Table

| Script Name | Purpose | Key Parameters | Expected Outputs |
| :--- | :--- | :--- | :--- |
| [`run_plot_geometry.m`] | Visualizes the 3D position coordinates of Drone, UE, and Target. | None (reads defaults from `SystemParams`) | Console positions, PDF plot: `results/system_geometry.pdf` |
| [`run_visualize_waveforms.m`] | Plots time and frequency representations of joint PMCW waveforms. | `M_plot`, `S_per_chip`, `sounding_config` | PDF plots: `waveform_visualization_time.pdf`, `waveform_visualization_freq.pdf` |
| [`run_visualize_waveforms_custom.m`] | Plots block-level structural layouts of sounding vs comms slots. | `M_plot`, `sounding_config`, `L_sound` | PDF plot: `waveform_custom_visualization.pdf` |
| [`run_sensing_observation.m`] | Animates dynamic Range-Doppler heatmaps before and after SIC. | `window_mode`, `window_size`, `EbN0_dB`, `SNR_target_dB` | Screen animation, GIF: `sensing_observation.gif`, snaps in `results/` |
| [`run_simulation_ber_only.m`] | Performs communication BPSK BER sweeps for Gaussian/Rayleigh channels. | Fading model (via constructor: `'static'`, `'ar1'`, `'jakes'`) | Console logs, PDF plot: `results/BER/ue_ber_comms_only.pdf`, `.mat` data |
| [`run_simulation_zoh_sounding.m`] | Sweeps sparse sounding intervals under Zero-Order Hold (ZOH). | `fading_models`, `L_sound_values` | Comparison plot: `ue_ber_zoh_sounding_comparison.pdf`, `.mat` files |
| [`run_simulation_Doppler_interpolate_sounding.m`] | Sweeps sparse sounding configs using Doppler-phase interpolation. | `fading_models`, `sounding_configs` (e.g. `[2 9]`, `[2 17]`) | Comparison plot: `ue_ber_sparse_sounding_comparison_configs.pdf`, `.mat` |
| [`run_simulation_nakagami.m`] | Runs communication BPSK BER sweeps for Nakagami-\(m\) fading. | `params.m_nakagami`, `fading_model` | Comparison plot: `ue_ber_comms_only_nakagami_m_*.pdf`, `.mat` data |

---

## Detailed Script Configurations

### 1. `run_plot_geometry.m`
* **Purpose**: Simple verification script to plot the spatial geometry of the 3-node system (Drone Tx, UE Rx, Target Reflector).
* **Parameters**: 
  - Uses defaults configured in `ntn.SystemParams` and `ntn.Geometry`.
* **Results**: 
  - Prints coordinate details to the command window.
  - Saves the 3D plot to: `main/results/system_geometry.pdf`.

---

### 2. `run_visualize_waveforms.m`
* **Purpose**: Generates high-fidelity vector graphics showing the joint PMCW radar-communications waveform. It includes subplots for baseband bits, spreading sequence, baseband product, and zoomed view of chip shapes, plus frequency-domain power spectral densities (PSD) for $+1$ and $-1$ slots.
* **Configurable Parameters** (Lines 19-21):
  - `M_plot`: Number of slow-time blocks to display in the time domain (default: `3`).
  - `S_per_chip`: Upsampling factor for plotting smooth rectangular pulse shapes (default: `1`).
  - `sounding_config`: Sounding configuration `[a b]`. Set to `[]` to enable sounding in every block.
* **Results**: 
  - Saves time-domain waveform analysis to: `main/results/waveform_visualization_time.pdf`.
  - Saves PSD spectrum graphs to: `main/results/waveform_visualization_freq.pdf`.

---

### 3. `run_visualize_waveforms_custom.m`
* **Purpose**: Generates structural block diagrams showing the time allocation of sounding slots (in blue) and communication slots (in black) over slow-time blocks.
* **Configurable Parameters** (Lines 21-23):
  - `M_plot`: Number of blocks to plot along the horizontal axis (default: `10`).
  - `sounding_config`: Sparse sounding config `[a b]` (e.g. `[2 9]`). If set to `[]`, the script falls back to periodic interval `L_sound`.
  - `L_sound`: Sounding slot period interval (used only if `sounding_config` is empty).
* **Results**: 
  - Saves the slot allocation diagram to: `main/results/waveform_custom_visualization.pdf`.

---

### 4. `run_sensing_observation.m`
* **Purpose**: Runs a dynamic simulation loop animating the range-Doppler sensing heatmap at the Drone side. Shows the signal *before* and *after* Successive Interference Cancellation (SIC) of the strong target reflection, verifying detection of the weaker secondary target.
* **Configurable Parameters** (Lines 19-28):
  - `window_mode`: `'sliding'` (a moving window of fixed size) or `'growing'` (a window expanding from a minimum length).
  - `window_size`: Size of integration window in blocks for sliding mode (default: `64`).
  - `step_size`: Frequency of heatmap updates in blocks (default: `2`).
  - `pause_duration`: Time in seconds between animation frames (default: `0.1`).
  - `EbN0_dB`: Communication SNR at the UE (default: `15` dB).
  - `SNR_target_dB`: Radar target single-period SNR at the Drone (default: `18` dB).
  - `save_gif`: Set to `true` to save the dynamic animation loop as a GIF (default: `true`).
* **Results**:
  - Interactive screen animation displaying Range-Doppler heatmaps before/after SIC side-by-side.
  - Saves three intermediate snapshot plots to: `main/results/sensing_snapshot_{early,mid,late}.pdf`.
  - Saves the completed animation loop to: `main/results/sensing_observation.gif`.

---

### 5. `run_simulation_ber_only.m`
* **Purpose**: Simulates the communication Bit Error Rate (BER) versus $E_b/N_0$ (0 to 20 dB). Runs Monte Carlo trials to plot simulation lines against theoretical AWGN/Rayleigh BPSK curves.
* **Configurable Parameters**:
  - The script runs the **non-fading (Gaussian)** or **time-varying (Rayleigh)** simulation based on the `BERAnalysis` constructor argument.
  - To run **AWGN (Gaussian)**:
    - Line 23: `ber = ntn.comms.BERAnalysis();`
    - Line 68: `fading = ntn.FadingSequence(params, geom, 2 * params.M);`
  - To run **Rayleigh Fading (block-varying)**:
    - Line 23: `ber = ntn.comms.BERAnalysis('ar1');`
    - Line 68: `fading = ntn.FadingSequence(params, geom, 2 * params.M, 'ar1');`
* **Results**:
  - Plots BER (Theory Perfect CSI, Theory Noisy Pilot, and Simulation Average) and saves to: `main/results/BER/ue_ber_comms_only.pdf`.
  - Saves simulation data to: `main/results/BER/ue_ber_comms_only_gaussian.mat` (or `_rayleigh.mat`).

---

### 6. `run_simulation_zoh_sounding.m`
* **Purpose**: Simulates and compares BER curves for sparse sounding intervals using a Zero-Order Hold (ZOH) channel estimate.
* **Configurable Parameters** (Lines 40-43):
  - `fading_models`: Cell array of fading environments to evaluate (e.g. `{'static'}`, `{'ar1'}`, or `{'jakes'}`).
  - `L_sound_values`: Sounding intervals to evaluate (default: `[8, 16]`).
* **Results**:
  - For each fading model and sounding interval, saves raw data to: `main/results/BER/<Model>_sequence_model/zoh_sound_L_<val>.mat`.
  - Plots a comparison figure containing the benchmark (sounding every block) and the sparse sounding lines, saving to: `main/results/BER/<Model>_sequence_model/ue_ber_zoh_sounding_comparison.pdf`.

---

### 7. `run_simulation_Doppler_interpolate_sounding.m`
* **Purpose**: Simulates and compares BER performance for sparse sounding configurations `[a b]` where the receiver estimates the Doppler shift using pilots at blocks $1$ and $a$ to interpolate the channel phase rotation for communication blocks in between.
* **Configurable Parameters** (Lines 38-41):
  - `fading_models`: Cell array of fading environments to evaluate (e.g. `{'static'}`, `{'ar1'}`, or `{'jakes'}`).
  - `sounding_configs`: Cell array of configs to evaluate (default: `{[2, 9], [2, 17]}`).
* **Results**:
  - Saves raw data to: `main/results/BER/<Model>_sequence_model/sound_config_a_b.mat`.
  - Saves comparison plots to: `main/results/BER/<Model>_sequence_model/ue_ber_sparse_sounding_comparison_configs.pdf`.

---

### 8. `run_simulation_nakagami.m`
* **Purpose**: Simulates the BER versus $E_b/N_0$ under Nakagami-$m$ fading channels. Sweeps the SNR, runs Monte Carlo trials, and compares the simulation line to theoretical Nakagami-$m$ curves under perfect CSI and pilot-assisted detection.
* **Configurable Parameters** (Lines 22-27):
  - `params.m_nakagami`: Fading shape parameter $m \ge 0.5$ (default: `2.0`). ($m = 0.5$ is one-sided Gaussian, $m=1.0$ is Rayleigh, higher $m$ represents less severe fading, $m \to \infty$ approaches AWGN).
  - `fading_model`: Fading dynamics style.
    - `'nakagami-static'`: Quasi-static fading. Fading coefficient $g$ is generated randomly once per Monte Carlo trial and remains fixed/constant for all blocks in that trial.
    - `'nakagami-ar1'`: Time-varying block-to-block Nakagami-$m$ fading with AR(1) correlation.
* **Results**:
  - Plots simulated BER against theoretical curves and saves to: `main/results/BER/ue_ber_comms_only_nakagami_m_<m_val>.pdf`.
  - Saves simulation data to: `main/results/BER/ue_ber_comms_only_nakagami_m_<m_val>_<model_type>.mat`.
