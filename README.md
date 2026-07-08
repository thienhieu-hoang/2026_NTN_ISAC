# Explanation of File Versions

- **v6_long_sequence_rayleigh_intraFading_globalLoop:**  
	- d
- NOTE: from v1 to v5: just the channel paths to the UE (D-U, D-T-U) are time-varying (Rayleigh) (for comm purpose), the channel paths reflecting back to the Drone (D-U-D, D-T-D, D-T-U-D) are still Gaussian (for sensing purpose), so the range-Doppler heatmap of v6 is more noisy than those in v1-v5.
- **v5_long_sequence_rayleigh_cp:**
	- Simulation Setup: Each Monte Carlo trial simulates multiple blocks. With delay synchronization, only the first block experiences transient synchronization errors.  
	- Channel Model: The channel varies over time (from block to block) according to a Rayleigh fading process - Channel is unchanged within 1 block duration
	- **Add CPs to each block**: before both sounding period and comm period
	  --> No interference between blocks
	  --> The Theory and Simulation lines match 
- **v4_long_sequence_rayleigh_perfect_guard:**
	- Simulation Setup: Each Monte Carlo trial simulates multiple blocks. With delay synchronization, only the first block experiences transient synchronization errors.
	- Channel Model: The channel varies over time (from block to block) according to a Rayleigh fading process - Channel is unchanged within 1 block duration
	- **Assuming no interference between blocks** (simulation by block by block; take convolution between transmission block and its corresponding channel)
	--> The Theory and Simulation lines match 
- **v3_long_sequence_rayleigh:**
	- Simulation Setup: Each Monte Carlo trial simulates multiple blocks. With delay synchronization, only the first block experiences transient synchronization errors.
	- Channel Model: **The channel varies over time** (from block to block) according to a Rayleigh fading process - Channel is unchanged within 1 block duration
	--> Simulation and Theory lines do not match DUE TO inter-block-interference.
- **v2_long_sequence_gaussian:**
	- Simulation Setup: Each Monte Carlo trial simulates **multiple blocks**. With delay synchronization, only the first block experiences transient synchronization errors.
	- Channel Model: The channel is static for each Monte Carlo trial, behaving like an AWGN channel.
- **v1_clean_synchronization:** 
	- Block Structure: 1 block = `[sounding period, communication period]`. Assumes a perfect guard interval between the two periods, meaning there is no inter-symbol interference (ISI) from channel delay spread. - no error caused by `circshift`.
	- Simulation Setup: Each Monte Carlo trial simulates only 1 block.
# Explanation of code files
- run_simulation_perfect_channel.m files: only consider 2 nodes, assuming the channel is perfectly known, and no scattering. --> Just to check if the analysis and simulation lines match or not
- run_simulation.m files: simulation with 3 nodes