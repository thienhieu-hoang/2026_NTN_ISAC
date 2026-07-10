*(Readup from the bottom (from version 1) to better understand the updates gradually through each code version)*
# Explanation of File Versions
See code versions here: [Releases · thienhieu-hoang/2026_NTN_ISAC](https://github.com/thienhieu-hoang/2026_NTN_ISAC/tags)
now I want to perform channel prediction instead of just estimate channel at sounding period then assume it unchanged  
in the code, I can configure parameters like: [1 2 9] - which means, the sounding period appears at block 1 and block 2, we estimate the channels ate these 2 blocks, then estimate the Doppler, then predict channel at blocks 3-8 (in which there are no sounding period)
- **v_7_3 - main:** channel prediction at UE
	- Config: `[a,b]` - sounding is sent at blocks `1, a, b`, then `b, b+a-1, b+b-1`, ...
	- Use predicted channel at blocks `1, a` to predict channels at blocks `a+1` to `b-1`, ...
	- All other slots are `comm` slots.
- **v7_2 - main:**
	- Frame structure: (for example, with sounding periodicity of 8 block)
		``[sound, comm],  [comm, comm]x7 ``
		`[slot , slot],  [slot, slot]` - 1 `block` is defined by 2 `slot`'s
- **v7_1 - main**:
	- Now all the code version is just with the same folder main
	- Version control by git
	- The sounding is periodic, the periodicity can be set to 8, 16, 32, 64, 128, 256 blocks
	- The channel can be set to vary overtime (every half-block) with AR(1) model or Jakes model
	- (not good) the frame structure over time is set as: 
		(for example, with sounding periodicity of 8 block)
		`` [sound, comm],  [empty_sound, comm]x7 ``
			--> waste ``empty_sound period`` --> not realistic
- **v6_long_sequence_rayleigh_intraFading_globalLoop:**  
	- Channel varies from half-block to half-block (channel in comm period is slightly different to channel in sounding period) 
- NOTE: **from v1 to v5**: just the channel paths to the UE (D-U, D-T-U) are time-varying (Rayleigh) (for comm purpose), the channel paths reflecting back to the Drone (D-U-D, D-T-D, D-T-U-D) are still Gaussian (for sensing purpose), so the range-Doppler heatmap of v6 is more noisy than those in v1-v5.
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