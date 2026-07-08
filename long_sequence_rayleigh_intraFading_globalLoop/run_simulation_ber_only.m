%% ================================================================
%  run_simulation_ber_only.m
%  NTN-ISAC Comms-Only BER Simulation Driver (3-Node Rayleigh Link)
%  Focuses purely on comms BER analysis, bypassing heavy radar sensing processing.
% ================================================================
clear; close all; clc;
rng(2024);

ROOT = fileparts(mfilename('fullpath'));   % project root directory
addpath(ROOT);

fprintf('================================================================\n');
fprintf('   NTN-ISAC Comms-Only Rayleigh Simulation & Verification\n');
fprintf('================================================================\n\n');

%% ---- Step 1: System, Geometry & Trajectory Initialization ----
params = ntn.SystemParams();
geom = ntn.Geometry(params);
channel = ntn.ChannelModel(params, geom);

% Instantiate helper for theoretical curves and storing BER
ber = ntn.comms.BERAnalysis();

% Sync parameters
params.M = ber.M_seq;       % M = 500 blocks for continuous sequence simulation
N_trials = ber.N_trials;    % 200 trials per SNR point
EbN0_dB = ber.EbN0_dB;

% Pre-compute mobility trajectory over 2 * M_seq slots
mob = ntn.MobilityModel(params, geom, 2 * params.M);

fprintf('--- MobilityModel (2x Slot-level): trajectory over %d slots ---\n', 2 * params.M);
fprintf('  Initial dDU=%.1f m -> Final dDU=%.1f m\n', mob.dDU_seq(1), mob.dDU_seq(end));
fprintf('  Initial dDT=%.1f m -> Final dDT=%.1f m\n', mob.dDT_seq(1), mob.dDT_seq(end));
fprintf('  Initial dTU=%.1f m -> Final dTU=%.1f m\n\n', mob.dTU_seq(1), mob.dTU_seq(end));

% Pre-compute channel constants for SNR calibration
C0 = (params.lamD / (4*pi))^2;
beta_DU_1 = C0 * mob.dDU_seq(1)^(-2);
avg_A_DU = sqrt(beta_DU_1) * sqrt(params.PD);

% Pre-allocate BER arrays
BER_sim_log    = zeros(size(EbN0_dB));
BER_block1_log = zeros(size(EbN0_dB));
BER_all_log    = zeros(size(EbN0_dB));

fprintf('=== Starting Comms-Only BER Sweep (3-Node Rayleigh fading) ===\n');

for ki = 1:numel(EbN0_dB)
    fprintf('SNR: %f\n', EbN0_dB(ki));
    EbN0 = 10^(EbN0_dB(ki)/10);
    sigU = avg_A_DU * sqrt(params.ND / (2*EbN0));
    
    errors_clean  = 0;
    total_clean   = 0;
    errors_block1 = 0;
    total_block1  = 0;
    
    for trial = 1:N_trials
        if ~mod(trial-1,50) || (trial==N_trials)
            fprintf('Simulation trial %d/%d \n', trial, N_trials);
        end
        % 1. Fresh transmit signal and data bits for this trial
        txSignal = ntn.TransmitSignal(params);
        
        % 2. Generate smooth fading sequence
        fading = ntn.FadingSequence(params, geom, 2 * params.M);
        
        % 3. Run UE Receiver (Comms) - generates signal with CP and removes it
        ueRx = ntn.comms.UEReceiver(params, geom, channel, txSignal, mob, fading, sigU);
        
        % 4. Run Channel Estimation & Demodulator (Comms)
        chEst = ntn.comms.ChannelEstimator(params, ueRx, geom);
        demod = ntn.comms.Demodulator(params, ueRx, chEst, txSignal);
        
        % Accumulate comm errors
        d_dec = demod.d_hat;
        d_sim = txSignal.dD;
        
        errors_block1 = errors_block1 + sum(d_dec(1) ~= d_sim(1));
        total_block1  = total_block1  + 1;
        
        errors_clean  = errors_clean  + sum(d_dec(2:end) ~= d_sim(2:end));
        total_clean   = total_clean   + (params.M - 1);
    end
    
    % Store average BERs
    BER_sim_log(ki)    = errors_clean / total_clean;
    BER_block1_log(ki) = errors_block1 / total_block1;
    BER_all_log(ki)    = (errors_block1 + errors_clean) / (total_block1 + total_clean);
    
    fprintf('  Eb/N0 = %2d dB | BER (Clean) = %.5f | BER (Block 1) = %.5f | BER (All) = %.5f\n', ...
            EbN0_dB(ki), BER_sim_log(ki), BER_block1_log(ki), BER_all_log(ki));
end

% Save results to helper object for plot compatibility
ber.BER_sim    = BER_sim_log;
ber.BER_block1 = BER_block1_log;
ber.BER_all    = BER_all_log;

% Analytical cross-check calculations using the final trial objects
ber.rho_norm = (ueRx.sD_DTU.' * ueRx.sD_DU) / params.ND;
EbN0_last    = 10^(ber.EbN0_dB(end)/10);
A_DU_mag2    = abs(ueRx.A_DU)^2;
ber.SINR_supp = A_DU_mag2 / ((abs(ueRx.A_DTU)*abs(ber.rho_norm))^2 + A_DU_mag2/EbN0_last);

fprintf('\n--- Analytical SINR cross-check (Sec. 4.1) ---\n');
fprintf('rho(ell_DTU)=%.5f  (1/N_D=%.5f)\n', ber.rho_norm, 1/params.ND);
fprintf('At Eb/N0=%d dB: SINR_U(scatter suppressed)=%.1f dB\n\n', ...
        ber.EbN0_dB(end), 10*log10(ber.SINR_supp));

%% ---- Step 5: Plot Comms-Only BER Results ----
plots.plotBER(ber, fullfile(ROOT, 'ue_ber_comms_only.png'));
fprintf('Saved: %s\n\n', fullfile(ROOT, 'ue_ber_comms_only.png'));

fprintf('=== Simulation Driver Completed Successfully ===\n');
