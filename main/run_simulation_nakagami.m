%% ================================================================
%  run_simulation_nakagami.m
%  NTN-ISAC Comms-Only BER Simulation Driver (3-Node Nakagami-m Link)
%  Focuses on comms BER analysis in Nakagami-m fading channels.
% ================================================================
clear; close all; clc;
rng(2024);

ROOT = fileparts(mfilename('fullpath'));   % project root directory
addpath(ROOT);

fprintf('================================================================\n');
fprintf('   NTN-ISAC Comms-Only Nakagami-m Simulation & Verification\n');
fprintf('================================================================\n\n');

%% ---- Step 1: System, Geometry & Trajectory Initialization ----
params = ntn.SystemParams();
params.sounding_config = []; % Reset to standard sounding (every block) for comms-only baseline

% Configure Nakagami parameter m
% User can change this to any value m >= 0.5 (e.g. 0.5 for one-sided Gaussian, 1 for Rayleigh, 2, 3, etc.)
params.m_nakagami = 2.0; 

% Select fading model:
% - 'nakagami-static': g is random per trial but constant over the CPI (quasi-static fading)
% - 'nakagami-ar1': g is time-varying block-to-block (time-correlated Nakagami fading)
fading_model = 'nakagami-static'; 

fprintf('Selected Fading Model: %s (m = %.1f)\n\n', upper(fading_model), params.m_nakagami);

geom = ntn.Geometry(params);
channel = ntn.ChannelModel(params, geom);

% Instantiate helper for theoretical curves and storing BER
ber = ntn.comms.BERAnalysis(fading_model, params.m_nakagami);

% Sync parameters
params.M = ber.M_seq;       % M = 256 blocks for continuous sequence simulation
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

fprintf('=== Starting Comms-Only BER Sweep (3-Node Nakagami-m fading) ===\n');

for ki = 1:numel(EbN0_dB)
    fprintf('SNR: %f dB\n', EbN0_dB(ki));
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
        
        % 2. Generate Nakagami fading sequence
        fading = ntn.FadingSequence(params, geom, 2 * params.M, fading_model);
        
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

%% ---- Step 5: Plot & Save Comms-Only BER Results ----
resultsDir = fullfile(ROOT, 'results', 'BER');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

pdfName = sprintf('ue_ber_comms_only_nakagami_m_%.1f.pdf', params.m_nakagami);
pdfPath = fullfile(resultsDir, pdfName);
plots.plotBER(ber, pdfPath);

benchmarkFile = sprintf('ue_ber_comms_only_nakagami_m_%.1f_%s.mat', params.m_nakagami, fading_model);
matPath = fullfile(resultsDir, benchmarkFile);
% Extract values of the plotted lines for easy loading/replotting
EbN0_dB = ber.EbN0_dB;
BER_theory = ber.BER_theory;
BER_theory_est = ber.BER_theory_est;
BER_sim = ber.BER_sim;
BER_block1 = ber.BER_block1;
BER_all = ber.BER_all;
M_seq = ber.M_seq;
m_nakagami = params.m_nakagami;

save(matPath, 'EbN0_dB', 'BER_theory', 'BER_theory_est', 'BER_sim', 'BER_block1', 'BER_all', 'M_seq', 'm_nakagami', 'fading_model');
fprintf('Saved plot image: %s\n', pdfPath);
fprintf('Saved data values: %s\n\n', matPath);

fprintf('=== Simulation Driver Completed Successfully ===\n');
