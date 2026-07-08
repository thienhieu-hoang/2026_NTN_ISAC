%% ================================================================
%  run_simulation.m  —  NTN-ISAC Three-Node PMCW Simulation Driver
%  With Global Loop over Eb/N0 and Monte Carlo Trials
% ================================================================
clear; close all; clc;
rng(2024);

ROOT = fileparts(mfilename('fullpath'));   % project root directory

%% ---- Sec. 1: System & waveform parameters -------------------------
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

% Pre-compute channel constants for SNR calibration & SINR cross-check
C0 = (params.lamD / (4*pi))^2;
beta_DU_1 = C0 * mob.dDU_seq(1)^(-2);
avg_A_DU = sqrt(beta_DU_1) * sqrt(params.PD);

% Pre-allocate logs
BER_sim_log    = zeros(size(EbN0_dB));
BER_block1_log = zeros(size(EbN0_dB));
BER_all_log    = zeros(size(EbN0_dB));

% Sensing metrics: detection probability (did we find target delay with error <= 1 range bin?)
sensing_success_count = zeros(size(EbN0_dB));

% Keep a reference droneRx from the last trial of the highest SNR point for range-Doppler mapping
droneRx_ref = [];

fprintf('=== Starting Global Loop (Sensing + Comms) ===\n');

for ki = 1:numel(EbN0_dB)
    EbN0 = 10^(EbN0_dB(ki)/10);
    
    % Noise std: Calibrate to average path loss beta_DU at the start of the sequence
    sigU = avg_A_DU * sqrt(params.ND / (2*EbN0));
    
    errors_clean  = 0;
    total_clean   = 0;
    errors_block1 = 0;
    total_block1  = 0;
    
    for trial = 1:N_trials
        % 1. Fresh transmit signal and data bits for this trial
        txSignal = ntn.TransmitSignal(params);
        
        % 2. Generate smooth fading sequence
        fading = ntn.FadingSequence(params, geom, 2 * params.M);
        
        % 3. Run UE Receiver (Comms)
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
        
        % 5. Run Drone Receiver (Sensing)
        droneRx = ntn.sensing.DroneReceiver(params, geom, channel, txSignal, mob, fading);
        
        % 6. Compute Range-Doppler map & SIC (Sensing)
        droneRx.computeRangeDoppler(params, txSignal);
        droneRx.applySIC(params, geom);
        
        % Keep a copy for plotting at highest SNR point on the last trial
        if ki == numel(EbN0_dB) && trial == N_trials
            droneRx_ref = droneRx;
        end
        
        % 7. Target Detection Verification
        % Find the target peak in the Range-Doppler map after SIC
        RD_after = droneRx.RD_after;
        [~, max_idx] = max(abs(RD_after(:)));
        [target_ell_est_idx, target_q_est_idx] = ind2sub(size(RD_after), max_idx);
        
        target_ell_est = target_ell_est_idx - 1; % 0-based
        target_q_shift = target_q_est_idx - 1 - floor(params.M/2);
        target_nu_est  = target_q_shift / (params.M * params.Tblock);
        
        % Detection criterion: Delay estimate is exactly at the aligned bin (0)
        % and Doppler estimate is within 1.5 bins of the true target Doppler
        doppler_bin_width = 1 / (params.M * params.Tblock);
        doppler_err_bins = abs(target_nu_est - mob.nu_mono_seq(1)) / doppler_bin_width;
        
        if (target_ell_est == 0) && (doppler_err_bins <= 1.5)
            sensing_success_count(ki) = sensing_success_count(ki) + 1;
        end
    end
    
    % Store average BERs
    BER_sim_log(ki)    = errors_clean / total_clean;
    BER_block1_log(ki) = errors_block1 / total_block1;
    BER_all_log(ki)    = (errors_block1 + errors_clean) / (total_block1 + total_clean);
    
    fprintf('  Eb/N0 = %2d dB | BER = %.5f | Sensing Det. Prob = %.2f%%\n', ...
            EbN0_dB(ki), BER_all_log(ki), (sensing_success_count(ki) / N_trials) * 100);
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

%% ---- Figures -------------------------------------------------------
% 1. Plot Range-Doppler heatmaps
if ~isempty(droneRx_ref)
    plots.plotRangeDoppler(droneRx_ref, geom, params, fullfile(ROOT, 'range_doppler.png'));
end

% 2. Plot BER vs Eb/N0
plots.plotBER(ber, fullfile(ROOT, 'ue_ber.png'));

% 3. Plot Geometry
plots.plotGeometry(geom, fullfile(ROOT, 'geometry.png'));

% 4. Plot Sensing Detection Probability
figure('Name', 'Sensing Performance', 'Position', [200, 200, 720, 540]);
plot(EbN0_dB, (sensing_success_count / N_trials) * 100, 'ro-', 'LineWidth', 1.5, 'MarkerFaceColor', 'r');
grid on;
xlabel('E_b/N_0 [dB]');
ylabel('Target Detection Probability [%]');
title('Radar Target Detection Probability vs. Communication SNR');
ylim([0 105]);
saveas(gcf, fullfile(ROOT, 'sensing_detection_probability.png'));
fprintf('Saved: %s\n\n', fullfile(ROOT, 'sensing_detection_probability.png'));

fprintf('=== Simulation Driver Completed Successfully ===\n');
