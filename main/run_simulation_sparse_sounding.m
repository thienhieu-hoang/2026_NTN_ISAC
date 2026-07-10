%% ================================================================
%  run_simulation_sparse_sounding.m
%  NTN-ISAC Sparse Sounding Comms-Only BER Simulation Driver
%  Evaluates different sounding densities (e.g. every 64, 128, 256 blocks)
% ================================================================
% clear; close all; clc;
close all; clc;
rng(2024);

% [USER CONFIGURABLE] Fading model type: 'ar1' or 'jakes'
if ~exist('fading_model', 'var')
    fading_model = 'ar1'; 
end

ROOT = fileparts(mfilename('fullpath'));   % project root directory
addpath(ROOT);

fprintf('================================================================\n');
fprintf('   NTN-ISAC Sparse Sounding Simulation & Verification\n');
fprintf('================================================================\n\n');

%% ---- Step 1: System, Geometry & Trajectory Initialization ----
params = ntn.SystemParams();
geom = ntn.Geometry(params);
channel = ntn.ChannelModel(params, geom);

% Instantiate helper for standard simulation constants
ber_helper = ntn.comms.BERAnalysis();

% Sync parameters
params.M = ber_helper.M_seq;       % M = 500 blocks for continuous sequence simulation
N_trials = ber_helper.N_trials;    % 200 trials per SNR point
EbN0_dB_sparse = 0:4:20;           % Sweep SNR points

% Pre-compute mobility trajectory over 2 * M_seq slots
mob = ntn.MobilityModel(params, geom, 2 * params.M);

% Pre-compute channel constants for SNR calibration
C0 = (params.lamD / (4*pi))^2;
beta_DU_1 = C0 * mob.dDU_seq(1)^(-2);
avg_A_DU = sqrt(beta_DU_1) * sqrt(params.PD);

% Setup results directory based on selected model
if strcmpi(fading_model, 'jakes')
    resultsDir = fullfile(ROOT, 'results', 'BER', 'Jake sequence model');
else
    resultsDir = fullfile(ROOT, 'results', 'BER', 'AR_1 sequence model');
end

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

% Sounding period densities to evaluate
if ~exist('L_sound_list', 'var')
    L_sound_list = [64, 128, 256];
end

fprintf('=== Starting Sparse Sounding BER Sweep (0:4:20 dB) ===\n');

for L_sound = L_sound_list
    fprintf('\n--- Evaluating Sounding Interval: Every %d blocks ---\n', L_sound);
    params.L_sound = L_sound; % Dynamically set sounding interval
    
    BER_all = zeros(size(EbN0_dB_sparse));
    
    for ki = 1:numel(EbN0_dB_sparse)
        fprintf('  SNR: %d dB\n', EbN0_dB_sparse(ki));
        EbN0 = 10^(EbN0_dB_sparse(ki)/10);
        sigU = avg_A_DU * sqrt(params.ND / (2*EbN0));
        
        errors_all = 0;
        total_all  = 0;
        
        for trial = 1:N_trials
            if ~mod(trial-1,50) || (trial==N_trials)
                fprintf('    Simulation trial %d/%d \n', trial, N_trials);
            end
            % 1. Fresh transmit signal and data bits for this trial
            txSignal = ntn.TransmitSignal(params);
            
            % 2. Generate smooth fading sequence
            fading = ntn.FadingSequence(params, geom, 2 * params.M, fading_model);
            
            % 3. Run UE Receiver (Comms)
            ueRx = ntn.comms.UEReceiver(params, geom, channel, txSignal, mob, fading, sigU);
            
            % 4. Run Channel Estimation & Demodulator (Comms)
            chEst = ntn.comms.ChannelEstimator(params, ueRx, geom);
            
            demod = ntn.comms.Demodulator(params, ueRx, chEst, txSignal);
            
            % Accumulate comm errors
            d_dec = demod.d_hat;
            d_sim = txSignal.dD;
            
            errors_all = errors_all + sum(d_dec ~= d_sim);
            total_all  = total_all  + length(d_sim);
        end
        
        BER_all(ki) = errors_all / total_all;
        fprintf('    Eb/N0 = %2d dB | BER (All) = %.5f\n', EbN0_dB_sparse(ki), BER_all(ki));
    end
    
    % Save simulation results for this sounding interval
    matPath = fullfile(resultsDir, sprintf('sound_every_%d.mat', L_sound));
    save(matPath, 'EbN0_dB_sparse', 'BER_all', 'L_sound', 'fading_model');
    fprintf('  Saved results to: %s\n', matPath);
end

%% ---- Step 5: Plot Comparison Along With Benchmark ----
benchmarkPath = fullfile(fileparts(resultsDir), 'ue_ber_comms_only.mat');
if exist(benchmarkPath, 'file')
    benchmark = load(benchmarkPath);
    hasBenchmark = true;
else
    hasBenchmark = false;
    warning('Benchmark file not found at: %s', benchmarkPath);
end

fig = figure('Name', 'Sparse Sounding BER Comparison', 'Position', [150 150 800 600]);

% 1. Plot benchmark curves
if hasBenchmark
    semilogy(benchmark.EbN0_dB, benchmark.BER_theory, 'k-', 'LineWidth', 1.8);
    hold on; grid on;
    semilogy(benchmark.EbN0_dB, benchmark.BER_theory_est, 'r--', 'LineWidth', 1.8);
    semilogy(benchmark.EbN0_dB, benchmark.BER_all, 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
else
    hold on; grid on;
end

% 2. Plot sparse sounding curves
colors = {'mx-', 'g^-', 'c*-', 'kd-'};
color_idx = 1;
for L_sound = L_sound_list
    matPath = fullfile(resultsDir, sprintf('sound_every_%d.mat', L_sound));
    if exist(matPath, 'file')
        data = load(matPath);
        semilogy(data.EbN0_dB_sparse, data.BER_all, colors{mod(color_idx-1, length(colors))+1}, ...
                 'LineWidth', 1.5, 'MarkerFaceColor', colors{mod(color_idx-1, length(colors))+1}(1));
        color_idx = color_idx + 1;
    end
end

xlabel('E_b/N_0 [dB]');
ylabel('BER');
ylim([1e-6 1]);
title('UE Downlink BER: Sounding Frequency Comparison');

% Build dynamic legend
legend_entries = {};
if hasBenchmark
    legend_entries{end+1} = 'Theory (Perfect CSI)';
    legend_entries{end+1} = 'Theory (Pilot ChEst - Sounding Every Block)';
    legend_entries{end+1} = 'Sim: Sounding Every Block (Benchmark)';
end
for L_sound = L_sound_list
    matPath = fullfile(resultsDir, sprintf('sound_every_%d.mat', L_sound));
    if exist(matPath, 'file')
        legend_entries{end+1} = sprintf('Sim: Sounding Every %d Blocks (%s)', L_sound, upper(fading_model));
    end
end
legend(legend_entries, 'Location', 'southwest');

% Save comparison plot
suffix = sprintf('_%d', L_sound_list);
comparisonPlotPath = fullfile(resultsDir, sprintf('ue_ber_sparse_sounding_comparison%s.pdf', suffix));
if endsWith(comparisonPlotPath, '.pdf', 'IgnoreCase', true)
    fig.Units = 'inches';
    fig.PaperUnits = 'inches';
    pos = fig.Position;
    fig.PaperSize = [pos(3), pos(4)];
    fig.PaperPosition = [0, 0, pos(3), pos(4)];
    print(fig, comparisonPlotPath, '-dpdf', '-r0');
else
    saveas(fig, comparisonPlotPath);
end
fprintf('\nSaved comparison plot: %s\n', comparisonPlotPath);
fprintf('=== Simulation Driver Completed Successfully ===\n');
