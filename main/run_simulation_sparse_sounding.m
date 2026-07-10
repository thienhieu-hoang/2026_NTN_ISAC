%% ================================================================
%  run_simulation_sparse_sounding.m
%  NTN-ISAC Sparse Sounding Comms-Only BER Simulation Driver
%  Evaluates different sounding configurations [a b] (e.g. [2 9], [2 17], [2 33])
% ================================================================
close all; clc;
rng(2024);

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

% List of fading models to sweep
fading_models = {'ar1', 'jakes'};

% List of sounding configurations to evaluate
sounding_configs = {[2, 9], [2, 17], [2, 33]};

for f_idx = 1:length(fading_models)
    fading_model = fading_models{f_idx};
    fprintf('\n================================================================\n');
    fprintf('   Fading Model: %s\n', upper(fading_model));
    fprintf('================================================================\n');
    
    % Setup results directory based on selected model
    if strcmpi(fading_model, 'jakes')
        resultsDir = fullfile(ROOT, 'results', 'BER', 'Jake sequence model');
    else
        resultsDir = fullfile(ROOT, 'results', 'BER', 'AR_1 sequence model');
    end

    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    for c_idx = 1:length(sounding_configs)
        config = sounding_configs{c_idx};
        a = config(1);
        b = config(2);
        
        fprintf('\n--- Evaluating Sounding Config: [%d, %d] ---\n', a, b);
        params.sounding_config = config; % Dynamically set sounding configuration
        
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
        
        % Save simulation results for this sounding config
        matPath = fullfile(resultsDir, sprintf('sound_config_%d_%d.mat', a, b));
        save(matPath, 'EbN0_dB_sparse', 'BER_all', 'config', 'fading_model');
        fprintf('  Saved results to: %s\n', matPath);
    end
    
    %% ---- Plot Comparison Along With Benchmark ----
    benchmarkPath = fullfile(fileparts(resultsDir), 'ue_ber_comms_only.mat');
    if exist(benchmarkPath, 'file')
        benchmark = load(benchmarkPath);
        hasBenchmark = true;
    else
        hasBenchmark = false;
        warning('Benchmark file not found at: %s', benchmarkPath);
    end

    fig = figure('Name', sprintf('Sparse Sounding BER Comparison (%s)', upper(fading_model)), 'Position', [150 150 800 600]);

    % 1. Plot benchmark/theory curves
    % Analytical curves from ber_helper (always available)
    semilogy(ber_helper.EbN0_dB, ber_helper.BER_theory, 'k-', 'LineWidth', 1.8);
    hold on; grid on;
    semilogy(ber_helper.EbN0_dB, ber_helper.BER_theory_est, 'r--', 'LineWidth', 1.8);

    if hasBenchmark
        semilogy(benchmark.EbN0_dB, benchmark.BER_all, 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
    end

    % 2. Plot sparse sounding curves
    colors = {'mx-', 'g^-', 'c*-', 'kd-'};
    color_idx = 1;
    for c_idx = 1:length(sounding_configs)
        config = sounding_configs{c_idx};
        a = config(1);
        b = config(2);
        matPath = fullfile(resultsDir, sprintf('sound_config_%d_%d.mat', a, b));
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
    title(sprintf('UE Downlink BER: Sounding Config Comparison (%s)', upper(fading_model)));

    % Build dynamic legend
    legend_entries = {};
    legend_entries{end+1} = 'Theory (Perfect CSI)';
    legend_entries{end+1} = 'Theory (Pilot ChEst - Sounding Every Block)';
    if hasBenchmark
        legend_entries{end+1} = 'Sim: Sounding Every Block (Benchmark)';
    end
    for c_idx = 1:length(sounding_configs)
        config = sounding_configs{c_idx};
        a = config(1);
        b = config(2);
        matPath = fullfile(resultsDir, sprintf('sound_config_%d_%d.mat', a, b));
        if exist(matPath, 'file')
            legend_entries{end+1} = sprintf('Sim: Sounding Config [%d, %d]', a, b);
        end
    end
    legend(legend_entries, 'Location', 'southwest');

    % Save comparison plot
    comparisonPlotPath = fullfile(resultsDir, 'ue_ber_sparse_sounding_comparison_configs.pdf');
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
    
    % Save as PNG as well for quick previewing
    saveas(fig, strrep(comparisonPlotPath, '.pdf', '.png'));
    fprintf('\nSaved comparison plot: %s\n', comparisonPlotPath);
end

fprintf('\n=== Simulation Driver Completed Successfully ===\n');
