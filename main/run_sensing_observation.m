%% ========================================================================
%  run_sensing_observation.m
%  NTN-ISAC Drone-Side Sensing Range-Doppler Map Dynamic Observation
%  Runs a single Monte Carlo trial and visualizes the Range-Doppler map
%  before and after SIC using a configurable sliding or growing window.
% ========================================================================
clear; close all; clc;
rng(2024); % Seed for reproducibility

ROOT = fileparts(mfilename('fullpath'));
addpath(ROOT);

fprintf('================================================================\n');
fprintf('   NTN-ISAC Drone-Side Sensing Range-Doppler Observation\n');
fprintf('================================================================\n\n');

%% ---- Sec. 1: Configuration parameters ----------------------------------
% [USER CONFIGURABLE] Window of observation parameters
window_mode = 'sliding';  % Options: 'sliding' (fixed window moving) or 'growing' (accumulating blocks)
window_size = 64;         % Size of the integration window in blocks (for 'sliding' mode)
step_size = 2;            % Update the heatmap after every 'step_size' blocks (e.g. 1, 2, 4, ...)
pause_duration = 0.1;     % Pause duration in seconds between updates for animation

% Simulation constants
EbN0_dB = 15;             % Communication SNR (Eb/N0) at the UE [dB]
SNR_target_dB = 18;       % Radar target single-period SNR [dB]
N_fft_doppler = 256;      % Slow-time FFT size (zero-padding for smooth Doppler bins)
save_gif = true;          % Set to true to save the animation as a GIF

%% ---- Sec. 2: System and Channel Initialization -----------------------
params = ntn.SystemParams();
geom = ntn.Geometry(params);
channel = ntn.ChannelModel(params, geom);

% Set total blocks for this trial
M_total = params.M; % Default is 256 or 500 based on SystemParams/BERAnalysis

% Pre-compute mobility trajectory over 2 * M_total slots
mob = ntn.MobilityModel(params, geom, 2 * M_total);

% Pre-compute channel constants for SNR calibration
C0 = (params.lamD / (4*pi))^2;
beta_DU_1 = C0 * mob.dDU_seq(1)^(-2);
avg_A_DU = sqrt(beta_DU_1) * sqrt(params.PD);
sigU = avg_A_DU * sqrt(params.ND / (2 * 10^(EbN0_dB/10)));

% Generate transmit signal and fading sequence
txSignal = ntn.TransmitSignal(params);
fading = ntn.FadingSequence(params, geom, 2 * M_total);

% Run UE receiver (Comms)
ueRx = ntn.comms.UEReceiver(params, geom, channel, txSignal, mob, fading, sigU);

% Instantiate Drone Receiver (Sensing)
droneRx = ntn.sensing.DroneReceiver(params, geom, channel, txSignal, mob, fading);
droneRx.SNR_target_dB = SNR_target_dB;

% Re-calculate sigD to match the configured SNR_target_dB
avg_aT = abs(channel.eta_T) * (C0 * mob.dDT_seq(1)^(-2)) * sqrt(params.PD);
droneRx.sigD = avg_aT * sqrt(params.ND * M_total) / sqrt(10^(SNR_target_dB/10));

%% ---- Sec. 3: Setup Windowing and Figure --------------------------------
if strcmp(window_mode, 'sliding')
    % Sliding window: fixed W, moves from 1 to M_total - W + 1
    W = window_size;
    m_starts = 1 : step_size : (M_total - W + 1);
    m_ends   = m_starts + W - 1;
    num_steps = numel(m_starts);
    fprintf('Running SLIDING window: size=%d blocks, step=%d blocks (%d steps total)\n', W, step_size, num_steps);
elseif strcmp(window_mode, 'growing')
    % Growing window: starts at W_start, grows by step_size up to M_total
    W_start = 16;
    m_starts = ones(1, floor((M_total - W_start)/step_size) + 1);
    m_ends   = W_start : step_size : M_total;
    % Ensure final block is included
    if m_ends(end) < M_total
        m_starts = [m_starts, 1];
        m_ends   = [m_ends, M_total];
    end
    num_steps = numel(m_ends);
    fprintf('Running GROWING window: start=%d blocks, step=%d blocks (%d steps total)\n', W_start, step_size, num_steps);
else
    error('Invalid window_mode. Choose ''sliding'' or ''growing''.');
end

% Set up results directory
resultsDir = fullfile(ROOT, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

% Set up figure for dynamic plotting
fig = figure('Name', 'Dynamic Range-Doppler Map Observation', 'Position', [100, 100, 1200, 500]);
gif_path = fullfile(resultsDir, 'sensing_observation.gif');

% Capture frame indices for snapshots
snapshot_steps = round(linspace(1, num_steps, 3));
snapshot_names = {'early', 'mid', 'late'};

fprintf('\n=== Starting Dynamic Range-Doppler Observation Loop ===\n');

for step_idx = 1:num_steps
    if ~mod(step_idx, 64)
        fprintf('Step %d/%d\n', step_idx, num_steps);
    end
    m_start = m_starts(step_idx);
    m_end   = m_ends(step_idx);
    W_curr  = m_end - m_start + 1;
    
    % 1. Extract block subsegments of sounding and data signals
    sub_yD_sound = droneRx.yD_sound(:, m_start:m_end);
    sub_yD_data  = droneRx.yD_data(:, m_start:m_end);
    sub_dD       = txSignal.dD(m_start:m_end);
    
    % 2. BPSK data stripping
    sub_ypD_data = sub_yD_data .* sub_dD;
    
    % 3. Coherent integration (sounding + data halves)
    % Shift by droneRx.ell_mono (target delay at block 1) to align map to physical range
    sub_yD_int = circshift(sub_yD_sound + sub_ypD_data, [droneRx.ell_mono, 0]);
    
    % 4. Compute Range-Doppler maps before and after SIC
    % We use our local function to allow a high-resolution, fixed Doppler grid
    RD_before = localRangeDopplerMap(sub_yD_int, params.s_mls, N_fft_doppler);
    
    % Apply SIC on the subsegment
    [sub_yD_int_sic, tgEst] = localSIC(sub_yD_int, params.s_mls, W_curr, params.Tblock);
    RD_after = localRangeDopplerMap(sub_yD_int_sic, params.s_mls, N_fft_doppler);
    
    % 5. Get true physical parameters at the center of the current window
    m_mid_slot = round(2 * ((m_start + m_end)/2) - 1);
    
    true_ell_target = mob.ell_mono_seq(m_mid_slot);
    true_v_target   = mob.nu_mono_seq(m_mid_slot) * params.lamD / 2;
    
    true_ell_ue     = mob.ell_2DU_seq(m_mid_slot);
    true_v_ue       = mob.nu_DU_seq(m_mid_slot) * params.lamD / 2;
    
    % 6. Plotting
    % Define axes for the plot
    rng_axis = (0:params.ND-1) * params.dR;
    fdopp_axis = ((0:N_fft_doppler-1) - floor(N_fft_doppler/2)) * (1 / (params.Tblock * N_fft_doppler));
    vel_axis = fdopp_axis * params.lamD / 2;
    
    % Normalization reference
    normRef = max(abs(RD_before(:)));
    
    % Subplot 1: Before SIC
    subplot(1, 2, 1);
    imagesc(vel_axis, rng_axis, 20*log10(abs(RD_before)/normRef));
    set(gca, 'YDir', 'normal');
    axis tight;
    caxis([-50, 0]);
    colorbar;
    xlabel('Velocity [m/s]');
    ylabel('Range [m]');
    title(sprintf('RD Map Before SIC (Blocks %d-%d, W=%d)', m_start, m_end, W_curr));
    colormap(jet);
    hold on;
    
    % Plot true locations
    h_tgt_b = plot(true_v_target, true_ell_target * params.dR, 'w^', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'r');
    h_ue_b  = plot(true_v_ue, true_ell_ue * params.dR, 'wo', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    hold off;
    legend([h_ue_b, h_tgt_b], {'Strong', 'Weak'}, 'TextColor', 'w', 'Location', 'northeast', 'Color', [0.2 0.2 0.2]);
    grid on;
    
    % Subplot 2: After SIC
    subplot(1, 2, 2);
    imagesc(vel_axis, rng_axis, 20*log10(abs(RD_after)/normRef));
    set(gca, 'YDir', 'normal');
    axis tight;
    caxis([-50, 0]);
    colorbar;
    xlabel('Velocity [m/s]');
    ylabel('Range [m]');
    title('RD Map After SIC (Strong Subtracted)');
    colormap(jet);
    hold on;
    
    % Plot true location of remaining weak target (strong target should be gone or heavily suppressed)
    h_tgt_a = plot(true_v_target, true_ell_target * params.dR, 'w^', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'r');
    hold off;
    legend(h_tgt_a, 'Weak', 'TextColor', 'w', 'Location', 'northeast', 'Color', [0.2 0.2 0.2]);
    grid on;
    
    % Super-title displaying window and time information
    integration_time_ms = W_curr * params.Tblock * 1000;
    sgtitle(sprintf('Drone Sensing Observation (Mode: %s) | Integration Time: %.2f ms\nStrong Est: Delay=%d, Dopp=%.1f Hz | True Strong: Delay=%d, Dopp=%.1f Hz', ...
        upper(window_mode), integration_time_ms, tgEst.ell, tgEst.nu, true_ell_target, mob.nu_mono_seq(m_mid_slot)), ...
        'FontSize', 12, 'FontWeight', 'bold');
    
    drawnow;
    
    % Capture snapshots
    snap_idx = find(snapshot_steps == step_idx);
    if ~isempty(snap_idx)
        snap_name = sprintf('sensing_snapshot_%s.pdf', snapshot_names{snap_idx});
        saveas(fig, fullfile(resultsDir, snap_name));
        fprintf('  Saved snapshot: %s\n', snap_name);
    end
    
    % Capture frame for GIF
    if save_gif
        frame = getframe(fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if step_idx == 1
            imwrite(imind, cm, gif_path, 'gif', 'Loopcount', inf, 'DelayTime', pause_duration);
        else
            imwrite(imind, cm, gif_path, 'gif', 'WriteMode', 'append', 'DelayTime', pause_duration);
        end
    end
    
    pause(pause_duration);
end

if save_gif
    fprintf('\nSaved complete animation to: %s\n', gif_path);
end
fprintf('=== Observation Completed successfully ===\n');

%% ---- Helper Functions --------------------------------------------------
function RD = localRangeDopplerMap(yp, s_mls, N_fft)
    % Circular matched filtering + Hann-windowed slow-time DFT with zero-padding
    ND = size(yp, 1);
    W_curr = size(yp, 2);
    
    Sf = fft(s_mls);
    R  = ifft(fft(yp, [], 1) .* conj(Sf), [], 1);     % range compression
    w  = 0.5 - 0.5 * cos(2*pi*(0:W_curr-1)/(W_curr-1)); % Hann window (slow-time)
    w  = w(:).'; % Row vector
    
    R_windowed = R .* w;
    RD = fftshift(fft(R_windowed, N_fft, 2), 2);      % Doppler FFT
end

function [yp_sic, est] = localSIC(yp, s_mls, W_curr, Tblock)
    % Local SIC version that operates on current window size
    ND = size(yp, 1);
    
    % 1. Detect dominant peak in standard RD map
    RD = localRangeDopplerMap(yp, s_mls, W_curr);
    [~, idx] = max(abs(RD(:)));
    [pPk, qPk] = ind2sub(size(RD), idx);
    
    ell = pPk - 1;                      % delay bin (0-based)
    qShift = qPk - 1 - floor(W_curr/2); % Doppler bin
    nuPk = qShift / (W_curr * Tblock);  % Doppler frequency [Hz]
    
    % 2. Estimate amplitude
    w  = 0.5 - 0.5 * cos(2*pi*(0:W_curr-1)/(W_curr-1));
    A  = RD(pPk, qPk) / (ND * sum(w));
    
    % 3. Reconstruct and subtract
    m = 0:W_curr-1;
    sPk = circshift(s_mls, ell);
    echo = A * (sPk * exp(1j*2*pi*nuPk*m*Tblock));
    yp_sic = yp - echo;
    
    est = struct('ell', ell, 'nu', nuPk, 'A', A);
end
