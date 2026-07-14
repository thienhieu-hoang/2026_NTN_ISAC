%% ================================================================
%  run_visualize_waveforms.m
%  Visualizes the joint PMCW radar-communication waveforms.
%  Figure 1: Time Domain (including zoomed 1st block).
%  Figure 2: Frequency Domain comparison of +1 vs -1 blocks.
% ================================================================
clear; close all; clc;

ROOT = fileparts(mfilename('fullpath'));
addpath(ROOT);

fprintf('================================================================\n');
fprintf('   NTN-ISAC Waveform Visualization (Time & Frequency Split)\n');
fprintf('================================================================\n\n');

% =========================================================================
%   CONFIGURATION PARAMETERS (Edit these as needed)
% =========================================================================
M_plot          = 3;     % Number of blocks to plot in the time domain
S_per_chip      = 1;    % Upsampling factor (Sample Rate / Chip Rate). Set to 1 for Sample Rate = Chip Rate.
sounding_config = []; %[3 5]; % Sounding configuration [a b]. Set to [] for standard sounding (sounding in every block).
% =========================================================================

% Initialize System parameters to get the default MLS code
params = ntn.SystemParams();
ND     = params.ND;
s_mls  = params.s_mls;

% Determine which blocks have sounding slots in the first half
is_sounding_block = false(1, M_plot);
if ~isempty(sounding_config)
    a = sounding_config(1);
    b = sounding_config(2);
    for m = 1:M_plot
        if mod(m-1, b-1) == 0 || mod(m-1, b-1) == a-1
            is_sounding_block(m) = true;
        end
    end
else
    is_sounding_block = true(1, M_plot); % Every block has sounding
end

% Set slot types (each block has 2 slots)
is_comm_slot = true(1, 2 * M_plot);
for m = 1:M_plot
    if is_sounding_block(m)
        is_comm_slot(2*m - 1) = false; % First slot of block m is sounding
    end
end

% Count number of communication slots and generate random BPSK symbols
N_comm = sum(is_comm_slot);
dD = 1 - 2 * randi([0 1], 1, N_comm);

% Time and chip axis setup (excluding cyclic prefix)
total_chips = M_plot * 2 * ND;
chip_axis   = 1:total_chips;

% 1. Bit sequence over time (sounding slot is +1, comm slot is dD(comm_idx))
bits_time = zeros(total_chips, 1);
comm_idx = 1;
for k = 1:2*M_plot
    idx_start = (k-1)*ND + 1;
    idx_end   = k*ND;
    if is_comm_slot(k)
        bits_time(idx_start : idx_end) = dD(comm_idx);
        comm_idx = comm_idx + 1;
    else
        bits_time(idx_start : idx_end) = 1; % Pilot is +1
    end
end

% 2. PRBS sequence (repeated MLS code for both slots)
prbs_time = repmat(s_mls, M_plot * 2, 1);

% 3. Modulated baseband sequence (PRBS * Bit Sequence)
prbs_bit_time = prbs_time .* bits_time;

% 4. Baseband Waveforms at 0Hz
total_samples = total_chips * S_per_chip;
t_samples     = 1:total_samples;
t_chips       = t_samples / S_per_chip;

% Generate the upsampled baseband signal (constant/rectangular pulse)
baseband_signal_upsampled = repelem(prbs_bit_time, S_per_chip);


%% ========================================================================
%  FIGURE 1: Time Domain Waveform Visualization
%  ========================================================================
fig1 = figure('Name', 'NTN-ISAC Waveform Analysis (Time Domain)', 'Position', [100 80 1200 850], 'Color', 'w');
y_lim = [-1.5, 1.5];

% Subplot 1: Bit Sequence (Data Payload + Pilot)
% ax1 = subplot(3, 1, 1);
ax1 = subplot(4, 1, 1);
draw_shading(ax1, ND, dD, M_plot, is_comm_slot, y_lim(1), y_lim(2));
plot(ax1, chip_axis, bits_time, 'k-', 'LineWidth', 2.5);
ylabel(ax1, 'Bit Sequence d_m', 'FontWeight', 'bold');
xlabel(ax1, 'Chip Index', 'FontWeight', 'bold');
title(ax1, '1. Bit Sequence Over Time (+1 = Sounding Pilot, \pm1 = Data Payload)', 'FontSize', 11, 'FontWeight', 'bold');
set_axes_style(ax1, total_chips, y_lim, ND);

% Add text annotations inside Subplot 1 dynamically
comm_count = 0;
for k = 1:2*M_plot
    x_center = (k-1)*ND + ND/2;
    if is_comm_slot(k)
        comm_count = comm_count + 1;
        text(ax1, x_center, 0.5, sprintf('Data: %+d', dD(comm_count)), ...
             'HorizontalAlignment', 'center', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 8);
    else
        text(ax1, x_center, 0.5, 'Sounding: +1', ...
             'HorizontalAlignment', 'center', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 8);
    end
end

% Subplot 2: PRBS (MLS Code Sequence)
% ax2 = subplot(3, 1, 2);
ax2 = subplot(4, 1, 2);
draw_shading(ax2, ND, dD, M_plot, is_comm_slot, y_lim(1), y_lim(2));
plot(ax2, chip_axis, prbs_time, 'Color', [0.10 0.35 0.75], 'LineWidth', 1.2);
ylabel(ax2, 'MLS Code s_{mls}', 'FontWeight', 'bold');
xlabel(ax2, 'Chip Index', 'FontWeight', 'bold');
title(ax2, '2. PRBS Sequence (Maximal Length Sequence repeated per slot)', 'FontSize', 11, 'FontWeight', 'bold');
set_axes_style(ax2, total_chips, y_lim, ND);

% Subplot 3: PRBS * Bit Sequence (Baseband Modulated Signal)
% ax3 = subplot(3, 1, 3);
ax3 = subplot(4, 1, 3);
draw_shading(ax3, ND, dD, M_plot, is_comm_slot, y_lim(1), y_lim(2));
plot(ax3, t_chips, baseband_signal_upsampled, 'Color', [0.85 0.30 0.10], 'LineWidth', 1.2);
ylabel(ax3, 's_{mls} \times d_m', 'FontWeight', 'bold');
xlabel(ax3, 'Chip Index', 'FontWeight', 'bold');
title(ax3, '3. Baseband Signal (PRBS \times Bit Sequence - Rectangular Pulse Shape)', 'FontSize', 11, 'FontWeight', 'bold');
set_axes_style(ax3, total_chips, y_lim, ND);

% Subplot 4: Baseband Waveform (Zoomed to 1st Slot only, chips 1 to 127)
ax4 = subplot(4, 1, 4);
draw_shading(ax4, ND, dD, M_plot, is_comm_slot, y_lim(1), y_lim(2), S_per_chip);
plot(ax4, t_samples, baseband_signal_upsampled, 'Color', [0.45 0.15 0.65], 'LineWidth', 1.5);
ylabel(ax4, 'Baseband Signal', 'FontWeight', 'bold');
xlabel(ax4, sprintf('Sample Index (Chip index x %d)', S_per_chip), 'FontWeight', 'bold');
% title(ax4, sprintf('4. Baseband Waveform (Zoomed View: First Slot [Sounding/Pilot] only, Samples 1 to %d)', ND * S_per_chip), 'FontSize', 11, 'FontWeight', 'bold');
title(ax4, sprintf('4. Baseband Waveform'), 'FontSize', 11, 'FontWeight', 'bold');
% Just plot the 1st slot in sample domain: X-limit from 0 to ND*S_per_chip
% set_axes_style(ax4, ND * S_per_chip, y_lim, ND * S_per_chip);
%
set_axes_style(ax4, total_chips, y_lim, ND * S_per_chip);

%% ========================================================================
%  FIGURE 2: Frequency Domain Analysis (Separate Figure)
%  =======================================================================
fig2 = figure('Name', 'NTN-ISAC Waveform Analysis (Frequency Domain)', 'Position', [150 150 1200 700], 'Color', 'w');

% Pre-compute sample grids for exactly 1 slot (ND chips)
ND_samples = ND * S_per_chip;
block_minus_baseband = -s_mls;
block_minus_upsampled = repelem(block_minus_baseband, S_per_chip);

block_plus_baseband = s_mls;
block_plus_upsampled = repelem(block_plus_baseband, S_per_chip);

signal_minus = block_minus_upsampled;
signal_plus  = block_plus_upsampled;

Y_minus = fftshift(fft(signal_minus));
PSD_minus = (abs(Y_minus).^2) / ND_samples;

Y_plus = fftshift(fft(signal_plus));
PSD_plus = (abs(Y_plus).^2) / ND_samples;

% Frequency axis normalized by the chip rate B
f_axis_block = ((0:ND_samples-1) - floor(ND_samples/2)) * (S_per_chip / ND_samples);

% -------------------------------------------------------------------------
% Subplot 2.1: -1 Bit Waveform Spectrum (Light Orange Background)
% -------------------------------------------------------------------------
ax_f1 = subplot(2, 1, 1);
set(ax_f1, 'Color', [0.99 0.94 0.90]); % Soft light peach/orange background
grid(ax_f1, 'on');
ax_f1.GridColor = [0.75 0.75 0.75];
ax_f1.GridAlpha = 0.5;
hold(ax_f1, 'on');

plot(ax_f1, f_axis_block, 10*log10(PSD_minus/max(PSD_minus)), 'Color', [0.85 0.30 0.10], 'LineWidth', 1.5);
ylabel(ax_f1, 'PSD [dB]', 'FontWeight', 'bold');
title(ax_f1, 'A. Spectrum of a Comm Slot carrying -1 Data Bit (Data = -1 at 0Hz Baseband)', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax_f1, [-40, 5]);

if S_per_chip > 1
    xlim(ax_f1, [-3, 3]);
else
    xlim(ax_f1, [-0.5, 0.5]);
end
xline(ax_f1, 0, 'k--', 'Baseband Center', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.2, 'FontWeight', 'bold');

% -------------------------------------------------------------------------
% Subplot 2.2: +1 Bit Waveform Spectrum (Light Green Background)
% -------------------------------------------------------------------------
ax_f2 = subplot(2, 1, 2);
set(ax_f2, 'Color', [0.92 0.97 0.92]); % Soft light green background
grid(ax_f2, 'on');
ax_f2.GridColor = [0.75 0.75 0.75];
ax_f2.GridAlpha = 0.5;
hold(ax_f2, 'on');

plot(ax_f2, f_axis_block, 10*log10(PSD_plus/max(PSD_plus)), 'Color', [0.10 0.55 0.15], 'LineWidth', 1.5);
ylabel(ax_f2, 'PSD [dB]', 'FontWeight', 'bold');
xlabel(ax_f2, 'Normalized Frequency (f / B)', 'FontWeight', 'bold');
title(ax_f2, 'B. Spectrum of a Comm Slot carrying +1 Data Bit (Data = +1 at 0Hz Baseband)', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax_f2, [-40, 5]);

if S_per_chip > 1
    xlim(ax_f2, [-3, 3]);
else
    xlim(ax_f2, [-0.5, 0.5]);
end
xline(ax_f2, 0, 'k--', 'Baseband Center', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.2, 'FontWeight', 'bold');


%% ========================================================================
%  Save Figures to PDF
%  =======================================================================
resultsDir = fullfile(ROOT, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

savePathTimePdf = fullfile(resultsDir, 'waveform_visualization_time.pdf');
exportgraphics(fig1, savePathTimePdf, 'ContentType', 'vector');
fprintf('Saved cropped time-domain vector plot to: %s\n', savePathTimePdf);

savePathFreqPdf = fullfile(resultsDir, 'waveform_visualization_freq.pdf');
exportgraphics(fig2, savePathFreqPdf, 'ContentType', 'vector');
fprintf('Saved cropped frequency-domain vector plot to: %s\n', savePathFreqPdf);

fprintf('\n=== Waveform Visualization Completed Successfully ===\n');


% -------------------------------------------------------------------------
% Helper Functions
% -------------------------------------------------------------------------
function draw_shading(ax, ND, dD, M_plot, is_comm_slot, y_min, y_max, scale)
    if nargin < 8
        scale = 1;
    end
    hold(ax, 'on');
    
    % Background shading colors
    color_plus1    = [0.88 0.96 0.88];  % Soft light green
    color_minus1   = [0.99 0.92 0.86];  % Soft light orange (peach)
    
    comm_count = 0;
    for k = 1:2*M_plot
        x_start = (k-1)*ND * scale;
        x_end   = k*ND * scale;
        if is_comm_slot(k)
            comm_count = comm_count + 1;
            if dD(comm_count) == 1
                faceColor = color_plus1;
            else
                faceColor = color_minus1;
            end
        else
            faceColor = color_plus1; % Sounding pilot is +1 (always green)
        end
        patch(ax, [x_start x_end x_end x_start], [y_min y_min y_max y_max], ...
              faceColor, 'EdgeColor', 'none', 'FaceAlpha', 0.8, 'HandleVisibility', 'off');
    end
end

function set_axes_style(ax, total_chips, y_lim, ND)
    grid(ax, 'on');
    ax.GridColor = [0.8 0.8 0.8];
    ax.GridAlpha = 0.4;
    xlim(ax, [0, total_chips]);
    ylim(ax, y_lim);
    
    % Label block and slot boundaries on the X axis
    ax.XTick = 0:ND:total_chips;
    
    % Draw vertical lines for visual division
    for x_val = ND:ND:total_chips-1
        if mod(x_val, 2*ND) == 0
            % Solid black lines for full block boundaries
            xline(ax, x_val, 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        else
            % Dashed black lines for sounding/data slot boundaries
            xline(ax, x_val, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
