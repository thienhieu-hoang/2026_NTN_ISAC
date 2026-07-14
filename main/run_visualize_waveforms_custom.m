%% ================================================================
%  run_visualize_waveforms_custom.m
%  Custom visualization of NTN-ISAC PMCW waveforms.
%  Subplot 1: Sounding bit sequence (sounding is +1 in blue, comm is 0/empty).
%  Subplot 2: Sounding and comm bit sequence (sounding +1 in blue, comm +/-1 in black).
%  X-axis: Block index (0.5 interval for slots, integer for block boundaries).
% ================================================================
clear; close all; clc;
rng(2024); % Set seed for consistent plot visualization

ROOT = fileparts(mfilename('fullpath'));
addpath(ROOT);

fprintf('================================================================\n');
fprintf('   NTN-ISAC Waveform Custom Visualization (Block-Index Axis)\n');
fprintf('================================================================\n\n');

% =========================================================================
%   CONFIGURATION PARAMETERS
% =========================================================================
M_plot          = 10;     % Number of blocks to plot
sounding_config = [2 9];    % Set to [] to use L_sound, or [a, b] for sparse sounding config
L_sound         = 2;     % Sounding interval. Only used if sounding_config = [] (set to 1 for every block)
% =========================================================================

% Initialize System parameters to get the default MLS code
params = ntn.SystemParams();

% Apply configurations to the params object
params.sounding_config = sounding_config;
params.L_sound         = L_sound;

% Set slot types (each block has 2 slots, total of 2 * M_plot slots)
is_comm_slot = true(1, 2 * M_plot);
if ~isempty(params.sounding_config)
    a = params.sounding_config(1);
    b = params.sounding_config(2);
    for m = 1:M_plot
        if mod(m-1, b-1) == 0 || mod(m-1, b-1) == a-1
            is_comm_slot(2*m - 1) = false; % Sounding slot in the first half of block m
        end
    end
else
    for m = 1:M_plot
        if mod(m-1, params.L_sound) == 0
            is_comm_slot(2*m - 1) = false; % Sounding slot in the first half of block m
        end
    end
end

% Count number of communication slots and generate random BPSK symbols
N_comm = sum(is_comm_slot);
dD = 1 - 2 * randi([0 1], 1, N_comm);

%% ========================================================================
%  FIGURE: Custom Waveform Visualization
%  ========================================================================
fig = figure('Name', 'NTN-ISAC Waveform Custom Analysis', 'Position', [100 80 1200 650], 'Color', 'w');
y_lim = [-1.5, 1.5];

% -------------------------------------------------------------------------
% Subplot 1: Sounding bits (sounding is +1 in blue, comm is 0/empty)
% -------------------------------------------------------------------------
ax1 = subplot(2, 1, 1);
hold(ax1, 'on');

for k = 1:2*M_plot
    x_start = (k-1) * 0.5;
    x_end   = k * 0.5;
    
    if ~is_comm_slot(k)
        % Plot sounding slot as a thick blue line at y=1
        plot(ax1, [x_start, x_end], [1, 1], 'b-', 'LineWidth', 3);
    end
    % Comm slot (0) is left unplotted/empty (only the thin 0-axis line is visible)
end

ylabel(ax1, 'Sounding Bit d_{sound}', 'FontWeight', 'bold');
xlabel(ax1, 'Block Index', 'FontWeight', 'bold');
title(ax1, '1. Sounding Bit Sequence (+1 = Sounding Pilot, 0 = No Sounding)', 'FontSize', 11, 'FontWeight', 'bold');
set_axes_style(ax1, M_plot, y_lim);

% Add text annotations inside Subplot 1 dynamically
for k = 1:2*M_plot
    x_center = (k-0.5) * 0.5;
    if ~is_comm_slot(k)
        text(ax1, x_center, 0, 'Sounding: +1', ...
             'Rotation', 90, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'Color', 'b', 'FontWeight', 'bold', 'FontSize', 9);
    else
        text(ax1, x_center, 0, 'Comm: 0', ...
             'Rotation', 90, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'Color', [0.5 0.5 0.5], 'FontWeight', 'bold', 'FontSize', 9);
    end
end

% -------------------------------------------------------------------------
% Subplot 2: Sounding and Comm Bit Sequence (sounding in blue, comm in black)
% -------------------------------------------------------------------------
ax2 = subplot(2, 1, 2);
hold(ax2, 'on');

comm_count = 0;
for k = 1:2*M_plot
    x_start = (k-1) * 0.5;
    x_end   = k * 0.5;
    
    if ~is_comm_slot(k)
        % Plot sounding slot as a thick blue line at y=1
        plot(ax2, [x_start, x_end], [1, 1], 'b-', 'LineWidth', 3);
    else
        comm_count = comm_count + 1;
        val = dD(comm_count);
        % Plot comm slot as a thick black line at y = +1 or -1
        plot(ax2, [x_start, x_end], [val, val], 'k-', 'LineWidth', 3);
    end
end

ylabel(ax2, 'Bit Sequence d_m', 'FontWeight', 'bold');
xlabel(ax2, 'Block Index', 'FontWeight', 'bold');
title(ax2, '2. Bit Sequence (Sounding Pilot = +1 in Blue, Comm Data = \pm1 in Black)', 'FontSize', 11, 'FontWeight', 'bold');
set_axes_style(ax2, M_plot, y_lim);

% Add text annotations inside Subplot 2 dynamically
comm_count = 0;
for k = 1:2*M_plot
    x_center = (k-0.5) * 0.5;
    if ~is_comm_slot(k)
        text(ax2, x_center, 0, 'Sounding: +1', ...
             'Rotation', 90, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'Color', 'b', 'FontWeight', 'bold', 'FontSize', 9);
    else
        comm_count = comm_count + 1;
        text(ax2, x_center, 0, sprintf('Data: %+d', dD(comm_count)), ...
             'Rotation', 90, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'Color', 'k', 'FontWeight', 'bold', 'FontSize', 9);
    end
end

%% ========================================================================
%  Save Figure to PDF
%  =======================================================================
resultsDir = fullfile(ROOT, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
savePathPdf = fullfile(resultsDir, 'waveform_custom_visualization.pdf');
exportgraphics(fig, savePathPdf, 'ContentType', 'vector');
fprintf('Saved custom time-domain vector plot to: %s\n', savePathPdf);

fprintf('\n=== Custom Waveform Visualization Completed Successfully ===\n');

% -------------------------------------------------------------------------
% Helper Functions
% -------------------------------------------------------------------------
function set_axes_style(ax, M_plot, y_lim)
    grid(ax, 'on');
    ax.GridColor = [0.8 0.8 0.8];
    ax.GridAlpha = 0.4;
    xlim(ax, [0, M_plot]);
    ylim(ax, y_lim);
    
    % Label block boundaries on the X axis (0, 1, 2, ..., M_plot)
    ax.XTick = 0:1:M_plot;
    
    % Draw thin horizontal line at y=0 (zero-axis)
    yline(ax, 0, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    
    % Draw vertical lines for visual division
    for x_val = 0.5:0.5:M_plot-0.1
        if mod(x_val, 1) == 0
            % Solid black lines for full block boundaries
            xline(ax, x_val, 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        else
            % Dashed black lines for sounding/data slot boundaries
            xline(ax, x_val, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
