%% ================================================================
%  replot_ber.m
%  Helper script to replot BER results from saved .mat file
% ================================================================
clear; close all; clc;

ROOT = fileparts(mfilename('fullpath'));
addpath(fullfile(ROOT, 'main')); % Ensure +plots package is on the path

resultsDir = fullfile(ROOT, 'main', 'results', 'BER');
matPath = fullfile(resultsDir, 'ue_ber_comms_only.mat');

if ~exist(matPath, 'file')
    error('Saved data file not found at: %s\nRun run_simulation_ber_only.m first.', matPath);
end

fprintf('Loading saved BER data from: %s...\n', matPath);
berData = load(matPath);

% Replot and save to results directory
pdfPath = fullfile(resultsDir, 'ue_ber_comms_only_replot.pdf');
plots.plotBER(berData, pdfPath);

fprintf('Replot complete. Saved figure to: %s\n', pdfPath);
