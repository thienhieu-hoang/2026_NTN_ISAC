%% ================================================================
%  run_plot_geometry.m
%  Simple script to plot the 3D system geometry.
%  Uses ntn.SystemParams, ntn.Geometry, and plots.plotGeometry.
% ================================================================
clear; close all; clc;

ROOT = fileparts(mfilename('fullpath'));
addpath(ROOT);

fprintf('================================================================\n');
% Initialize System parameters and Geometry
params = ntn.SystemParams();
geom = ntn.Geometry(params);

% Print Geometry Info
fprintf('Drone Position: [%.1f, %.1f, %.1f] m\n', geom.pD(1), geom.pD(2), geom.pD(3));
fprintf('UE Position   : [%.1f, %.1f, %.1f] m\n', geom.pU(1), geom.pU(2), geom.pU(3));
fprintf('Target Position: [%.1f, %.1f, %.1f] m\n', geom.pT(1), geom.pT(2), geom.pT(3));
fprintf('================================================================\n\n');

% Define results directory and output paths
resultsDir = fullfile(ROOT, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
savePathPdf = fullfile(resultsDir, 'system_geometry.pdf');

% Call the plot function
plots.plotGeometry(geom, savePathPdf);

fprintf('=== Geometry Plotting Completed Successfully ===\n');
