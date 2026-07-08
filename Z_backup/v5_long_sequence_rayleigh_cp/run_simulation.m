%% ================================================================
%  run_simulation.m  —  NTN-ISAC Three-Node PMCW Simulation Driver
%
%  Modular project layout (MATLAB +package namespace):
%
%    +ntn/
%      SystemParams.m      Sec. 1  — waveform constants + MLS
%      Geometry.m          Sec. 1  — node positions, delays, Dopplers
%      ChannelModel.m      Sec. 2  — path loss + Rayleigh fading
%      TransmitSignal.m    Sec. 3  — PMCW frame [sounding; data]
%      +sensing/
%        RangeDopplerMap.m          — FFT circular corr + Doppler FFT
%        SIC.m                      — dominant-peak cancellation
%        DroneReceiver.m   Sec. 5  — Drone Rx + coherent integration
%      +comms/
%        UEReceiver.m      Sec. 4  — UE Rx signal construction
%        ChannelEstimator.m Sec 4.1 Step 1 — sounding ChEst
%        Demodulator.m     Sec. 4.1 Steps 2-6 — despread/equalize/BPSK
%        BERAnalysis.m     Sec. 4  — BER sweep + SINR cross-check
%    +plots/
%        plotRangeDoppler.m         — Figure 1: RD heatmaps
%        plotBER.m                  — Figure 2: BER vs Eb/N0
%
%  See system_model_3node_v2_THH.md for full mathematical derivations.
%  No toolboxes required (base MATLAB / Octave compatible).
%% ================================================================
clear; close all; clc;
rng(2024);

ROOT = fileparts(mfilename('fullpath'));   % project root directory

%% ---- Sec. 1: System & waveform parameters -------------------------
params = ntn.SystemParams();

%% ---- Sec. 1: Geometry (positions, delays, Dopplers) ---------------
geom = ntn.Geometry(params);

%% ---- Sec. 2: Channel model (path loss + Rayleigh fading) ----------
channel = ntn.ChannelModel(params, geom);

%% ---- Sec. 3: Transmit signal (PMCW frame: sounding + data) --------
txSignal = ntn.TransmitSignal(params);

%% ---- Sec. 5: Drone sensing pipeline --------------------------------
%  DroneReceiver builds the 2*ND x M Rx signal, then:
%    computeRangeDoppler()  ->  BPSK strip + coherent integration + RD map
%    applySIC()             ->  detect dominant peak, subtract, re-map

droneRx = ntn.sensing.DroneReceiver(params, geom, channel, txSignal);
droneRx.computeRangeDoppler(params, txSignal);
droneRx.applySIC(params, geom);

%% ---- Sec. 4: UE communication pipeline ----------------------------
%  UEReceiver : builds 2*ND x M received signal at UE
%  ChannelEstimator : sounding matched filter  ->  h_hat
%  Demodulator      : data matched filter  ->  equalize  ->  BPSK decision

ueRx  = ntn.comms.UEReceiver(params, geom, channel, txSignal);
chEst = ntn.comms.ChannelEstimator(params, ueRx, geom);
demod = ntn.comms.Demodulator(params, ueRx, chEst, txSignal);

%% ---- Sec. 4: BER analysis (sweep + analytical curves) -------------
%  BERAnalysis() pre-computes the two theoretical curves.
%  runSweep()   runs the Monte Carlo Eb/N0 sweep.

ber = ntn.comms.BERAnalysis();
ber.runSweep(params, geom, ueRx, chEst);

%% ---- Figures -------------------------------------------------------
plots.plotRangeDoppler(droneRx, geom, params, fullfile(ROOT, 'range_doppler.png'));
plots.plotBER(ber, fullfile(ROOT, 'ue_ber.png'));
plots.plotGeometry(geom, fullfile(ROOT, 'geometry.png'));
