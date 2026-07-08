classdef BERAnalysis < handle
% BERANALYSIS  Sec. 4 — UE BER vs Eb/N0 sweep with time-varying channel.
%
%   Computes three BER curves:
%     BER_theory     : BPSK Rayleigh reference with perfect CSI
%                      Pe = 0.5 * (1 - sqrt(EbN0 / (1 + EbN0)))
%     BER_theory_est : Analytical pilot-assisted BPSK under Rayleigh fading
%                      Pe = 1 / (2 * (1 + EbN0))
%     BER_sim        : Monte Carlo simulation with sounding ChEst
%     BER_block1     : BER of Block 1 (synchronisation transient)
%     BER_all        : Average BER over all blocks (1..M_seq)
%
%   Time-varying channel model (per Monte Carlo trial):
%     - Node positions advance linearly each block: p(m) = p(0) + v*(m-1)*Tblock
%     - Per-block path gains beta_DU(m) and beta_DTU(m) are derived from
%       the time-varying distances.
%     - Small-scale fading g_DU(m) and g_DTU(m) are generated via AR(1)
%       correlated Rayleigh sequences (ntn.FadingSequence) so they change
%       smoothly across blocks, not abruptly.
%     - Receiver thermal noise remains i.i.d. across chips (physically correct).
%
%   Sub-files used:
%     ntn.MobilityModel   — per-block positions, distances, delays, Dopplers
%     ntn.FadingSequence  — AR(1) smooth Rayleigh fading sequences
%
%   Usage:
%       ber = ntn.comms.BERAnalysis();
%       ber.runSweep(params, geom, ueRx, chEst);

    properties
        EbN0_dB    = 0:2:20    % Eb/N0 sweep points [dB]
        M_seq      = 500       % Number of blocks in the continuous sequence
        N_trials   = 200       % Monte Carlo trials per SNR point

        BER_sim                 % 1 x numel(EbN0_dB)  simulated BER (clean blocks 2..M_seq)
        BER_block1              % 1 x numel(EbN0_dB)  simulated BER of first block (transient)
        BER_all                 % 1 x numel(EbN0_dB)  simulated BER of entire sequence (average)
        BER_theory              % 1 x numel(EbN0_dB)  BPSK AWGN (perfect CSI)
        BER_theory_est          % 1 x numel(EbN0_dB)  pilot ChEst analytical

        rho_norm                % scalar  MLS cross-correlation rho(ell_DTU)
        SINR_supp               % scalar  SINR with scatter suppression
    end

    methods
% Pre-compute theoretical BER curves for Rayleigh Fading
        function obj = BERAnalysis()
            EbN0_lin           = 10.^(obj.EbN0_dB/10);
            obj.BER_sim        = zeros(size(obj.EbN0_dB));
            obj.BER_block1     = zeros(size(obj.EbN0_dB));
            obj.BER_all        = zeros(size(obj.EbN0_dB));
            obj.BER_theory     = 0.5 * (1 - sqrt(EbN0_lin ./ (1 + EbN0_lin)));
            obj.BER_theory_est = 1 ./ (2 * (1 + EbN0_lin));
        end

        function runSweep(obj, p, g, ueRx, chEst)
        % Run the Monte Carlo Eb/N0 sweep with time-varying channel.
        %   p     : ntn.SystemParams
        %   g     : ntn.Geometry   (initial positions and velocities)
        %   ueRx  : ntn.comms.UEReceiver
        %   chEst : ntn.comms.ChannelEstimator

            % ---- 1. Pre-compute mobility trajectory (fixed across all trials & SNR) ----
            % Positions and geometry are deterministic (linear motion), so we compute
            % them once.  Only the fading g[m] and noise are random per trial.
            mob = ntn.MobilityModel(p, g, obj.M_seq);

            fprintf('--- MobilityModel: trajectory over %d blocks ---\n', obj.M_seq);
            fprintf('  Initial dDU=%.1f m -> Final dDU=%.1f m\n', mob.dDU_seq(1), mob.dDU_seq(end));
            fprintf('  Initial dDT=%.1f m -> Final dDT=%.1f m\n', mob.dDT_seq(1), mob.dDT_seq(end));
            fprintf('  Initial dTU=%.1f m -> Final dTU=%.1f m\n\n', mob.dTU_seq(1), mob.dTU_seq(end));

            % ---- 2. Pre-compute channel constants for SNR calibration & SINR cross-check ----
            % Use average power E[|g|^2] = 1 for correct Eb/N0 calibration
            C0 = (p.lamD / (4*pi))^2;
            beta_DU_1 = C0 * mob.dDU_seq(1)^(-2);
            avg_A_DU = sqrt(beta_DU_1) * sqrt(p.PD);
            A_DU_mag = abs(ueRx.A_DU);

            % ---- 3. Pre-allocate BER accumulators ----
            obj.BER_sim    = zeros(size(obj.EbN0_dB));
            obj.BER_block1 = zeros(size(obj.EbN0_dB));
            obj.BER_all    = zeros(size(obj.EbN0_dB));

            % ---- 4. Per-SNR Monte Carlo sweep ----
            for ki = 1:numel(obj.EbN0_dB)
                EbN0 = 10^(obj.EbN0_dB(ki)/10);

                % Noise std: Calibrate to average path loss beta_DU at the start of the sequence
                sigU = avg_A_DU * sqrt(p.ND / (2*EbN0));

                errors_clean  = 0;
                total_clean   = 0;
                errors_block1 = 0;
                total_block1  = 0;

                for trial = 1:obj.N_trials
                    % ---- 4a. Generate smooth AR(1) fading for this trial ----
                    %                 First-order Autogressive model
                    % A new independent fading realization is drawn for each trial
                    % (different channel realisation), but within each trial the
                    % fading changes smoothly from block to block via AR(1).
                    fading = ntn.FadingSequence(p, g, obj.M_seq);

                    % ---- 4b. Build per-block channel amplitudes ----
                    % Large-scale path gain: beta_DU(m) = C0 * d_DU(m)^{-alpha}
                    C0        = (p.lamD / (4*pi))^2;
                    beta_DU_m = C0 * mob.dDU_seq .^ (-2);    % 1 x M_seq
                    beta_DTU_m = C0^2 .* (mob.dDT_seq .* mob.dTU_seq).^(-2) ...
                                 * abs(ueRx.A_DTU/ueRx.A_DU)^2 / A_DU_mag^2 ...
                                 * A_DU_mag^2;
                    % Simplified: use ratio |A_DTU/A_DU| from static channel
                    % scaled by time-varying path loss ratio
                    A_DU_m   = sqrt(beta_DU_m) .* fading.g_DU * sqrt(p.PD);  % 1 x M_seq
                    % Scattered path: A_DTU(m) = eta_T * sqrt(beta_DT(m)*beta_TU(m)) * g_DT(m)*g_TU(m) * sqrt(PD)
                    beta_DT_m = C0 * mob.dDT_seq .^ (-2);   % 1 x M_seq
                    beta_TU_m = C0 * mob.dTU_seq .^ (-2);   % 1 x M_seq
                    A_DTU_m  = abs(ueRx.A_DTU) / (abs(ueRx.A_DU)) ...
                               * sqrt(beta_DT_m .* beta_TU_m) .* fading.g_DT .* fading.g_TU ...
                               * sqrt(p.PD);
                    % Scale the scattered amplitude using static eta_T ratio

                    % ---- 4c. Generate data bits and reference MLS ----
                    b_sim = randi([0 1], 1, obj.M_seq);
                    d_sim = 1 - 2*b_sim;

                    % ---- 4d. Build the continuous received flat signal block-by-block ----
                    % Each block m contributes 2*ND chips:
                    %   Chips 1..ND  : sounding (d_m = +1)
                    %   Chips ND+1..2ND : data (d_m = d_sim(m))
                    %
                    % Received signal for each block (ignoring inter-block interference):
                    %   y_block(m) = A_DU(m) * s_delayed_DU * exp(j*2*pi*fD_DU(m)*m*Tblock)
                    %              + A_DTU(m) * s_delayed_DTU * exp(j*2*pi*nu_DTU(m)*m*Tblock)
                    %              + noise
                    %
                    % We use block-level delay bins from MobilityModel.

                    y_rx_flat = zeros(2*p.ND * obj.M_seq, 1);

                    % Allocate yS and yP directly to simulate perfect block-by-block guard
                    yS = zeros(p.ND, obj.M_seq);
                    yP = zeros(p.ND, obj.M_seq);

                    for m = 1:obj.M_seq
                        % Delay for direct path and scattered path at block m
                        ell_du  = mob.ell_DU_seq(m);
                        ell_dtu = mob.ell_DTU_seq(m);

                        % Doppler phase at slow-time index m
                        ph_DU  = exp(1j*2*pi*mob.fD_DU_seq(m)  * (m-1) * p.Tblock);
                        ph_DTU = exp(1j*2*pi*mob.nu_DTU_seq(m)  * (m-1) * p.Tblock);

                        % Generate received signals for block m
                        if m == 1
                            % Block 1: Startup transient (no preceding block)
                            x_S_DU  = [zeros(ell_du, 1); p.s_mls(1 : p.ND - ell_du)];
                            x_S_DTU = [zeros(ell_dtu, 1); p.s_mls(1 : p.ND - ell_dtu)];
                            
                            x_D_DU  = [p.s_mls(p.ND - ell_du + 1 : p.ND); p.s_mls(1 : p.ND - ell_du) * d_sim(1)];
                            x_D_DTU = [p.s_mls(p.ND - ell_dtu + 1 : p.ND); p.s_mls(1 : p.ND - ell_dtu) * d_sim(1)];
                        else
                            % Blocks 2+: Perfect guard (circular shift within each slot of size ND)
                            x_S_DU  = circshift(p.s_mls, ell_du);
                            x_S_DTU = circshift(p.s_mls, ell_dtu);
                            
                            x_D_DU  = circshift(p.s_mls * d_sim(m), ell_du);
                            x_D_DTU = circshift(p.s_mls * d_sim(m), ell_dtu);
                        end

                        yS(:, m) = A_DU_m(m)  * x_S_DU  * ph_DU  ...
                                 + A_DTU_m(m) * x_S_DTU * ph_DTU ...
                                 + sigU * (randn(p.ND,1) + 1j*randn(p.ND,1));
                                 
                        yP(:, m) = A_DU_m(m)  * x_D_DU  * ph_DU  ...
                                 + A_DTU_m(m) * x_D_DTU * ph_DTU ...
                                 + sigU * (randn(p.ND,1) + 1j*randn(p.ND,1));
                    end

                    % ---- 4e. Frame synchronisation: compensate direct-path delay locally ----
                    ell_sync = mob.ell_DU_seq(1);
                    yS = circshift(yS, [-ell_sync, 0]);
                    yP = circshift(yP, [-ell_sync, 0]);

                    % ---- 4g. Channel estimation from sounding ----
                    Rs = ifft(fft(yS, [], 1) .* conj(chEst.Sf), [], 1);
                    [~, peak_idx_sim] = max(mean(abs(Rs), 2));
                    ell_DU_est_sim    = peak_idx_sim - 1;
                    hh = Rs(ell_DU_est_sim + 1, :) / p.ND;   % 1 x M_seq

                    % ---- 4h. Despread + equalise + BPSK decision ----
                    Rd    = ifft(fft(yP, [], 1) .* conj(chEst.Sf), [], 1);
                    zd    = Rd(ell_DU_est_sim + 1, :) ./ hh;   % 1 x M_seq
                    d_dec = sign(real(zd));                     % 1 x M_seq

                    % ---- 4i. Count errors ----
                    errors_block1 = errors_block1 + sum(d_dec(1) ~= d_sim(1));
                    total_block1  = total_block1  + 1;

                    errors_clean  = errors_clean  + sum(d_dec(2:end) ~= d_sim(2:end));
                    total_clean   = total_clean   + (obj.M_seq - 1);
                end

                obj.BER_sim(ki)    = errors_clean  / total_clean;
                obj.BER_block1(ki) = errors_block1 / total_block1;
                obj.BER_all(ki)    = (errors_block1 + errors_clean) / (total_block1 + total_clean);
            end

            % ---- 5. Analytical SINR cross-check (Sec. 4.1 SINR formula) ----
            obj.rho_norm  = (ueRx.sD_DTU.' * ueRx.sD_DU) / p.ND;
            EbN0_last     = 10^(obj.EbN0_dB(end)/10);
            A_DU_mag2     = A_DU_mag^2;
            obj.SINR_supp = A_DU_mag2 / ...
                            ((abs(ueRx.A_DTU)*abs(obj.rho_norm))^2 + A_DU_mag2/EbN0_last);

            fprintf('--- Analytical SINR cross-check (Sec. 4.1) ---\n');
            fprintf('rho(ell_DTU)=%.5f  (1/N_D=%.5f)\n', obj.rho_norm, 1/p.ND);
            fprintf('At Eb/N0=%d dB: SINR_U(scatter suppressed)=%.1f dB\n\n', ...
                    obj.EbN0_dB(end), 10*log10(obj.SINR_supp));
        end
    end
end
