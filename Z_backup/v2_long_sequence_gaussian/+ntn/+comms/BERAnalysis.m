classdef BERAnalysis < handle
% BERANALYSIS  Sec. 4 — UE BER vs Eb/N0 sweep.
%
%   Computes three BER curves:
%     BER_theory     : BPSK AWGN reference with perfect CSI
%                      Pe = 0.5*erfc(sqrt(Eb/N0))
%     BER_theory_est : Analytical pilot-assisted BPSK (Sec. 4.2, MGF derivation)
%                      Pe = 0.5*exp(-Eb/N0)
%     BER_sim        : Monte Carlo simulation with sounding ChEst
%
%   Also performs the analytical SINR cross-check (Sec. 4.1).
%
%   Usage:
%       ber = ntn.comms.BERAnalysis();
%       ber.runSweep(params, geom, ueRx, chEst);

    properties
        EbN0_dB    = 0:2:20    % Eb/N0 sweep points [dB]
        M_seq      = 1       % Number of blocks in the continuous sequence
        N_trials   = 500       % Monte Carlo trials per SNR point

        BER_sim                 % 1 x numel(EbN0_dB)  simulated BER (clean blocks 2..M_seq)
        BER_block1              % 1 x numel(EbN0_dB)  simulated BER of first block (transient)
        BER_all                 % 1 x numel(EbN0_dB)  simulated BER of entire sequence (average)
        BER_theory              % 1 x numel(EbN0_dB)  BPSK AWGN (perfect CSI)
        BER_theory_est          % 1 x numel(EbN0_dB)  pilot ChEst analytical

        rho_norm                % scalar  MLS cross-correlation rho(ell_DTU)
        SINR_supp               % scalar  SINR with scatter suppression
    end

    methods
        function obj = BERAnalysis()
            % Pre-compute theoretical BER curves (no dependencies needed)
            EbN0_lin           = 10.^(obj.EbN0_dB/10);
            obj.BER_sim        = zeros(size(obj.EbN0_dB));
            obj.BER_block1     = zeros(size(obj.EbN0_dB));
            obj.BER_all        = zeros(size(obj.EbN0_dB));
            obj.BER_theory     = 0.5 * erfc(sqrt(EbN0_lin));
            obj.BER_theory_est = 0.5 * exp(-EbN0_lin);
        end

        function runSweep(obj, p, g, ueRx, chEst)
        % Run the Monte Carlo Eb/N0 sweep and SINR cross-check.
        %   p     : ntn.SystemParams
        %   g     : ntn.Geometry
        %   ueRx  : ntn.comms.UEReceiver
        %   chEst : ntn.comms.ChannelEstimator

            A_DU_mag = abs(ueRx.A_DU);

            % Pre-compute slow-time Doppler phases for M_seq blocks
            mm_blk   = 0:obj.M_seq-1;
            ph_DU_n  = exp(1j*2*pi*g.fD_DU  * mm_blk * p.Tblock);
            ph_DTU_n = exp(1j*2*pi*g.nu_DTU  * mm_blk * p.Tblock);
            
            % Replicate slow-time Doppler phase vectors over the chips for the sequence
            ph_DU_chips = kron(ph_DU_n, ones(2*p.ND, 1));
            ph_DU_chips_flat = ph_DU_chips(:);
            
            ph_DTU_chips = kron(ph_DTU_n, ones(2*p.ND, 1));
            ph_DTU_chips_flat = ph_DTU_chips(:);

            % Pre-allocate BER arrays
            obj.BER_sim    = zeros(size(obj.EbN0_dB));
            obj.BER_block1 = zeros(size(obj.EbN0_dB));
            obj.BER_all    = zeros(size(obj.EbN0_dB));

            for ki = 1:numel(obj.EbN0_dB)
                EbN0 = 10^(obj.EbN0_dB(ki)/10);

                % Noise std:  sigma_U^2 = A_DU_mag^2 * ND / EbN0
                sigU = A_DU_mag * sqrt(p.ND / (2*EbN0));

                errors_clean = 0;
                total_bits_clean = 0;
                errors_block1 = 0;
                total_bits_block1 = 0;

                for trial = 1:obj.N_trials
                    b_sim   = randi([0 1], 1, obj.M_seq);
                    d_sim   = 1 - 2*b_sim;

                    % Continuous transmit signal: Stack sounding and BPSK data
                    x_tx_seq = [p.s_mls * ones(1, obj.M_seq); ...
                                 p.s_mls * d_sim];
                    
                    % Flatten to 1D vector
                    x_tx_flat = x_tx_seq(:);

                    % Continuous channel propagation with delay
                    y_rx_flat = ...
                        ueRx.A_DU  * circshift(x_tx_flat, [g.ell_DU,  0]) .* ph_DU_chips_flat  ...
                      + ueRx.A_DTU * circshift(x_tx_flat, [g.ell_DTU, 0]) .* ph_DTU_chips_flat ...
                      + sigU * (randn(size(x_tx_flat)) + 1j*randn(size(x_tx_flat)));

                    % --- UE Receiver Frame Synchronization ---
                    % Shift back the entire continuous stream to compensate for direct path delay
                    y_rx_aligned = circshift(y_rx_flat, [-g.ell_DU, 0]);

                    % Reshape back to 2ND x M_seq matrix
                    y_full_sim = reshape(y_rx_aligned, [2*p.ND, obj.M_seq]);

                    yS = y_full_sim(1:p.ND,     :);
                    yP = y_full_sim(p.ND+1:end, :);

                    % Step 1: Channel estimation from sounding (using matched filter)
                    Rs = ifft(fft(yS, [], 1) .* conj(chEst.Sf), [], 1);
                    [~, peak_idx_sim]  = max(mean(abs(Rs), 2));
                    ell_DU_est_sim     = peak_idx_sim - 1;
                    hh = Rs(ell_DU_est_sim + 1, :) / p.ND;

                    % Steps 2-4: Despread data + equalize
                    Rd = ifft(fft(yP, [], 1) .* conj(chEst.Sf), [], 1);
                    zd = Rd(ell_DU_est_sim + 1, :) ./ hh;

                    % Steps 5-6: BPSK decision
                    d_dec = sign(real(zd));

                    % Record errors for block 1 (corrupted block)
                    errors_block1 = errors_block1 + sum(d_dec(1) ~= d_sim(1));
                    total_bits_block1 = total_bits_block1 + 1;

                    % Record errors for blocks 2 to M_seq (clean blocks)
                    errors_clean = errors_clean + sum(d_dec(2:end) ~= d_sim(2:end));
                    total_bits_clean = total_bits_clean + (obj.M_seq - 1);
                end

                obj.BER_sim(ki)    = errors_clean / total_bits_clean;
                obj.BER_block1(ki) = errors_block1 / total_bits_block1;
                obj.BER_all(ki)    = (errors_block1 + errors_clean) / (total_bits_block1 + total_bits_clean);
            end

            % Analytical SINR cross-check (Sec. 4.1 SINR formula)
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
