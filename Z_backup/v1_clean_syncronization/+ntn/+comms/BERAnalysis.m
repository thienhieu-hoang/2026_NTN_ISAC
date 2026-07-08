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
        Nblocks    = 5000       % Monte Carlo symbols per SNR point

        BER_sim                 % 1 x numel(EbN0_dB)  simulated BER
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

            % Pre-compute slow-time Doppler phases for Nblocks
            mm_blk   = 0:obj.Nblocks-1;
            ph_DU_n  = exp(1j*2*pi*g.fD_DU  * mm_blk * p.Tblock);
            ph_DTU_n = exp(1j*2*pi*g.nu_DTU  * mm_blk * p.Tblock);

            for ki = 1:numel(obj.EbN0_dB)
                EbN0 = 10^(obj.EbN0_dB(ki)/10);

                % Noise std:  sigma_U^2 = A_DU_mag^2 * ND / EbN0
                sigU = A_DU_mag * sqrt(p.ND / (2*EbN0));

                b_sim   = randi([0 1], 1, obj.Nblocks);
                d_sim   = 1 - 2*b_sim;

                % Split transmit frame into sounding and data halves to apply delay separately.
                % We assume there are guard intervals between blocks so that the circular shift
                % does not cause inter-period leakage (clean blocks).
                x_sound_sim = p.s_mls * ones(1, obj.Nblocks);
                x_data_sim  = p.s_mls * d_sim;

                % Apply delay separately to sounding and data periods
                yS = ueRx.A_DU  * circshift(x_sound_sim, [g.ell_DU,  0]) .* ph_DU_n  ...
                   + ueRx.A_DTU * circshift(x_sound_sim, [g.ell_DTU, 0]) .* ph_DTU_n ...
                   + sigU * (randn(p.ND, obj.Nblocks) + 1j*randn(p.ND, obj.Nblocks));

                yP = ueRx.A_DU  * circshift(x_data_sim, [g.ell_DU,  0]) .* ph_DU_n  ...
                   + ueRx.A_DTU * circshift(x_data_sim, [g.ell_DTU, 0]) .* ph_DTU_n ...
                   + sigU * (randn(p.ND, obj.Nblocks) + 1j*randn(p.ND, obj.Nblocks));

                % Step 1: Channel estimation from sounding
                Rs = ifft(fft(yS, [], 1) .* conj(chEst.Sf), [], 1);
                [~, peak_idx_sim]  = max(mean(abs(Rs), 2));
                ell_DU_est_sim     = peak_idx_sim - 1;
                hh = Rs(ell_DU_est_sim + 1, :) / p.ND;

                % Steps 2-4: Despread data + equalize
                Rd = ifft(fft(yP, [], 1) .* conj(chEst.Sf), [], 1);
                zd = Rd(ell_DU_est_sim + 1, :) ./ hh;

                % Steps 5-6: BPSK decision
                obj.BER_sim(ki) = mean(sign(real(zd)) ~= d_sim);
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
