classdef UEReceiver < handle
% UERECEIVER  Sec. 4 — UE received signal construction.
%
%   Builds the full 2*ND x M received signal at the UE, comprising:
%     - Direct D->U link (desired communication signal)
%     - Scattered D->T->U interference (unknown channel)
%     - AWGN noise (reference power = |A_DU|, i.e. SNR~0 dB pre-despread)
%
%   The signal is split into sounding and data halves for downstream
%   processing by ChannelEstimator and Demodulator.
%
%   Usage:
%       ueRx = ntn.comms.UEReceiver(params, geom, channel, txSignal);

    properties
        % One-way link amplitudes at UE
        A_DU            % h_DU * sqrt(PD)           — direct path
        A_DTU           % eta_T * h_DT * h_TU * sqrt(PD)  — scattered

        % Delayed MLS replicas (used in BERAnalysis SINR cross-check)
        sD_DU           % ND x 1  direct path delayed code
        sD_DTU          % ND x 1  scattered path delayed code

        % Slow-time Doppler phase vectors (1 x M)
        ph_DU
        ph_DTU

        % Reference noise std (SNR ~ 0 dB pre-despread)
        sigU_ref

        % Received signal
        yU_full         % 2*ND x M  full received frame
        yU_sound        % ND x M    sounding half
        yU_data         % ND x M    data half
    end

    methods
        function obj = UEReceiver(p, g, ch, tx)
        % p  : ntn.SystemParams
        % g  : ntn.Geometry
        % ch : ntn.ChannelModel
        % tx : ntn.TransmitSignal

            obj.A_DU  = ch.h_DU * sqrt(p.PD);
            obj.A_DTU = ch.eta_T * ch.h_DT * ch.h_TU * sqrt(p.PD);

            fprintf('--- UE link amplitudes ---\n');
            fprintf('|A_DU|=%.4e  |A_DTU|=%.4e  direct/scatter ratio=%.1f dB\n\n', ...
                    abs(obj.A_DU), abs(obj.A_DTU), ...
                    20*log10(abs(obj.A_DU) / max(abs(obj.A_DTU), 1e-30)));

            obj.sD_DU  = circshift(p.s_mls, g.ell_DU);
            obj.sD_DTU = circshift(p.s_mls, g.ell_DTU);

            obj.ph_DU  = exp(1j*2*pi*g.fD_DU  * tx.m_idx * p.Tblock);
            obj.ph_DTU = exp(1j*2*pi*g.nu_DTU  * tx.m_idx * p.Tblock);

            % Reference noise amplitude (SNR ~ 0 dB pre-despread)
            obj.sigU_ref = abs(obj.A_DU);

            % Transmit signal matrix towards UE: [sounding; data] (2*ND x M)
            xU_tx = tx.xD_tx;

            % Split transmit signal into sounding and data halves to apply delay separately.
            % We assume there are guard intervals between blocks so that the circular shift
            % does not cause inter-period leakage (clean blocks).
            xU_sound = xU_tx(1:p.ND, :);
            xU_data  = xU_tx(p.ND+1:end, :);

            % Apply delay separately to sounding and data periods
            yU_sound_clean = obj.A_DU  * circshift(xU_sound, [g.ell_DU,  0]) .* obj.ph_DU  ...
                           + obj.A_DTU * circshift(xU_sound, [g.ell_DTU, 0]) .* obj.ph_DTU;
                       
            yU_data_clean  = obj.A_DU  * circshift(xU_data, [g.ell_DU,  0]) .* obj.ph_DU  ...
                           + obj.A_DTU * circshift(xU_data, [g.ell_DTU, 0]) .* obj.ph_DTU;

            % Add noise and assign to sounding and data variables
            obj.yU_sound = yU_sound_clean + obj.sigU_ref/sqrt(2) * (randn(p.ND, p.M) + 1j*randn(p.ND, p.M));
            obj.yU_data  = yU_data_clean  + obj.sigU_ref/sqrt(2) * (randn(p.ND, p.M) + 1j*randn(p.ND, p.M));

            % Stitch the clean parts back together for completeness of yU_full
            obj.yU_full  = [obj.yU_sound; obj.yU_data];
        end
    end
end
