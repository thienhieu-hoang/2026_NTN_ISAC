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

            % Flatten transmit signal matrix to a 1D vector (continuous stream)
            xD_tx_flat = tx.xD_tx(:);

            % Replicate slow-time Doppler phase vectors over the chips for the sequence
            ph_DU_chips = kron(obj.ph_DU, ones(2*p.ND, 1));
            ph_DU_chips_flat = ph_DU_chips(:);
            
            ph_DTU_chips = kron(obj.ph_DTU, ones(2*p.ND, 1));
            ph_DTU_chips_flat = ph_DTU_chips(:);

            % Generate full received signal continuously (1D vector)
            yU_full_flat = ...
                  obj.A_DU  * circshift(xD_tx_flat, [g.ell_DU,  0]) .* ph_DU_chips_flat  ...
                + obj.A_DTU * circshift(xD_tx_flat, [g.ell_DTU, 0]) .* ph_DTU_chips_flat ...
                + obj.sigU_ref/sqrt(2) * (randn(2*p.ND*p.M, 1) + 1j*randn(2*p.ND*p.M, 1));

            % --- UE Receiver Processing (Frame Synchronization) ---
            % Compensate the delay by circularly shifting the continuous stream back
            yU_aligned = circshift(yU_full_flat, [-g.ell_DU, 0]);

            % Reshape back to 2ND x M matrix
            obj.yU_full = reshape(yU_aligned, [2*p.ND, p.M]);

            % Split into sounding and data halves (clean for m > 1)
            obj.yU_sound = obj.yU_full(1:p.ND,     :);
            obj.yU_data  = obj.yU_full(p.ND+1:end, :);
        end
    end
end
