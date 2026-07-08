classdef ChannelEstimator < handle
% CHANNELESTIMATOR  Sec. 4.1, Step 1 — Delay estimation and per-block
%   channel estimation from the sounding matched filter.
%
%   Method:  circular cross-correlation via FFT (matched filter).
%   The peak of the averaged correlation magnitude gives ell_DU_est;
%   slicing at that bin and normalising by ND yields h_hat.
%
%   h_hat(:,m) ~ A_DU * exp(j*2*pi*fD_DU*m*Tblock)  +  O(sigU/sqrt(ND))
%
%   Usage:
%       chEst = ntn.comms.ChannelEstimator(params, ueRx, geom);

    properties
        Sf              % ND x 1  pilot DFT  (reused by Demodulator & BERAnalysis)
        R_sound         % ND x M  sounding correlation output
        ell_DU_est      % scalar  estimated delay bin (0-based)
        h_hat           % 1 x M   per-block complex channel estimate
    end

    methods
        function obj = ChannelEstimator(p, ueRx, g)
        % p     : ntn.SystemParams
        % ueRx  : ntn.comms.UEReceiver
        % g     : ntn.Geometry

            obj.Sf      = fft(p.s_mls);
            obj.R_sound = ifft(fft(ueRx.yU_sound, [], 1) .* conj(obj.Sf), [], 1);

            % Delay estimation: peak of averaged cross-correlation power
            [~, peak_idx] = max(mean(abs(obj.R_sound), 2));
            obj.ell_DU_est = peak_idx - 1;      % convert to 0-based index

            % Per-block channel estimate: normalise by code length
            obj.h_hat = obj.R_sound(obj.ell_DU_est + 1, :) / p.ND;

            fprintf('--- Channel estimation at UE (Sec. 4.1, Step 1) ---\n');
            fprintf('Estimated UE delay: %d chips (True: %d)\n', ...
                    obj.ell_DU_est, g.ell_DU);
            fprintf('True |A_DU|=%.4e   Mean|h_hat|=%.4e\n\n', ...
                    abs(ueRx.A_DU), mean(abs(obj.h_hat)));
        end
    end
end
