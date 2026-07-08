classdef Demodulator < handle
% DEMODULATOR  Sec. 4.1, Steps 2-6 — Data despread, equalisation,
%   and BPSK symbol decision.
%
%   Steps:
%     2-3: Matched filter on data period  ->  despread peak z_mf
%     4  : Scalar equalization z_eq = z_mf / h_hat
%     5  : BPSK decision  d_hat = sign(Re{z_eq})
%     6  : Bit recovery   b_hat = (1 - d_hat) / 2
%
%   Usage:
%       demod = ntn.comms.Demodulator(params, ueRx, chEst, txSignal);

    properties
        R_data          % ND x M  data-period correlation output
        z_mf            % 1 x M   despread peak (matched filter output)
        z_eq            % 1 x M   equalised peak
        d_hat           % 1 x M   estimated polar symbols {+1,-1}
        b_hat           % 1 x M   recovered binary bits {0,1}
        BER_ref         % scalar  BER at reference noise level
    end

    methods
        function obj = Demodulator(p, ueRx, chEst, tx)
        % p     : ntn.SystemParams
        % ueRx  : ntn.comms.UEReceiver
        % chEst : ntn.comms.ChannelEstimator
        % tx    : ntn.TransmitSignal

            % Steps 2-3: Matched filter on data period
            obj.R_data = ifft(fft(ueRx.yU_data, [], 1) .* conj(chEst.Sf), [], 1);
            obj.z_mf   = obj.R_data(chEst.ell_DU_est + 1, :);

            % Step 4: Equalization — cancels h_DU and slow-time Doppler phase
            obj.z_eq   = obj.z_mf ./ chEst.h_hat;

            % Step 5: BPSK decision on real part
            obj.d_hat  = sign(real(obj.z_eq));          % {+1,-1}

            % Step 6: Bit recovery  b_hat = (1 - d_hat) / 2
            obj.b_hat  = (1 - obj.d_hat) / 2;

            obj.BER_ref = mean(obj.b_hat ~= tx.bD);
            fprintf('--- BER at reference noise (SNR~0 dB pre-despread) ---\n');
            fprintf('BER = %.4f\n\n', obj.BER_ref);
        end
    end
end
