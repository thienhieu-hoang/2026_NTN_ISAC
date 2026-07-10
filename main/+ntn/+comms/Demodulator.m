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

            % 1. Determine slot types and build is_comm_slot
            is_comm_slot = true(1, 2 * p.M);
            if isprop(p, 'sounding_config') && ~isempty(p.sounding_config)
                a = p.sounding_config(1);
                b = p.sounding_config(2);
                for m = 1:p.M
                    if mod(m-1, b-1) == 0 || mod(m-1, b-1) == a-1
                        is_comm_slot(2*m - 1) = false; % sounding slot
                    end
                end
            else
                for m = 1:p.M
                    if mod(m-1, p.L_sound) == 0
                        is_comm_slot(2*m - 1) = false; % sounding slot
                    end
                end
            end
            
            % Interleave sounding and data slots to get all 2*M slots (ND x 2M)
            yU_slots = zeros(p.ND, 2 * p.M);
            yU_slots(:, 1:2:end) = ueRx.yU_sound;
            yU_slots(:, 2:2:end) = ueRx.yU_data;
            
            % Extract only the communication slots
            yU_comm = yU_slots(:, is_comm_slot); % ND x N_comm
            
            % Steps 2-3: Matched filter on communication slots
            obj.R_data = ifft(fft(yU_comm, [], 1) .* conj(chEst.Sf), [], 1);
            obj.z_mf   = obj.R_data(chEst.ell_DU_est + 1, :); % 1 x N_comm
            
            % Map each communication slot to its block index m
            comm_indices = find(is_comm_slot);
            block_indices = floor((comm_indices - 1)/2) + 1;
            
            % Channel estimates for the communication slots
            h_hat_comm = chEst.h_hat(block_indices); % 1 x N_comm
            
            % Step 4: Equalization — cancels h_DU and slow-time Doppler phase
            obj.z_eq   = obj.z_mf ./ h_hat_comm;

            % Step 5: BPSK decision on real part
            obj.d_hat  = sign(real(obj.z_eq));          % 1 x N_comm

            % Step 6: Bit recovery  b_hat = (1 - d_hat) / 2
            obj.b_hat  = (1 - obj.d_hat) / 2;

            obj.BER_ref = mean(obj.b_hat ~= tx.bD);
        end
    end
end
