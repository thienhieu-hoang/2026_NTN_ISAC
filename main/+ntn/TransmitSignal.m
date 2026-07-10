classdef TransmitSignal < handle
% TRANSMITSIGNAL  Sec. 3 — PMCW frame construction.
%
%   Builds the joint [sounding; data] transmit matrix  xD_tx  (2*ND x M).
%     Sounding half : pure MLS  (d_m = +1, no data modulation)
%     Data half     : MLS * d_m (BPSK-modulated)
%
%   BPSK bit-to-symbol map:  b_m in {0,1}  ->  d_m = 1 - 2*b_m in {+1,-1}
%   Note: d_m^2 = 1, used by DroneReceiver for radar data stripping.
%
%   Usage:
%       tx = ntn.TransmitSignal(params);

    properties
        bD      % 1 x N_comm  random binary bits {0,1}
        dD      % 1 x N_comm  polar BPSK symbols {+1,-1}
        m_idx   % 1 x M  block index vector  0:M-1
        xD_tx   % 2*ND x M  full transmit matrix [sounding; data]
    end

    methods
        function obj = TransmitSignal(p)
        % p : ntn.SystemParams
            obj.m_idx = 0:p.M-1;

            % Determine which slots are communication slots
            is_comm_slot = true(1, 2 * p.M);
            if isprop(p, 'sounding_config') && ~isempty(p.sounding_config)
                a = p.sounding_config(1);
                b = p.sounding_config(2);
                for m = 1:p.M
                    if mod(m-1, b-1) == 0 || mod(m-1, b-1) == a-1
                        is_comm_slot(2*m - 1) = false; % Sounding slot in the first half
                    end
                end
            else
                for m = 1:p.M
                    if mod(m-1, p.L_sound) == 0
                        is_comm_slot(2*m - 1) = false; % Sounding slot in the first half
                    end
                end
            end
            
            N_comm = sum(is_comm_slot);
            obj.bD = randi([0 1], 1, N_comm);
            obj.dD = 1 - 2*obj.bD;

            % Build the xD_tx matrix (2*L_slot x M)
            s_mls_cp = [p.s_mls(end - p.Ncp + 1 : end); p.s_mls];
            xD_tx_slots = zeros(p.ND + p.Ncp, 2 * p.M);
            
            comm_idx = 1;
            for k = 1:2*p.M
                if is_comm_slot(k)
                    xD_tx_slots(:, k) = s_mls_cp * obj.dD(comm_idx);
                    comm_idx = comm_idx + 1;
                else
                    xD_tx_slots(:, k) = s_mls_cp; % pilot symbol
                end
            end
            
            % Reshape to (2 * L_slot) x M
            obj.xD_tx = reshape(xD_tx_slots, [2 * (p.ND + p.Ncp), p.M]);
        end
    end
end
