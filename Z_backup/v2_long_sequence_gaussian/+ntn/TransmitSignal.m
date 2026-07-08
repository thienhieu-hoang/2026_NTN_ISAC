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
        bD      % 1 x M  random binary bits {0,1}
        dD      % 1 x M  polar BPSK symbols {+1,-1}
        m_idx   % 1 x M  block index vector  0:M-1
        xD_tx   % 2*ND x M  full transmit matrix [sounding; data]
    end

    methods
        function obj = TransmitSignal(p)
        % p : ntn.SystemParams
            obj.bD    = randi([0 1], 1, p.M);
            obj.dD    = 1 - 2*obj.bD;
            obj.m_idx = 0:p.M-1;

            % [sounding half (d_m=+1); data half (d_m=±1)]
            obj.xD_tx = [p.s_mls * ones(1, p.M); ...
                         p.s_mls * obj.dD];
        end
    end
end
