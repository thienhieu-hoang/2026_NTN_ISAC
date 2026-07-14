classdef BERAnalysis < handle
% BERANALYSIS  Sec. 4 — Helper class for BPSK Rayleigh fading theoretical curves.
%
%   Computes three BER curves:
%     BER_theory     : BPSK Rayleigh reference with perfect CSI
%                      Pe = 0.5 * (1 - sqrt(EbN0 / (1 + EbN0)))
%     BER_theory_est : Analytical pilot-assisted BPSK under Rayleigh fading
%                      Pe = 1 / (2 * (1 + EbN0))
%     BER_all        : Average BER over all blocks (filled by run_simulation.m)

    properties
        EbN0_dB    = 0:2:20    % Eb/N0 sweep points [dB]
        M_seq      = 256       % Number of blocks in the continuous sequence
        N_trials   = 200       % Monte Carlo trials per SNR point
        model_type             % char ('static', 'ar1', or 'jakes')

        BER_sim                 % 1 x numel(EbN0_dB)  simulated BER (clean blocks 2..M_seq)
        BER_block1              % 1 x numel(EbN0_dB)  simulated BER of first block (transient)
        BER_all                 % 1 x numel(EbN0_dB)  simulated BER of entire sequence (average)
        BER_theory              % 1 x numel(EbN0_dB)  BPSK Rayleigh fading (perfect CSI)
        BER_theory_est          % 1 x numel(EbN0_dB)  BPSK Rayleigh fading (noisy pilot)

        rho_norm                % scalar  MLS cross-correlation rho(ell_DTU)
        SINR_supp               % scalar  SINR with scatter suppression
    end

    methods
        function obj = BERAnalysis(model_type)
            if nargin < 1
                model_type = 'static';
            end
            obj.model_type = model_type;
            
            % Pre-compute theoretical BER curves
            EbN0_lin           = 10.^(obj.EbN0_dB/10);
            obj.BER_sim        = zeros(size(obj.EbN0_dB));
            obj.BER_block1     = zeros(size(obj.EbN0_dB));
            obj.BER_all        = zeros(size(obj.EbN0_dB));
            
            if strcmpi(obj.model_type, 'static')
                obj.BER_theory     = 0.5 * erfc(sqrt(EbN0_lin));
                obj.BER_theory_est = 0.5 * exp(-EbN0_lin);
            else
                obj.BER_theory     = 0.5 * (1 - sqrt(EbN0_lin ./ (1 + EbN0_lin)));
                obj.BER_theory_est = 1 ./ (2 * (1 + EbN0_lin));
            end
        end
    end
end
