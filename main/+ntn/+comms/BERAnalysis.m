classdef BERAnalysis < handle
% BERANALYSIS  Sec. 4 — Helper class for BPSK fading theoretical curves.
%
%   Computes three BER curves:
%     BER_theory     : BPSK reference with perfect CSI
%     BER_theory_est : Analytical pilot-assisted BPSK under fading
%     BER_all        : Average BER over all blocks (filled by run_simulation.m)

    properties
        EbN0_dB    = 0:2:20    % Eb/N0 sweep points [dB]
        M_seq      = 256       % Number of blocks in the continuous sequence
        N_trials   = 200       % Monte Carlo trials per SNR point
        model_type             % char ('static', 'ar1', 'jakes', 'rayleigh-static', 'nakagami', 'nakagami-static')
        m_nakagami = 2.0       % Nakagami shape parameter (default 2.0)

        BER_sim                 % 1 x numel(EbN0_dB)  simulated BER (clean blocks 2..M_seq)
        BER_block1              % 1 x numel(EbN0_dB)  simulated BER of first block (transient)
        BER_all                 % 1 x numel(EbN0_dB)  simulated BER of entire sequence (average)
        BER_theory              % 1 x numel(EbN0_dB)  BPSK fading (perfect CSI)
        BER_theory_est          % 1 x numel(EbN0_dB)  BPSK fading (noisy pilot)

        rho_norm                % scalar  MLS cross-correlation rho(ell_DTU)
        SINR_supp               % scalar  SINR with scatter suppression
    end

    methods
        function obj = BERAnalysis(model_type, m_nakagami)
            if nargin < 1
                model_type = 'static';
            end
            obj.model_type = model_type;
            
            if nargin >= 2
                obj.m_nakagami = m_nakagami;
            end
            
            % Pre-compute theoretical BER curves
            EbN0_lin           = 10.^(obj.EbN0_dB/10);
            obj.BER_sim        = zeros(size(obj.EbN0_dB));
            obj.BER_block1     = zeros(size(obj.EbN0_dB));
            obj.BER_all        = zeros(size(obj.EbN0_dB));
            
            if strcmpi(obj.model_type, 'static')
                obj.BER_theory     = 0.5 * erfc(sqrt(EbN0_lin));
                obj.BER_theory_est = 0.5 * exp(-EbN0_lin);
            elseif strcmpi(obj.model_type, 'nakagami') || strcmpi(obj.model_type, 'nakagami-static')
                m = obj.m_nakagami;
                % Perfect CSI: Nakagami-m theoretical BER
                if mod(m, 1) == 0  % integer m
                    mu = sqrt(EbN0_lin ./ (m + EbN0_lin));
                    sum_term = zeros(size(EbN0_lin));
                    for k = 0:(m-1)
                        sum_term = sum_term + nchoosek(m-1+k, k) * ((1 + mu)/2).^k;
                    end
                    obj.BER_theory = ((1 - mu)/2).^m .* sum_term;
                else  % non-integer m: numerical integration of Q-function over Gamma PDF
                    obj.BER_theory = zeros(size(EbN0_lin));
                    for idx = 1:length(EbN0_lin)
                        bar_gamma = EbN0_lin(idx);
                        % Gamma PDF: pdf(g) = exp( m*log(m) - m*log(bar_gamma) - gammaln(m) + (m-1)*log(g) - m*g/bar_gamma )
                        log_coeff = m * log(m) - m * log(bar_gamma) - gammaln(m);
                        fun = @(g) 0.5 * erfc(sqrt(g)) .* exp(log_coeff + (m-1)*log(g) - m*g/bar_gamma);
                        obj.BER_theory(idx) = integral(fun, 0, inf);
                    end
                end
                
                % Noisy pilot (pilot-assisted): 0.5 * (1 + EbN0/m)^(-m)
                obj.BER_theory_est = 0.5 * (1 + EbN0_lin ./ m).^(-m);
            else
                % Rayleigh (ar1, jakes, rayleigh-static, etc.)
                obj.BER_theory     = 0.5 * (1 - sqrt(EbN0_lin ./ (1 + EbN0_lin)));
                obj.BER_theory_est = 1 ./ (2 * (1 + EbN0_lin));
            end
        end
    end
end
