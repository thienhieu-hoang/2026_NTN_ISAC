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
            h_hat_all = obj.R_sound(obj.ell_DU_est + 1, :) / p.ND;
            
            obj.h_hat = zeros(1, p.M);
            if isprop(p, 'sounding_config') && ~isempty(p.sounding_config)
                a = p.sounding_config(1);
                b = p.sounding_config(2);
                
                num_periods = ceil(p.M / (b - 1));
                for k = 1:num_periods
                    S_k = (k - 1) * (b - 1) + 1;
                    if S_k > p.M
                        break;
                    end
                    
                    % First sounding block in period
                    obj.h_hat(S_k) = h_hat_all(S_k);
                    
                    idx_second = S_k + a - 1;
                    if idx_second <= p.M
                        % Second sounding block in period
                        obj.h_hat(idx_second) = h_hat_all(idx_second);
                        
                        h1 = h_hat_all(S_k);
                        h2 = h_hat_all(idx_second);
                        
                        % Doppler phase rotation per block
                        delta_theta = angle(h2 * conj(h1));
                        psi = delta_theta / (a - 1);
                        
                        % Interpolate for blocks between S_k and S_k + a - 1 (if a > 2)
                        for m = (S_k + 1):(idx_second - 1)
                            if m <= p.M
                                t_frac = (m - S_k) / (a - 1);
                                interp_amp = abs(h1) + (abs(h2) - abs(h1)) * t_frac;
                                obj.h_hat(m) = interp_amp * exp(1j * (angle(h1) + psi * (m - S_k)));
                            end
                        end
                        
                        % Extrapolate/predict for blocks after the second sounding block
                        if S_k + b - 1 <= p.M
                            upper_limit = S_k + b - 2;
                        else
                            upper_limit = p.M;
                        end
                        
                        for m = (idx_second + 1):upper_limit
                            if m <= p.M
                                obj.h_hat(m) = h2 * exp(1j * psi * (m - idx_second));
                            end
                        end
                    else
                        % If only S_k is within bounds, hold its estimate
                        for m = (S_k + 1):p.M
                            obj.h_hat(m) = h_hat_all(S_k);
                        end
                    end
                end
            else
                % Fallback to L_sound zero-order hold
                for m = 1:p.M
                    last_sounding_idx = floor((m-1)/p.L_sound) * p.L_sound + 1;
                    obj.h_hat(m) = h_hat_all(last_sounding_idx);
                end
            end

            % fprintf('--- Channel estimation at UE (Sec. 4.1, Step 1) ---\n');
            % fprintf('Estimated UE delay: %d chips (True: %d)\n', ...
            %         obj.ell_DU_est, g.ell_DU);
            % fprintf('True |A_DU|=%.4e   Mean|h_hat|=%.4e\n\n', ...
            %         abs(ueRx.A_DU), mean(abs(obj.h_hat)));
        end
    end
end
