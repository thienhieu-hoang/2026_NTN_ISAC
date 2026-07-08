classdef FadingSequence < handle
% FADINGSEQUENCE  AR(1) time-correlated Rayleigh fading sequences.
%
%   Generates M_seq smoothly-varying complex fading samples g[m] ~ CN(0,1)
%   using the first-order autoregressive (AR(1)) model:
%
%       g[m] = alpha * g[m-1] + sqrt(1 - alpha^2) * w[m]
%
%   where w[m] ~ CN(0,1) is i.i.d., and alpha = J_0(2*pi*fd*Tblock) is
%   derived from Jakes' model.  A high node speed -> large fd -> small alpha
%   -> faster fading.  A slow or stationary node gives alpha ~ 1 (very slow
%   fading).
%
%   Three independent sequences are generated for the three links:
%       g_DU(m)  — Drone -> UE direct path
%       g_DT(m)  — Drone -> Target path
%       g_TU(m)  — Target -> UE path
%
%   Usage:
%       fs = ntn.FadingSequence(params, geom, M_seq);
%       fs.g_DU   % 1 x M_seq  complex fading for D-U link
%       fs.g_DT   % 1 x M_seq  complex fading for D-T link
%       fs.g_TU   % 1 x M_seq  complex fading for T-U link

    properties
        M_seq       % number of blocks

        g_DU        % 1 x M_seq  D-U link fading
        g_DT        % 1 x M_seq  D-T link fading
        g_TU        % 1 x M_seq  T-U link fading

        alpha_DU    % AR(1) correlation coefficient for D-U link
        alpha_DT    % AR(1) correlation coefficient for D-T link
        alpha_TU    % AR(1) correlation coefficient for T-U link

        fd_DU       % max Doppler on D-U link [Hz]
        fd_DT       % max Doppler on D-T link [Hz]
        fd_TU       % max Doppler on T-U link [Hz]
    end

    methods
        function obj = FadingSequence(p, g, M_seq)
        % p     : ntn.SystemParams
        % g     : ntn.Geometry   (provides velocities and initial Doppler estimates)
        % M_seq : number of blocks

            obj.M_seq = M_seq;

            % Maximum Doppler frequency for each link [Hz]
            % |f_D| = |v_rel_radial| / lambda = max relative speed / lambda
            % We use the magnitude of relative velocity projected onto LOS
            % (absolute value of the computed one-way Doppler as an estimate)
            obj.fd_DU = abs(g.fD_DU);
            obj.fd_DT = abs(g.fD_DT);
            obj.fd_TU = abs(g.fD_TU);

            % AR(1) correlation coefficient: alpha = J_0(2*pi*fd*Tblock)
            % J_0: zeroth-order Bessel function of first kind
            obj.alpha_DU = besselj(0, 2*pi*obj.fd_DU * p.Tblock);
            obj.alpha_DT = besselj(0, 2*pi*obj.fd_DT * p.Tblock);
            obj.alpha_TU = besselj(0, 2*pi*obj.fd_TU * p.Tblock);

            % Generate AR(1) fading sequences
            obj.g_DU = ntn.FadingSequence.ar1_sequence(obj.alpha_DU, M_seq);
            obj.g_DT = ntn.FadingSequence.ar1_sequence(obj.alpha_DT, M_seq);
            obj.g_TU = ntn.FadingSequence.ar1_sequence(obj.alpha_TU, M_seq);

            % fprintf('--- FadingSequence (AR(1) Rayleigh) ---\n');
            % fprintf('  D-U: fd=%.2f Hz, alpha=%.6f  (Tblock=%.3f us)\n', ...
            %         obj.fd_DU, obj.alpha_DU, p.Tblock*1e6);
            % fprintf('  D-T: fd=%.2f Hz, alpha=%.6f\n', obj.fd_DT, obj.alpha_DT);
            % fprintf('  T-U: fd=%.2f Hz, alpha=%.6f\n\n', obj.fd_TU, obj.alpha_TU);
        end
    end

    methods (Static)
        function g = ar1_sequence(alpha, M)
        % AR1_SEQUENCE  Generate one AR(1) complex Gaussian fading sequence.
        %   alpha : correlation coefficient (Jakes J_0 value)
        %   M     : sequence length (number of blocks)
        %
        %   Returns g: 1 x M complex vector, g[m] ~ CN(0,1) marginally.

            g    = zeros(1, M);
            g(1) = (randn + 1j*randn) / sqrt(2);
            noise_scale = sqrt(1 - alpha^2);
            for m = 2:M
                w    = (randn + 1j*randn) / sqrt(2);
                g(m) = alpha * g(m-1) + noise_scale * w;
            end
        end
    end
end
