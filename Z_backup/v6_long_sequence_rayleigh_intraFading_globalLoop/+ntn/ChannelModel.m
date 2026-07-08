classdef ChannelModel < handle
% CHANNELMODEL  Sec. 2 — Distance-dependent path loss and Rayleigh
%   small-scale fading coefficients for all three links.
%
%   Channel:  h_ij = sqrt(beta_ij) * g_ij
%     beta_ij = C0 * (d_ij/d0)^{-alpha}   [large-scale path gain]
%     g_ij ~ CN(0,1)                       [Rayleigh small-scale fading]
%
%   Doppler rotation exp(j2pi f_{D,ij} t) is applied per block in
%   DroneReceiver and UEReceiver (block-fading assumption).
%
%   Usage:
%       ch = ntn.ChannelModel(params, geom);

    properties
        % Path-loss model
        d0    = 1       % reference distance [m]
        alpha = 2.0     % free-space exponent (NTN LOS)
        C0              % reference channel gain

        % Large-scale path gains
        beta_DT; beta_DU; beta_TU

        % Small-scale Rayleigh fading coefficients  g ~ CN(0,1)
        g_DT; g_DU; g_TU

        % Complex channel amplitudes  h = sqrt(beta)*g
        h_DT; h_DU; h_TU

        % Complex reflection / scattering coefficients
        eta_T           % passive target (small UAV RCS)
        eta_U           % UE surface reflection
    end

    methods
        function obj = ChannelModel(p, g)
        % p : ntn.SystemParams
        % g : ntn.Geometry
            obj.C0      = (p.lamD / (4*pi*obj.d0))^2;

            obj.beta_DT = obj.C0 * (g.dDT/obj.d0)^(-obj.alpha);
            obj.beta_DU = obj.C0 * (g.dDU/obj.d0)^(-obj.alpha);
            obj.beta_TU = obj.C0 * (g.dTU/obj.d0)^(-obj.alpha);

            obj.g_DT = (randn + 1j*randn) / sqrt(2);
            obj.g_DU = (randn + 1j*randn) / sqrt(2);
            obj.g_TU = (randn + 1j*randn) / sqrt(2);

            obj.h_DT = sqrt(obj.beta_DT) * obj.g_DT;
            obj.h_DU = sqrt(obj.beta_DU) * obj.g_DU;
            obj.h_TU = sqrt(obj.beta_TU) * obj.g_TU;

            % Set physical RCS values (in m^2)
            % sigma_T = 1e-2;     % Target RCS: -20 dBsm (Small UAV/Drone)
            sigma_T = 1e-3;     % Target RCS: -30 dBsm (Bird)
            sigma_U = 1e-1;     % UE RCS: -10 dBsm (Phone + User Hand/Body)          
            % Compute corresponding scattering parameters
            obj.eta_T = sqrt(4 * pi * sigma_T) / p.lamD * exp(1j * 2 * pi * rand);
            obj.eta_U = sqrt(4 * pi * sigma_U) / p.lamD * exp(1j * 2 * pi * rand);

            obj.print();
        end

        function print(obj)
            fprintf('--- Channel (Sec. 2) ---\n');
            fprintf('beta_DT=%.3e  beta_DU=%.3e  beta_TU=%.3e\n', ...
                    obj.beta_DT, obj.beta_DU, obj.beta_TU);
            fprintf('|h_DT|=%.4e   |h_DU|=%.4e   |h_TU|=%.4e\n', ...
                    abs(obj.h_DT), abs(obj.h_DU), abs(obj.h_TU));
            fprintf('|eta_T|=%.4f   |eta_U|=%.4f\n\n', ...
                    abs(obj.eta_T), abs(obj.eta_U));
        end
    end
end
