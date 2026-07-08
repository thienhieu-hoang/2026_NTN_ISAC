classdef Geometry < handle
% GEOMETRY  Sec. 1 — Node positions, velocities, propagation delays,
%   and Doppler shifts for the three-node NTN-ISAC system.
%
%   Usage:
%       g = ntn.Geometry(params);

    properties
        % Node positions [m] and velocities [m/s]
        pD = [  0,  0, 100];   vD = [ 15,  0,  0];   % Drone 54 km/h
        pU = [180,  0,   0];   vU = [  0,  8,  0];   % UE    28.8 km/h
        pT = [120, 40,  90];   vT = [-11, 4,  1];   % Moving target 11.747m/s - 42.3 km/h

        % Link distances [m]
        dDU; dDT; dTU

        % Integer chip delay bins
        ell_mono    % D->T->D  round-trip (monostatic)
        ell_2DU     % D->U->D  round-trip (UE echo at drone)
        ell_DU      % D->U     one-way    (comm path)
        ell_DTU     % D->T->U  two-hop    (scattered interference)

        % LOS unit vectors
        uDU; uDT; uTU

        % One-way Doppler shifts [Hz]
        fD_DU; fD_DT; fD_TU

        % Composite Doppler shifts [Hz]
        nu_mono     % target two-way Doppler (drone Rx)
        nu_DU       % UE two-way Doppler     (drone Rx)
        nu_DTU      % scattered D->T->U Doppler at UE
    end

    methods
        function obj = Geometry(p)
        % p : ntn.SystemParams
            obj.dDU = norm(obj.pU - obj.pD);
            obj.dDT = norm(obj.pT - obj.pD);
            obj.dTU = norm(obj.pU - obj.pT);

            obj.ell_mono = round(2*obj.dDT / p.c0 * p.fs);
            obj.ell_2DU  = round(2*obj.dDU / p.c0 * p.fs);
            obj.ell_DU   = round(  obj.dDU / p.c0 * p.fs);
            obj.ell_DTU  = round((obj.dDT + obj.dTU) / p.c0 * p.fs);

            obj.uDU = (obj.pU - obj.pD) / obj.dDU;
            obj.uDT = (obj.pT - obj.pD) / obj.dDT;
            obj.uTU = (obj.pU - obj.pT) / obj.dTU;

            obj.fD_DU = -dot(obj.vU - obj.vD, obj.uDU) / p.lamD;
            obj.fD_DT = -dot(obj.vT - obj.vD, obj.uDT) / p.lamD;
            obj.fD_TU = -dot(obj.vU - obj.vT, obj.uTU) / p.lamD;

            obj.nu_mono = 2 * obj.fD_DT;
            obj.nu_DU   = 2 * obj.fD_DU;
            obj.nu_DTU  = obj.fD_DT + obj.fD_TU;

            obj.print(p);
        end

        function print(obj, p)
            fprintf('--- Geometry ---\n');
            fprintf('dDT=%.1f m  dDU=%.1f m  dTU=%.1f m\n', ...
                    obj.dDT, obj.dDU, obj.dTU);
            fprintf('Target : range bin %d (%.0f m),  v_r=%.2f m/s,  nu_mono=%.2f Hz\n', ...
                    obj.ell_mono, obj.ell_mono*p.dR, obj.nu_mono*p.lamD/2, obj.nu_mono);
            fprintf('UE     : range bin %d (%.0f m),  v_r=%.2f m/s,  nu_DU=%.2f Hz\n\n', ...
                    obj.ell_2DU, obj.ell_2DU*p.dR, obj.nu_DU*p.lamD/2, obj.nu_DU);
        end
    end
end
