classdef MobilityModel < handle
% MOBILITYMODEL  Linear mobility trajectories and per-block geometry.
%
%   Computes node positions at each slow-time block m under constant-velocity
%   (linear) motion:
%       p_node(m) = p_node(0) + v_node * m * Tblock
%
%   From the per-block positions, it derives:
%     - Inter-node distances  d_DU(m), d_DT(m), d_TU(m)
%     - One-way propagation delay bins  ell_DU(m), ell_DT(m) etc.
%     - One-way Doppler shifts  fD_DU(m), fD_DT(m), fD_TU(m)
%     - Composite Doppler shifts  nu_DU(m), nu_mono(m), nu_DTU(m)
%
%   Usage:
%       mob = ntn.MobilityModel(params, geom, M_seq);
%       mob.pD_seq(:,m)   % 3x1 drone position at block m
%       mob.dDU_seq(m)    % drone-UE distance at block m
%       mob.ell_DU_seq(m) % one-way delay bin at block m

    properties
        M_seq           % total number of blocks

        % Position sequences  (3 x M_seq)
        pD_seq          % Drone position [m]
        pU_seq          % UE position [m]
        pT_seq          % Target position [m]

        % Distance sequences  (1 x M_seq)
        dDU_seq
        dDT_seq
        dTU_seq

        % Integer chip delay bins  (1 x M_seq)  — round-tripped where appropriate
        ell_DU_seq      % D->U  one-way
        ell_DTU_seq     % D->T->U  two-hop
        ell_mono_seq    % D->T->D  monostatic
        ell_2DU_seq     % D->U->D  round-trip

        % One-way Doppler shifts [Hz]  (1 x M_seq)
        fD_DU_seq
        fD_DT_seq
        fD_TU_seq

        % Composite Doppler shifts [Hz]  (1 x M_seq)
        nu_DU_seq       % UE round-trip
        nu_mono_seq     % target monostatic round-trip
        nu_DTU_seq      % scattered D->T->U
    end

    methods
        function obj = MobilityModel(p, g, M_seq)
        % p     : ntn.SystemParams
        % g     : ntn.Geometry   (initial positions and velocities)
        % M_seq : number of blocks

            obj.M_seq = M_seq;

            % Pre-allocate
            obj.pD_seq = zeros(3, M_seq);
            obj.pU_seq = zeros(3, M_seq);
            obj.pT_seq = zeros(3, M_seq);

            obj.dDU_seq     = zeros(1, M_seq);
            obj.dDT_seq     = zeros(1, M_seq);
            obj.dTU_seq     = zeros(1, M_seq);

            obj.ell_DU_seq   = zeros(1, M_seq);
            obj.ell_DTU_seq  = zeros(1, M_seq);
            obj.ell_mono_seq = zeros(1, M_seq);
            obj.ell_2DU_seq  = zeros(1, M_seq);

            obj.fD_DU_seq    = zeros(1, M_seq);
            obj.fD_DT_seq    = zeros(1, M_seq);
            obj.fD_TU_seq    = zeros(1, M_seq);

            obj.nu_DU_seq    = zeros(1, M_seq);
            obj.nu_mono_seq  = zeros(1, M_seq);
            obj.nu_DTU_seq   = zeros(1, M_seq);

            % Compute per-block quantities
            for m = 1:M_seq
                t = (m - 1) * p.Tblock;   % time of block m [s]

                % Linear motion: p(t) = p(0) + v*t
                pD = (g.pD + g.vD * t).';   % 3x1
                pU = (g.pU + g.vU * t).';
                pT = (g.pT + g.vT * t).';

                obj.pD_seq(:, m) = pD;
                obj.pU_seq(:, m) = pU;
                obj.pT_seq(:, m) = pT;

                % Distances
                dDU = norm(pU - pD);
                dDT = norm(pT - pD);
                dTU = norm(pU - pT);
                obj.dDU_seq(m) = dDU;
                obj.dDT_seq(m) = dDT;
                obj.dTU_seq(m) = dTU;

                % Delay bins
                obj.ell_DU_seq(m)   = round(dDU / p.c0 * p.fs);
                obj.ell_DTU_seq(m)  = round((dDT + dTU) / p.c0 * p.fs);
                obj.ell_mono_seq(m) = round(2 * dDT / p.c0 * p.fs);
                obj.ell_2DU_seq(m)  = round(2 * dDU / p.c0 * p.fs);

                % LOS unit vectors
                uDU = (pU - pD) / dDU;
                uDT = (pT - pD) / dDT;
                uTU = (pU - pT) / dTU;

                % Doppler shifts — uses fixed velocities (constant-velocity model)
                obj.fD_DU_seq(m) = -dot(g.vU.' - g.vD.', uDU) / p.lamD;
                obj.fD_DT_seq(m) = -dot(g.vT.' - g.vD.', uDT) / p.lamD;
                obj.fD_TU_seq(m) = -dot(g.vU.' - g.vT.', uTU) / p.lamD;

                % Composite Dopplers
                obj.nu_DU_seq(m)    = 2 * obj.fD_DU_seq(m);
                obj.nu_mono_seq(m)  = 2 * obj.fD_DT_seq(m);
                obj.nu_DTU_seq(m)   = obj.fD_DT_seq(m) + obj.fD_TU_seq(m);
            end
        end
    end
end
