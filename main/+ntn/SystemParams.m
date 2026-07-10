classdef SystemParams < handle
% SYSTEMPARAMS  Sec. 1 — System & waveform parameters for the NTN-ISAC
%   three-node PMCW simulation.
%
%   Usage:
%       p = ntn.SystemParams();
%
%   All scalar constants and the MLS code are set here. Other classes
%   receive a SystemParams object via constructor injection.

    properties
        % Physical constants
        c0     = 3e8        % speed of light [m/s]

        % Carrier
        fcD    = 24e9       % drone carrier frequency [Hz]
        lamD                % wavelength [m]

        % PMCW waveform
        B      = 50e6       % chip rate / bandwidth [Hz]
        Tc                  % chip duration [s]
        fs                  % sampling rate = chip rate [Hz]
        mLFSR  = 7          % MLS order  ->  ND = 2^m - 1
        ND                  % code length (chips)
        Ncp    = 34         % Cyclic Prefix length (chips)
        M      = 256        % number of slow-time blocks
        L_sound = 1         % sounding period interval (default 1)
        TPRI                % one code period + CP = half-block [s]
        Tblock              % full block: [sounding | data] [s]
        PD     = 1          % drone Tx power (normalised)

        % MLS waveform
        s_mls               % ND x 1  polar {+1,-1}

        % Radar performance limits
        dR                  % range resolution [m]
        Rmax                % max unambiguous range [m]
        dv                  % velocity resolution [m/s]
        vmax                % max unambiguous velocity [m/s]
    end

    methods
        function obj = SystemParams()
            obj.lamD   = obj.c0 / obj.fcD;
            obj.Tc     = 1 / obj.B;
            obj.fs     = obj.B;
            obj.ND     = 2^obj.mLFSR - 1;
            obj.TPRI   = (obj.ND + obj.Ncp) * obj.Tc;
            obj.Tblock = 2 * obj.TPRI;
            obj.s_mls  = ntn.SystemParams.gen_mls(obj.mLFSR);

            obj.dR   = obj.c0 / (2 * obj.fs);
            obj.Rmax = obj.c0 * obj.ND * obj.Tc / 2;
            obj.dv   = obj.lamD / (2 * obj.M * obj.Tblock);
            obj.vmax = obj.lamD / (4 * obj.Tblock);

            obj.print();
        end

        function print(obj)
            fprintf('=== NTN-ISAC 3-node PMCW simulation (with channel model) ===\n');
            fprintf('N_D=%d  M=%d  T_PRI=%.2f us  T_block=%.2f us\n', ...
                    obj.ND, obj.M, obj.TPRI*1e6, obj.Tblock*1e6);
            fprintf('dR=%.2f m  Rmax=%.0f m  dv=%.4f m/s  vmax=%.2f m/s\n\n', ...
                    obj.dR, obj.Rmax, obj.dv, obj.vmax);
        end
    end

    methods (Static)
        function s = gen_mls(m)
        % GEN_MLS  Generate a maximal-length sequence (LFSR), binary {0,1}
        %   mapped to polar {+1,-1}.
            switch m
                case 7,  taps = [7 6];
                case 8,  taps = [8 6 5 4];
                case 9,  taps = [9 5];
                case 10, taps = [10 7];
                otherwise, error('ntn:SystemParams:unsupportedOrder', ...
                    'Add primitive-polynomial taps for m=%d', m);
            end
            N   = 2^m - 1;
            reg = ones(1, m);
            c   = zeros(N, 1);
            for i = 1:N
                c(i) = reg(end);
                fb   = mod(sum(reg(taps)), 2);
                reg  = [fb reg(1:end-1)];
            end
            s = 1 - 2*c;   % {0,1} -> {+1,-1}
        end
    end
end
