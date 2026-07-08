classdef DroneReceiver < handle
% DRONERECEIVER  Sec. 5 — Drone Rx signal construction and sensing pipeline.
%
%   The drone receives backscatter from both the UE (D->U->D) and the
%   moving target (D->T->D).  Processing steps:
%     1. Generate full 2ND x M received signal
%     2. computeRangeDoppler() : BPSK data strip + coherent integration + RD map
%     3. applySIC()            : detect dominant peak, subtract, re-map
%
%   Usage:
%       droneRx = ntn.sensing.DroneReceiver(params, geom, channel, txSignal);
%       droneRx.computeRangeDoppler(params, txSignal);
%       droneRx.applySIC(params, geom);

    properties
        % Echo amplitudes
        aT              % target echo amplitude
        aU              % UE echo amplitude
        sigD            % drone receiver noise std
        SNR_target_dB = 18   % desired single-period target SNR [dB]

        % Received signal (full 2*ND x M frame)
        yD_full
        yD_sound        % ND x M  sounding half
        yD_data         % ND x M  data half

        % Sensing pipeline outputs
        yD_int          % ND x M  coherently integrated signal
        RD_before       % ND x M  Range-Doppler map (before SIC)
        yD_int_sic      % ND x M  signal after SIC
        tgEst           % struct  SIC estimates (.ell, .nu, .A)
        RD_after        % ND x M  Range-Doppler map (after SIC)

        % Display axes
        rng_axis        % ND x 1  range axis [m]
        fdopp           % 1 x M   Doppler axis [Hz]
        vel_axis        % 1 x M   velocity axis [m/s]
    end

    methods
        function obj = DroneReceiver(p, g, ch, tx)
        % p  : ntn.SystemParams
        % g  : ntn.Geometry
        % ch : ntn.ChannelModel
        % tx : ntn.TransmitSignal

            % Two-way echo amplitudes  a_i = eta_i * beta_Di * g_Di^2 * sqrt(PD)
            obj.aT   = ch.eta_T * ch.beta_DT * ch.g_DT^2 * sqrt(p.PD);
            obj.aU   = ch.eta_U * ch.beta_DU * ch.g_DU^2 * sqrt(p.PD);

            % Noise variance: single-period target SNR = SNR_target_dB
            obj.sigD = abs(obj.aT) * sqrt(p.ND * p.M) / ...
                       sqrt(10^(obj.SNR_target_dB/10));

            fprintf('--- Drone Rx amplitudes (Sec. 5) ---\n');
            fprintf('|aT|=%.4e  |aU|=%.4e  UE/target ratio=%.1f dB\n', ...
                    abs(obj.aT), abs(obj.aU), 20*log10(abs(obj.aU)/abs(obj.aT)));
            fprintf('sigD=%.4e  (single-period target SNR = %d dB; after integration = %.0f dB)\n\n', ...
                    obj.sigD, obj.SNR_target_dB, obj.SNR_target_dB + 10*log10(2));

            % Slow-time Doppler phase vectors (1 x M)
            phT = exp(1j*2*pi*g.nu_mono * tx.m_idx * p.Tblock);
            phU = exp(1j*2*pi*g.nu_DU   * tx.m_idx * p.Tblock);

            % Split transmit signal into sounding and data halves to apply delay separately.
            % We assume there are guard intervals between blocks so that the circular shift
            % does not cause inter-period leakage (clean blocks).
            xD_sound = tx.xD_tx(1:p.ND, :);
            xD_data  = tx.xD_tx(p.ND+1:end, :);

            % Apply delay separately to sounding and data periods
            yD_sound_clean = obj.aT * circshift(xD_sound, [g.ell_mono, 0]) .* phT ...
                           + obj.aU * circshift(xD_sound, [g.ell_2DU,  0]) .* phU;

            yD_data_clean  = obj.aT * circshift(xD_data, [g.ell_mono, 0]) .* phT ...
                           + obj.aU * circshift(xD_data, [g.ell_2DU,  0]) .* phU;

            % Add noise and assign to sounding and data variables
            obj.yD_sound = yD_sound_clean + obj.sigD/sqrt(2) * (randn(p.ND, p.M) + 1j*randn(p.ND, p.M));
            obj.yD_data  = yD_data_clean  + obj.sigD/sqrt(2) * (randn(p.ND, p.M) + 1j*randn(p.ND, p.M));

            % Stitch the clean parts back together for completeness of yD_full
            obj.yD_full  = [obj.yD_sound; obj.yD_data];

            % Pre-compute display axes
            obj.rng_axis = (0:p.ND-1) * p.dR;
            obj.fdopp    = ((0:p.M-1) - floor(p.M/2)) / (p.M * p.Tblock);
            obj.vel_axis = obj.fdopp * p.lamD / 2;
        end

        function computeRangeDoppler(obj, p, tx)
        % Step 1: BPSK data stripping  ->  coherent integration  ->  RD map.
        %   p  : ntn.SystemParams
        %   tx : ntn.TransmitSignal

            % Strip BPSK modulation from data half (d_m^2 = 1)
            ypD_data  = obj.yD_data .* tx.dD;

            % Coherent integration: sounding + stripped-data
            %   Signal power x4 (+6 dB), noise power x2 (+3 dB) => net +3 dB
            obj.yD_int = obj.yD_sound + ypD_data;

            fprintf('--- Drone Rx: coherent integration (sounding + data halves) ---\n');
            fprintf('Signal amplitude: x2  =>  +6 dB signal power\n');
            fprintf('Noise power    : x2  =>  +3 dB noise power\n');
            fprintf('Net SNR gain   : +3 dB vs. single-half processing\n\n');

            obj.RD_before = ntn.sensing.RangeDopplerMap(obj.yD_int, p.s_mls, p.M);
        end

        function applySIC(obj, p, g)
        % Step 5: SIC — remove dominant peak, re-compute RD map.
        %   p : ntn.SystemParams
        %   g : ntn.Geometry

            [obj.yD_int_sic, obj.tgEst] = ...
                ntn.sensing.SIC(obj.yD_int, p.s_mls, p.M, p.Tblock);
            obj.RD_after = ntn.sensing.RangeDopplerMap(obj.yD_int_sic, p.s_mls, p.M);

            % Determine which reflector is closer to the estimated dominant peak
            d_to_UE = abs(obj.tgEst.ell - g.ell_2DU);
            d_to_Target = abs(obj.tgEst.ell - g.ell_mono);

            fprintf('--- SIC result ---\n');
            if d_to_UE < d_to_Target
                fprintf('Dominant peak removed: ell_est=%d (true %d),  nu_est=%.2f Hz (true %.2f Hz)\n', ...
                        obj.tgEst.ell, g.ell_2DU, obj.tgEst.nu, g.nu_DU);
                fprintf('Remaining peak unmasked at: true_ell=%d (%.0f m),  true_nu=%.2f Hz\n\n', ...
                        g.ell_mono, g.ell_mono*p.dR, g.nu_mono);
            else
                fprintf('Dominant peak removed: ell_est=%d (true %d),  nu_est=%.2f Hz (true %.2f Hz)\n', ...
                        obj.tgEst.ell, g.ell_mono, obj.tgEst.nu, g.nu_mono);
                fprintf('Remaining peak unmasked at: true_ell=%d (%.0f m),  true_nu=%.2f Hz\n\n', ...
                        g.ell_2DU, g.ell_2DU*p.dR, g.nu_DU);
            end
        end
    end
end
