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
        a3hop           % three-hop target-UE echo amplitude (D->T->U->D / D->U->T->D)
        sigD            % drone receiver noise std
        SNR_target_dB = 18   % desired single-period target SNR [dB]
        ell_mono        % target monostatic delay (stored for range display)
        ell_3hop        % three-hop target-UE delay (round-trip)

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
        function obj = DroneReceiver(p, g, ch, tx, mob, fading)
        % p  : ntn.SystemParams
        % g  : ntn.Geometry
        % ch : ntn.ChannelModel
        % tx : ntn.TransmitSignal
        % mob    : ntn.MobilityModel (optional, slot-level)
        % fading : ntn.FadingSequence (optional, slot-level)

            if nargin < 5
                % --- Original Static/Quasi-Static Mode ---
                % Two-way echo amplitudes  a_i = eta_i * beta_Di * g_Di^2 * sqrt(PD)
                obj.aT   = ch.eta_T * ch.beta_DT * ch.g_DT^2 * sqrt(p.PD);
                obj.aU   = ch.eta_U * ch.beta_DU * ch.g_DU^2 * sqrt(p.PD);
                obj.ell_mono = g.ell_mono;

                % Three-hop double-scatter path: D->T->U->D and D->U->T->D (symmetric)
                obj.ell_3hop = round((g.dDU + g.dTU + g.dDT) / p.c0 * p.fs);
                obj.a3hop    = 2 * ch.eta_T * ch.eta_U * sqrt(ch.beta_DU * ch.beta_TU * ch.beta_DT) * ...
                               (ch.g_DU * ch.g_TU * ch.g_DT) * sqrt(p.PD);

                % Noise variance: single-period target SNR = SNR_target_dB
                obj.sigD = abs(obj.aT) * sqrt(p.ND * p.M) / ...
                           sqrt(10^(obj.SNR_target_dB/10));

                fprintf('--- Drone Rx amplitudes (Static Mode) ---\n');
                fprintf('|aT|=%.4e  |aU|=%.4e  |a3hop|=%.4e  UE/target ratio=%.1f dB\n', ...
                        abs(obj.aT), abs(obj.aU), abs(obj.a3hop), 20*log10(abs(obj.aU)/abs(obj.aT)));
                fprintf('sigD=%.4e  (single-period target SNR = %d dB; after integration = %.0f dB)\n\n', ...
                        obj.sigD, obj.SNR_target_dB, obj.SNR_target_dB + 10*log10(2));

                % Slow-time Doppler phase vectors (1 x M)
                phT = exp(1j*2*pi*g.nu_mono * tx.m_idx * p.Tblock);
                phU = exp(1j*2*pi*g.nu_DU   * tx.m_idx * p.Tblock);
                
                % Doppler shift of the three-hop path is the sum of all three segments' Doppler shifts
                nu_3hop = g.fD_DU + g.fD_DT + g.fD_TU;
                ph3hop = exp(1j*2*pi*nu_3hop * tx.m_idx * p.Tblock);

                % Flatten transmit signal matrix to a 1D vector (continuous stream)
                xD_tx_flat = tx.xD_tx(:);

                % Replicate slow-time Doppler phase vectors over the chips for the sequence
                phT_chips = kron(phT, ones(2*(p.ND + p.Ncp), 1));
                phT_chips_flat = phT_chips(:);
                
                phU_chips = kron(phU, ones(2*(p.ND + p.Ncp), 1));
                phU_chips_flat = phU_chips(:);
                
                ph3hop_chips = kron(ph3hop, ones(2*(p.ND + p.Ncp), 1));
                ph3hop_chips_flat = ph3hop_chips(:);

                % Generate received signal
                L_block = 2*(p.ND + p.Ncp);
                yD_full_flat = obj.aT * circshift(xD_tx_flat, [g.ell_mono, 0]) .* phT_chips_flat ...
                             + obj.aU * circshift(xD_tx_flat, [g.ell_2DU,  0]) .* phU_chips_flat ...
                             + obj.a3hop * circshift(xD_tx_flat, [obj.ell_3hop, 0]) .* ph3hop_chips_flat ...
                             + obj.sigD/sqrt(2) * (randn(L_block*p.M, 1) + 1j*randn(L_block*p.M, 1));

                % --- Drone Receiver Processing (Window alignment to Target delay & CP Discard) ---
                yD_aligned = circshift(yD_full_flat, [-g.ell_mono, 0]);
                yD_full_aligned_mat = reshape(yD_aligned, [L_block, p.M]);
                
                yD_sound_cp = yD_full_aligned_mat(1 : p.ND + p.Ncp, :);
                yD_data_cp  = yD_full_aligned_mat(p.ND + p.Ncp + 1 : end, :);
                
                obj.yD_sound = yD_sound_cp(p.Ncp + 1 : end, :);
                obj.yD_data  = yD_data_cp(p.Ncp + 1 : end, :);
            else
                % --- Dynamic Time-Varying (Slot-Level) Mode ---
                C0          = (p.lamD / (4*pi))^2;
                beta_DT_seq = C0 * mob.dDT_seq .^ (-2);
                beta_DU_seq = C0 * mob.dDU_seq .^ (-2);
                beta_TU_seq = C0 * mob.dTU_seq .^ (-2);
                
                aT_seq = ch.eta_T * beta_DT_seq .* (fading.g_DT .^ 2) * sqrt(p.PD);
                aU_seq = ch.eta_U * beta_DU_seq .* (fading.g_DU .^ 2) * sqrt(p.PD);
                a3hop_seq = 2 * ch.eta_T * ch.eta_U * sqrt(beta_DU_seq .* beta_TU_seq .* beta_DT_seq) .* ...
                            (fading.g_DU .* fading.g_TU .* fading.g_DT) * sqrt(p.PD);

                obj.aT = aT_seq(1);
                obj.aU = aU_seq(1);
                obj.a3hop = a3hop_seq(1);
                obj.ell_mono = mob.ell_mono_seq(1);
                obj.ell_3hop = round((mob.dDU_seq(1) + mob.dTU_seq(1) + mob.dDT_seq(1)) / p.c0 * p.fs);

                % Noise variance based on average target echo strength
                avg_aT = abs(ch.eta_T) * (C0 * mob.dDT_seq(1)^(-2)) * sqrt(p.PD);
                obj.sigD = avg_aT * sqrt(p.ND * p.M) / sqrt(10^(obj.SNR_target_dB/10));

                L_slot = p.ND + p.Ncp;
                L_block = 2 * L_slot;
                yD_full_flat = zeros(L_block * p.M, 1);
                xD_tx_flat = tx.xD_tx(:);

                for m = 1:p.M
                    blk_start = (m-1)*L_block + 1;
                    blk_end   = m*L_block;

                    % Sounding slot (first half of block m)
                    idx_S      = 2*m - 1;
                    ell_mono_S = mob.ell_mono_seq(idx_S);
                    ell_2du_S  = mob.ell_2DU_seq(idx_S);
                    ell_3hop_S = round((mob.dDU_seq(idx_S) + mob.dTU_seq(idx_S) + mob.dDT_seq(idx_S)) / p.c0 * p.fs);
                    
                    phT_S      = exp(1j*2*pi*mob.nu_mono_seq(idx_S) * (idx_S - 1) * p.TPRI);
                    phU_S      = exp(1j*2*pi*mob.nu_DU_seq(idx_S) * (idx_S - 1) * p.TPRI);
                    nu_3hop_S  = mob.fD_DU_seq(idx_S) + mob.fD_DT_seq(idx_S) + mob.fD_TU_seq(idx_S);
                    ph3hop_S   = exp(1j*2*pi*nu_3hop_S * (idx_S - 1) * p.TPRI);
                    
                    xD_del_T_S  = circshift(xD_tx_flat, ell_mono_S);
                    xD_del_U_S  = circshift(xD_tx_flat, ell_2du_S);
                    xD_del_3hop_S = circshift(xD_tx_flat, ell_3hop_S);
                    
                    if m == 1
                        xD_del_T_S(1 : ell_mono_S) = 0;
                        xD_del_U_S(1 : ell_2du_S) = 0;
                        xD_del_3hop_S(1 : ell_3hop_S) = 0;
                    end

                    y_rx_S = aT_seq(idx_S) * xD_del_T_S(blk_start : blk_start + L_slot - 1) * phT_S ...
                           + aU_seq(idx_S) * xD_del_U_S(blk_start : blk_start + L_slot - 1) * phU_S ...
                           + a3hop_seq(idx_S) * xD_del_3hop_S(blk_start : blk_start + L_slot - 1) * ph3hop_S ...
                           + obj.sigD/sqrt(2) * (randn(L_slot, 1) + 1j*randn(L_slot, 1));

                    % Data slot (second half of block m)
                    idx_D      = 2*m;
                    ell_mono_D = mob.ell_mono_seq(idx_D);
                    ell_2du_D  = mob.ell_2DU_seq(idx_D);
                    ell_3hop_D = round((mob.dDU_seq(idx_D) + mob.dTU_seq(idx_D) + mob.dDT_seq(idx_D)) / p.c0 * p.fs);
                    
                    phT_D      = exp(1j*2*pi*mob.nu_mono_seq(idx_D) * (idx_D - 1) * p.TPRI);
                    phU_D      = exp(1j*2*pi*mob.nu_DU_seq(idx_D) * (idx_D - 1) * p.TPRI);
                    nu_3hop_D  = mob.fD_DU_seq(idx_D) + mob.fD_DT_seq(idx_D) + mob.fD_TU_seq(idx_D);
                    ph3hop_D   = exp(1j*2*pi*nu_3hop_D * (idx_D - 1) * p.TPRI);
                    
                    xD_del_T_D  = circshift(xD_tx_flat, ell_mono_D);
                    xD_del_U_D  = circshift(xD_tx_flat, ell_2du_D);
                    xD_del_3hop_D = circshift(xD_tx_flat, ell_3hop_D);

                    y_rx_D = aT_seq(idx_D) * xD_del_T_D(blk_start + L_slot : blk_end) * phT_D ...
                           + aU_seq(idx_D) * xD_del_U_D(blk_start + L_slot : blk_end) * phU_D ...
                           + a3hop_seq(idx_D) * xD_del_3hop_D(blk_start + L_slot : blk_end) * ph3hop_D ...
                           + obj.sigD/sqrt(2) * (randn(L_slot, 1) + 1j*randn(L_slot, 1));

                    yD_full_flat(blk_start:blk_end) = [y_rx_S; y_rx_D];
                end

                % Align to target delay
                yD_aligned = circshift(yD_full_flat, [-obj.ell_mono, 0]);
                yD_full_aligned_mat = reshape(yD_aligned, [L_block, p.M]);
                
                yD_sound_cp = yD_full_aligned_mat(1:p.ND + p.Ncp,     :);
                yD_data_cp  = yD_full_aligned_mat(p.ND + p.Ncp + 1:end, :);
                
                obj.yD_sound = yD_sound_cp(p.Ncp + 1:end, :);
                obj.yD_data  = yD_data_cp(p.Ncp + 1:end, :);
            end

            % Stitch the full signal back together for completeness
            obj.yD_full  = yD_full_aligned_mat;

            % Pre-compute display axes
            obj.rng_axis = (0:p.ND-1) * p.dR;
            obj.fdopp    = ((0:p.M-1) - floor(p.M/2)) / (p.M * p.Tblock);
            obj.vel_axis = obj.fdopp * p.lamD / 2;
        end

        function computeRangeDoppler(obj, p, tx)
        % Step 1: BPSK data stripping  ->  coherent integration  ->  RD map.
        %   p  : ntn.SystemParams
        %   tx : ntn.TransmitSignal

            % Build is_comm_slot and dD_all of length 2*M
            is_comm_slot = true(1, 2 * p.M);
            if isprop(p, 'sounding_config') && ~isempty(p.sounding_config)
                a = p.sounding_config(1);
                b = p.sounding_config(2);
                for m = 1:p.M
                    if mod(m-1, b-1) == 0 || mod(m-1, b-1) == a-1
                        is_comm_slot(2*m - 1) = false;
                    end
                end
            else
                for m = 1:p.M
                    if mod(m-1, p.L_sound) == 0
                        is_comm_slot(2*m - 1) = false;
                    end
                end
            end
            
            dD_all = ones(1, 2 * p.M);
            dD_all(is_comm_slot) = tx.dD;
            
            % Interleave sounding and data slots to get all 2*M slots (ND x 2M)
            yD_slots = zeros(p.ND, 2 * p.M);
            yD_slots(:, 1:2:end) = obj.yD_sound;
            yD_slots(:, 2:2:end) = obj.yD_data;
            
            % Strip modulation from all slots
            yD_slots_stripped = yD_slots .* dD_all;
            
            % Coherent integration: sum first half and second half of each block
            yD_int_raw = yD_slots_stripped(:, 1:2:end) + yD_slots_stripped(:, 2:2:end);
            
            % Shift the integrated signal forward by obj.ell_mono so that
            % the target peak appears at its true physical delay (range) in the map.
            obj.yD_int = circshift(yD_int_raw, [obj.ell_mono, 0]);

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

            fprintf('--- SIC result ---\n');
            fprintf('Dominant peak removed (UE): ell_est=%d (true %d),  nu_est=%.2f Hz (true %.2f Hz)\n', ...
                    obj.tgEst.ell, g.ell_2DU, obj.tgEst.nu, g.nu_DU);
            fprintf('Target now unmasked at: ell=%d (%.0f m),  nu_mono=%.2f Hz\n\n', ...
                    g.ell_mono, g.ell_mono*p.dR, g.nu_mono);
        end
    end
end
