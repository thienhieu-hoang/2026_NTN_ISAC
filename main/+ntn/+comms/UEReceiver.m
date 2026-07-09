classdef UEReceiver < handle
% UERECEIVER  Sec. 4 — UE received signal construction.
%
%   Builds the full 2*ND x M received signal at the UE, comprising:
%     - Direct D->U link (desired communication signal)
%     - Scattered D->T->U interference (unknown channel)
%     - AWGN noise (reference power = |A_DU|, i.e. SNR~0 dB pre-despread)
%
%   The signal is split into sounding and data halves for downstream
%   processing by ChannelEstimator and Demodulator.
%
%   Usage:
%       ueRx = ntn.comms.UEReceiver(params, geom, channel, txSignal);

    properties
        % One-way link amplitudes at UE
        A_DU            % h_DU * sqrt(PD)           — direct path
        A_DTU           % eta_T * h_DT * h_TU * sqrt(PD)  — scattered

        % Delayed MLS replicas (used in BERAnalysis SINR cross-check)
        sD_DU           % ND x 1  direct path delayed code
        sD_DTU          % ND x 1  scattered path delayed code

        % Slow-time Doppler phase vectors (1 x M)
        ph_DU
        ph_DTU

        % Reference noise std (SNR ~ 0 dB pre-despread)
        sigU_ref

        % Received signal
        yU_full         % 2*ND x M  full received frame
        yU_sound        % ND x M    sounding half
        yU_data         % ND x M    data half
    end

    methods
        function obj = UEReceiver(p, g, ch, tx, mob, fading, sigU)
        % p  : ntn.SystemParams
        % g  : ntn.Geometry
        % ch : ntn.ChannelModel
        % tx : ntn.TransmitSignal
        % mob    : ntn.MobilityModel (optional, slot-level)
        % fading : ntn.FadingSequence (optional, slot-level)
        % sigU   : scalar (optional, noise std per dimension)

            if nargin < 5
                % --- Original Static/Quasi-Static Mode ---
                obj.A_DU  = ch.h_DU * sqrt(p.PD);
                obj.A_DTU = ch.eta_T * ch.h_DT * ch.h_TU * sqrt(p.PD);

                fprintf('--- UE link amplitudes (Static Mode) ---\n');
                fprintf('|A_DU|=%.4e  |A_DTU|=%.4e  direct/scatter ratio=%.1f dB\n\n', ...
                        abs(obj.A_DU), abs(obj.A_DTU), ...
                        20*log10(abs(obj.A_DU) / max(abs(obj.A_DTU), 1e-30)));

                obj.sD_DU  = circshift(p.s_mls, g.ell_DU);
                obj.sD_DTU = circshift(p.s_mls, g.ell_DTU);

                obj.ph_DU  = exp(1j*2*pi*g.fD_DU  * tx.m_idx * p.Tblock);
                obj.ph_DTU = exp(1j*2*pi*g.nu_DTU  * tx.m_idx * p.Tblock);

                % Reference noise amplitude (SNR ~ 0 dB pre-despread)
                obj.sigU_ref = abs(obj.A_DU);

                % Flatten transmit signal matrix to a 1D vector (continuous stream)
                xD_tx_flat = tx.xD_tx(:);

                % Replicate slow-time Doppler phase vectors over the chips for the sequence
                ph_DU_chips = kron(obj.ph_DU, ones(2*(p.ND + p.Ncp), 1));
                ph_DU_chips_flat = ph_DU_chips(:);
                
                ph_DTU_chips = kron(obj.ph_DTU, ones(2*(p.ND + p.Ncp), 1));
                ph_DTU_chips_flat = ph_DTU_chips(:);

                % Generate received signal
                L_block = 2*(p.ND + p.Ncp);
                yU_full_flat = ...
                      obj.A_DU  * circshift(xD_tx_flat, [g.ell_DU,  0]) .* ph_DU_chips_flat  ...
                    + obj.A_DTU * circshift(xD_tx_flat, [g.ell_DTU, 0]) .* ph_DTU_chips_flat ...
                    + obj.sigU_ref/sqrt(2) * (randn(L_block*p.M, 1) + 1j*randn(L_block*p.M, 1));

                % --- UE Receiver Processing (Frame Synchronization & CP Discard) ---
                yU_aligned = circshift(yU_full_flat, [-g.ell_DU, 0]);
                yU_full_aligned_mat = reshape(yU_aligned, [L_block, p.M]);
                
                yU_sound_cp = yU_full_aligned_mat(1 : p.ND + p.Ncp, :);
                yU_data_cp  = yU_full_aligned_mat(p.ND + p.Ncp + 1 : end, :);
                
                obj.yU_sound = yU_sound_cp(p.Ncp + 1 : end, :);
                obj.yU_data  = yU_data_cp(p.Ncp + 1 : end, :);
                obj.yU_full  = [obj.yU_sound; obj.yU_data];
            else
                % --- Dynamic Time-Varying (Slot-Level) Mode ---
                obj.sigU_ref = sigU;
                
                % Reference properties for compatibility
                obj.A_DU  = ch.h_DU * sqrt(p.PD);
                obj.A_DTU = ch.eta_T * ch.h_DT * ch.h_TU * sqrt(p.PD);
                obj.sD_DU  = circshift(p.s_mls, g.ell_DU);
                obj.sD_DTU = circshift(p.s_mls, g.ell_DTU);

                C0          = (p.lamD / (4*pi))^2;
                beta_DU_seq = C0 * mob.dDU_seq .^ (-2);
                A_DU_seq    = sqrt(beta_DU_seq) .* fading.g_DU * sqrt(p.PD);

                %% Uncomment this if need to plot the channel over time
                % M_seq = length(fading.g_DU);
                % block_axis = 1:M_seq;
                % 
                % figure('Name', 'Channel Evolution Over Blocks', 'Position', [100, 100, 800, 600]);
                % 
                % % --- Top Subplot: Small-Scale Fading |g| ---
                % subplot(2, 1, 1);
                % plot(block_axis, abs(fading.g_DU), 'b-', 'LineWidth', 1.5);
                % grid on;
                % ylabel('|g_{DU}[m]| (Rayleigh Fading)');
                % xlabel('Half-Block Index (m*2)');
                % title('Small-Scale Rayleigh Fading Magnitude Over Blocks');
                % 
                % % --- Bottom Subplot: Full Channel Amplitude |h_DU_m| ---
                % subplot(2, 1, 2);
                % plot(block_axis, abs(A_DU_seq)./sqrt(p.PD), 'r-', 'LineWidth', 1.5);
                % grid on;
                % ylabel('|h_{DU}[m]| (Path Loss + Fading)');
                % xlabel('Half-Block Index (m*2)');
                % title('Full Desired Channel Amplitude Over Blocks');  
                %%

                beta_DT_seq = C0 * mob.dDT_seq .^ (-2);
                beta_TU_seq = C0 * mob.dTU_seq .^ (-2);
                A_DTU_seq   = abs(ch.eta_T) * sqrt(beta_DT_seq .* beta_TU_seq) .* fading.g_DT .* fading.g_TU * sqrt(p.PD);

                L_slot = p.ND + p.Ncp;
                L_block = 2 * L_slot;
                yU_full_flat = zeros(L_block * p.M, 1);
                xD_tx_flat = tx.xD_tx(:);

                for m = 1:p.M
                    blk_start = (m-1)*L_block + 1;
                    blk_end   = m*L_block;

                    % Sounding slot (first half of block m)
                    idx_S     = 2*m - 1;
                    ell_du_S  = mob.ell_DU_seq(idx_S);
                    ell_dtu_S = mob.ell_DTU_seq(idx_S);
                    ph_DU_S   = exp(1j*2*pi*mob.fD_DU_seq(idx_S) * (idx_S - 1) * p.TPRI);
                    ph_DTU_S  = exp(1j*2*pi*mob.nu_DTU_seq(idx_S) * (idx_S - 1) * p.TPRI);
                    
                    xD_del_DU_S  = circshift(xD_tx_flat, ell_du_S);
                    xD_del_DTU_S = circshift(xD_tx_flat, ell_dtu_S);
                    
                    if m == 1
                        xD_del_DU_S(1 : ell_du_S) = 0;
                        xD_del_DTU_S(1 : ell_dtu_S) = 0;
                    end

                    y_rx_S = A_DU_seq(idx_S)  * xD_del_DU_S(blk_start : blk_start + L_slot - 1)  * ph_DU_S ...
                           + A_DTU_seq(idx_S) * xD_del_DTU_S(blk_start : blk_start + L_slot - 1) * ph_DTU_S ...
                           + sigU * (randn(L_slot, 1) + 1j*randn(L_slot, 1));

                    % Data slot (second half of block m)
                    idx_D     = 2*m;
                    ell_du_D  = mob.ell_DU_seq(idx_D);
                    ell_dtu_D = mob.ell_DTU_seq(idx_D);
                    ph_DU_D   = exp(1j*2*pi*mob.fD_DU_seq(idx_D) * (idx_D - 1) * p.TPRI);
                    ph_DTU_D  = exp(1j*2*pi*mob.nu_DTU_seq(idx_D) * (idx_D - 1) * p.TPRI);
                    
                    xD_del_DU_D  = circshift(xD_tx_flat, ell_du_D);
                    xD_del_DTU_D = circshift(xD_tx_flat, ell_dtu_D);

                    y_rx_D = A_DU_seq(idx_D)  * xD_del_DU_D(blk_start + L_slot : blk_end)  * ph_DU_D ...
                           + A_DTU_seq(idx_D) * xD_del_DTU_D(blk_start + L_slot : blk_end) * ph_DTU_D ...
                           + sigU * (randn(L_slot, 1) + 1j*randn(L_slot, 1));

                    yU_full_flat(blk_start:blk_end) = [y_rx_S; y_rx_D];
                end

                % Sync using block 1 sounding delay
                ell_sync = mob.ell_DU_seq(1);
                yU_aligned = circshift(yU_full_flat, [-ell_sync, 0]);
                yU_full_aligned_mat = reshape(yU_aligned, [L_block, p.M]);
                
                yU_sound_cp = yU_full_aligned_mat(1 : p.ND + p.Ncp, :);
                yU_data_cp  = yU_full_aligned_mat(p.ND + p.Ncp + 1 : end, :);
                
                obj.yU_sound = yU_sound_cp(p.Ncp + 1 : end, :);
                obj.yU_data  = yU_data_cp(p.Ncp + 1 : end, :);
                obj.yU_full  = [obj.yU_sound; obj.yU_data];
            end
        end
    end
end
