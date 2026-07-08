function plotBER(ber, savePath)
% PLOTBER  Figure 2 — UE downlink BER vs Eb/N0.
%
%   Plots three curves on the same semilog axes:
%     k-   : Theory — BPSK AWGN with perfect CSI
%     r--  : Theory — Pilot-assisted BPSK via MGF derivation (Sec. 4.2)
%     bo-  : Simulation — sounding ChEst -> despread -> equalise -> decide
%
%   plots.plotBER(ber, savePath)
%
%   Inputs:
%     ber      : ntn.comms.BERAnalysis  (must have run runSweep first)
%     savePath : char  full path for the output PNG file

    figure('Name','UE BER','Position',[150 150 720 540]);

    semilogy(ber.EbN0_dB, ber.BER_theory,     'k-',  'LineWidth', 1.5);
    hold on; grid on;
    semilogy(ber.EbN0_dB, ber.BER_theory_est, 'r--', 'LineWidth', 1.5);
    
    % % Simulated BER for synchronized (clean) blocks 2..M
    % semilogy(ber.EbN0_dB, max(ber.BER_sim, 1e-7), ...
    %         'mx-', 'LineWidth', 1.2, 'MarkerFaceColor', 'm');
    % 
    % % Simulated BER for the unsynchronized (corrupted) block 1
    % semilogy(ber.EbN0_dB, max(ber.BER_block1, 1e-7), ...
    %          'gs-', 'LineWidth', 1.2, 'MarkerFaceColor', 'g');

    % Simulated BER for the entire block sequence (average)
    semilogy(ber.EbN0_dB, ber.BER_all, ...
             'bo-', 'LineWidth', 1.2, 'MarkerFaceColor', 'b');

    xlabel('E_b/N_0 [dB]');
    ylabel('BER');
    ylim([1e-6 1]);
    title('UE downlink BER — continuous sequence PMCW');
    legend('Theory (BPSK AWGN — Perfect CSI)', ...
           'Theory (Pilot ChEst — Noisy Pilot)', ...
           ['Sim: All Blocks (1 to ', num2str(ber.M_seq), '; Average)'], ...
           'Location', 'southwest');
           % ['Sim: Clean Blocks (2 to ', num2str(ber.M_seq), ')'], ...
           % 'Sim: Corrupted Block 1 (Sync Transient)', ...
           % ['Sim: All Blocks (1 to ', num2str(ber.M_seq), '; Average)'], ...
           % 'Location', 'southwest');

    saveas(gcf, savePath);
    fprintf('Saved: %s\n', savePath);
end
