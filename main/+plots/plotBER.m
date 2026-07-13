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
%     savePath : char  full path for the output PDF file

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
    
    if strcmpi(ber.model_type, 'static')
        theory_legend1 = 'Theory (BPSK AWGN — Perfect CSI)';
        theory_legend2 = 'Theory (Pilot ChEst — Noisy Pilot, AWGN)';
    else
        theory_legend1 = 'Theory (BPSK Rayleigh — Perfect CSI)';
        theory_legend2 = 'Theory (Pilot ChEst — Noisy Pilot, Rayleigh)';
    end
    
    legend(theory_legend1, ...
           theory_legend2, ...
           ['Sim: All Blocks (1 to ', num2str(ber.M_seq), '; Average)'], ...
           'Location', 'southwest');
           % ['Sim: Clean Blocks (2 to ', num2str(ber.M_seq), ')'], ...
           % 'Sim: Corrupted Block 1 (Sync Transient)', ...
           % ['Sim: All Blocks (1 to ', num2str(ber.M_seq), '; Average)'], ...
           % 'Location', 'southwest');

    if endsWith(savePath, '.pdf', 'IgnoreCase', true)
        fig = gcf;
        fig.Units = 'inches';
        fig.PaperUnits = 'inches';
        pos = fig.Position;
        fig.PaperSize = [pos(3), pos(4)];
        fig.PaperPosition = [0, 0, pos(3), pos(4)];
        print(fig, savePath, '-dpdf', '-painters', '-r0');
    else
        saveas(gcf, savePath);
    end
    fprintf('Saved: %s\n', savePath);
end
