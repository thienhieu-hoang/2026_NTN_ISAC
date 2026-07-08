function [yp_sic, est] = SIC(yp, s_mls, M, Tblock)
% SIC  Successive Interference Cancellation (Sec. 5.1, Step 5).
%   Detects the dominant reflector peak in the Range-Doppler map,
%   reconstructs its echo, and subtracts it from the integrated signal.
%
%   [yp_sic, est] = ntn.sensing.SIC(yp, s_mls, M, Tblock)
%
%   Inputs:
%     yp     : ND x M  coherently integrated, data-stripped signal
%     s_mls  : ND x 1  reference MLS code
%     M      : number of slow-time blocks
%     Tblock : full block duration [s]  (used to convert Doppler bin -> Hz)
%
%   Outputs:
%     yp_sic : ND x M  signal after dominant-peak subtraction
%     est    : struct with fields  .ell (delay bin), .nu (Doppler [Hz]), .A (amplitude)

    ND = size(yp, 1);

    % --- Detect dominant peak ---
    RD          = ntn.sensing.RangeDopplerMap(yp, s_mls, M);
    [~, idx]    = max(abs(RD(:)));
    [pPk, qPk]  = ind2sub(size(RD), idx);

    ell    = pPk - 1;                       % delay bin (0-based)
    qShift = qPk - 1 - floor(M/2);         % Doppler bin (fftshifted)
    nuPk   = qShift / (M * Tblock);        % Doppler frequency [Hz]

    % --- Estimate complex amplitude via Hann-weighted coherent sum ---
    w  = 0.5 - 0.5*cos(2*pi*(0:M-1)/(M-1));
    A  = RD(pPk, qPk) / (ND * sum(w));

    % --- Reconstruct data-stripped echo and subtract ---
    m      = 0:M-1;
    sPk    = circshift(s_mls, ell);
    echo   = A * (sPk * exp(1j*2*pi*nuPk*m*Tblock));   % ND x M
    yp_sic = yp - echo;

    est = struct('ell', ell, 'nu', nuPk, 'A', A);
end
