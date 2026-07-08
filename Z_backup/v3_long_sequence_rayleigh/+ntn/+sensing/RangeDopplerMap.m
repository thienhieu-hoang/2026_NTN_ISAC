function RD = RangeDopplerMap(yp, s_mls, M)
% RANGEDOPPLERMAP  Range-Doppler map via circular matched filter +
%   Hann-windowed slow-time DFT.
%
%   RD = ntn.sensing.RangeDopplerMap(yp, s_mls, M)
%
%   Inputs:
%     yp    : ND x M  data-stripped received signal
%     s_mls : ND x 1  reference MLS code
%     M     : number of slow-time blocks (columns of yp)
%
%   Output:
%     RD    : ND x M  Range-Doppler map (fftshifted along Doppler axis)

    Sf = fft(s_mls);
    R  = ifft(fft(yp, [], 1) .* conj(Sf), [], 1);     % range compression
    w  = 0.5 - 0.5*cos(2*pi*(0:M-1)/(M-1));           % Hann window (slow-time)
    RD = fftshift(fft(R .* w, [], 2), 2);              % Doppler FFT + fftshift
end
