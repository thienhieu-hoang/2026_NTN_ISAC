%% ================================================================
%  NTN-ISAC Three-Node System -- Single-Code PMCW Simulation
%  Physical channel model (system_model_3node.md, Sec. 2):
%    h_ij(t) = sqrt(beta_ij) * g_ij * exp(j2pi f_{D,ij} t)
%    g_ij ~ CN(0,1)  -- Rayleigh per-path fading (one path per link)
%
%  Frame structure (Sec. 3):  T_block = 2*T_PRI
%    | sounding (T_PRI) | data (T_PRI) |   per block m
%
%  Nodes:
%    Drone (D) : PMCW Tx + full-duplex monostatic Rx  -> SIC + RD sensing
%    UE    (U) : Rx only -> sounding ChEst + despread + equalize + BPSK
%    Target(T) : passive scatterer (unknown to UE/Drone a priori)
%
%  Figures:
%    1 : Range-Doppler heatmaps at Drone (before / after SIC)
%    2 : UE BER vs Eb/N0
%
%  No toolboxes required (base MATLAB / Octave compatible).
%% ================================================================
clear; close all; clc;
rng(2024);

%% ---- 1. System & waveform parameters ----
c0     = 3e8;           % speed of light [m/s]
fcD    = 24e9;          % drone carrier frequency [Hz]
lamD   = c0 / fcD;      % wavelength [m]
B      = 50e6;          % chip rate / bandwidth [Hz]
Tc     = 1 / B;         % chip duration [s]
fs     = B;             % sampling rate = chip rate [Hz]
mLFSR  = 7;             % MLS order -> N_D = 2^m - 1
ND     = 2^mLFSR - 1;  % code length = 127 chips
M      = 256;           % number of slow-time blocks
TPRI   = ND * Tc;       % one code period = half-block [s]
Tblock = 2 * TPRI;      % full block: [sounding | data], Sec. 3 [s]
PD     = 1;             % drone Tx power (normalized)

s_mls  = gen_mls(mLFSR);   % ND x 1, polar MLS in {+1,-1}

% Radar performance limits  (slow-time spacing = Tblock)
dR    = c0 / (2*fs);
Rmax  = c0 * ND * Tc / 2;
dv    = lamD / (2 * M * Tblock);    % velocity resolution [m/s]
vmax  = lamD / (4 * Tblock);        % max unambiguous velocity [m/s]

fprintf('=== NTN-ISAC 3-node PMCW simulation (with channel model) ===\n');
fprintf('N_D=%d  M=%d  T_PRI=%.2f us  T_block=%.2f us\n', ND, M, TPRI*1e6, Tblock*1e6);
fprintf('dR=%.2f m  Rmax=%.0f m  dv=%.4f m/s  vmax=%.2f m/s\n\n', dR, Rmax, dv, vmax);

%% ---- 2. Geometry (positions & velocities) ----
pD = [  0,  0, 100];   vD = [ 15,  0,  0];   % drone   [m], [m/s]
pU = [180,  0,   0];   vU = [  0,  8,  0];   % UE
pT = [120, 40,  90];   vT = [-25, 12,  3];   % moving target

dDU = norm(pU - pD);
dDT = norm(pT - pD);
dTU = norm(pU - pT);

% Integer chip delay bins (Sec. 1)
ell_mono = round(2*dDT / c0 * fs);        % D->T->D  round-trip (monostatic)
ell_2DU  = round(2*dDU / c0 * fs);        % D->U->D  round-trip (UE echo at drone)
ell_DU   = round(  dDU / c0 * fs);        % D->U     one-way   (comm path)
ell_DTU  = round((dDT + dTU) / c0 * fs);  % D->T->U  two-hop   (scattered)

% LOS unit vectors
uDU = (pU - pD) / dDU;
uDT = (pT - pD) / dDT;
uTU = (pU - pT) / dTU;

% Radial velocities -> Doppler shifts [Hz]  (Sec. 1)
fD_DU = -dot(vU - vD, uDU) / lamD;
fD_DT = -dot(vT - vD, uDT) / lamD;
fD_TU = -dot(vU - vT, uTU) / lamD;

nu_mono = 2 * fD_DT;          % target two-way Doppler [Hz]  (drone Rx)
nu_DU   = 2 * fD_DU;          % UE two-way Doppler [Hz]      (drone Rx)
nu_DTU  = fD_DT + fD_TU;      % scattered path Doppler at UE [Hz]

fprintf('--- Geometry ---\n');
fprintf('dDT=%.1f m  dDU=%.1f m  dTU=%.1f m\n', dDT, dDU, dTU);
fprintf('Target : range bin %d (%.0f m),  v_r=%.2f m/s,  nu_mono=%.2f Hz\n', ...
        ell_mono, ell_mono*dR, nu_mono*lamD/2, nu_mono);
fprintf('UE     : range bin %d (%.0f m),  v_r=%.2f m/s,  nu_DU=%.2f Hz\n\n', ...
        ell_2DU, ell_2DU*dR, nu_DU*lamD/2, nu_DU);

%% ---- 3. Channel model  (system_model_3node.md, Sec. 2) ----
%
%  Per-path channel:  h_ij = sqrt(beta_ij) * g_ij
%    beta_ij = C0 * (d_ij/d0)^{-alpha}   [large-scale path gain]
%    g_ij ~ CN(0,1)                       [Rayleigh small-scale fading]
%  Time-varying Doppler rotation exp(j2pi f_{D,ij} t) applied as
%  slow-time phase below (block-fading: h_ij constant within one block).
%
d0    = 1;      % reference distance [m]
alpha = 2.0;    % free-space path-loss exponent (NTN LOS)
% Reference gain C0 = GT*GR*(lambda/(4*pi*d0))^2, GT=GR=1 (isotropic)
C0    = (lamD / (4*pi*d0))^2;

beta_DT = C0 * (dDT/d0)^(-alpha);
beta_DU = C0 * (dDU/d0)^(-alpha);
beta_TU = C0 * (dTU/d0)^(-alpha);

% Per-path Rayleigh small-scale fading: g_ij ~ CN(0,1)
g_DT = (randn + 1j*randn) / sqrt(2);
g_DU = (randn + 1j*randn) / sqrt(2);
g_TU = (randn + 1j*randn) / sqrt(2);

% Channel amplitudes (quasi-static across all M blocks)
h_DT = sqrt(beta_DT) * g_DT;   % D->T
h_DU = sqrt(beta_DU) * g_DU;   % D->U
h_TU = sqrt(beta_TU) * g_TU;   % T->U

% Complex reflection coefficients
eta_T = 1e-3 * exp(1j*2*pi*rand);   % passive target (small UAV RCS)
eta_U = 0.30 * exp(1j*2*pi*rand);   % UE surface reflection

fprintf('--- Channel (Sec. 2) ---\n');
fprintf('beta_DT=%.3e  beta_DU=%.3e  beta_TU=%.3e\n', beta_DT, beta_DU, beta_TU);
fprintf('|h_DT|=%.4e   |h_DU|=%.4e   |h_TU|=%.4e\n', abs(h_DT), abs(h_DU), abs(h_TU));
fprintf('|eta_T|=%.4f   |eta_U|=%.4f\n\n', abs(eta_T), abs(eta_U));

%% ---- 4. BPSK payload bits ----
bD    = randi([0 1], 1, M);   % one bit per block
dD    = 1 - 2*bD;             % polar BPSK symbols in {+1,-1}
m_idx = 0:M-1;                % block index vector

%% ---- 5. Drone Rx: received signal -- FULL two-half-block frame (Sec. 5) ----
%
%  The drone transmits a full T_block = 2*T_PRI frame per slow-time block m:
%    Half 1 (sounding)  : pure MLS, d_m = +1  -> drone Rx: echo of sounding
%    Half 2 (data)      : MLS * d_m           -> drone Rx: echo of data
%
%  Two-way echo amplitudes (back-scatter):
%    a_i = eta_i * beta_Di * g_Di^2 * sqrt(PD)
%  Derivation: D->i: h_Di = sqrt(beta_Di)*g_Di;
%              i->D: by reciprocity same channel -> product = beta_Di*g_Di^2
%
aT = eta_T * beta_DT * g_DT^2 * sqrt(PD);   % target echo amplitude
aU = eta_U * beta_DU * g_DU^2 * sqrt(PD);   % UE echo amplitude

% Drone noise variance: set so single-period target SNR = SNR_target_dB
% (after coherent integration of both halves SNR improves by +3 dB)
SNR_target_dB = 18;
sigD = abs(aT) * sqrt(ND*M) / sqrt(10^(SNR_target_dB/10));

fprintf('--- Drone Rx amplitudes (Sec. 5) ---\n');
fprintf('|aT|=%.4e  |aU|=%.4e  UE/target ratio=%.1f dB\n', ...
        abs(aT), abs(aU), 20*log10(abs(aU)/abs(aT)));
fprintf('sigD=%.4e  (single-period target SNR = %d dB; after integration = %.0f dB)\n\n', ...
        sigD, SNR_target_dB, SNR_target_dB + 10*log10(2));

% Slow-time Doppler phases (one per block, reference interval = Tblock)
phT = exp(1j*2*pi*nu_mono*m_idx*Tblock);   % 1 x M
phU = exp(1j*2*pi*nu_DU  *m_idx*Tblock);   % 1 x M

% --- Transmit signal matrix (2*ND x M) ---
% Concatenated sounding (first ND) and data (last ND) periods
xD_tx = [s_mls * ones(1, M); s_mls * dD];   % 2ND x M

% Drone Rx: received signal of full 2ND length per block
yD_full = aT * circshift(xD_tx, [ell_mono, 0]) .* phT ...   % 2ND x M
        + aU * circshift(xD_tx, [ell_2DU,  0]) .* phU ...   % 2ND x M
        + sigD/sqrt(2) * (randn(2*ND,M) + 1j*randn(2*ND,M));

% Separate the sounding and data halves at the receiver
yD_sound = yD_full(1:ND, :);
yD_data  = yD_full(ND+1:end, :);

%% ---- 6. Drone sensing pipeline -- coherent sounding+data integration (Sec. 5) ----
%
%  Step 1: BPSK data stripping on the data half
%    Since d_m^2 = 1, multiplying by the known d_m removes the BPSK modulation:
%    y'_D_data[n,m] = y_D_data[n,m] * d_m
%                   = aT*s[(n-ell_mono)]*exp(j2pi*nu_mono*m*Tblock) + d_m*noise
%
ypD_data = yD_data .* dD;    % ND x M  -- data stripped

%  Step 2: Coherent integration of sounding + stripped-data halves
%    Both halves now carry the same unmodulated echo structure, so they add coherently:
%      Signal amplitude : aT + aT = 2*aT  => Signal power x4 (+6 dB)
%      Noise variance   : sigma^2 + sigma^2 = 2*sigma^2  => Noise power x2 (+3 dB)
%      Net SNR gain     : +3 dB vs. using only one half  (PMCW pipeline Sec. 5, Step 2)
%
yD_int = yD_sound + ypD_data;    % ND x M  -- integrated (2x amplitude)

fprintf('--- Drone Rx: coherent integration (sounding + data halves) ---\n');
fprintf('Signal amplitude: x2  =>  +6 dB signal power\n');
fprintf('Noise power    : x2  =>  +3 dB noise power\n');
fprintf('Net SNR gain   : +3 dB vs. single-half processing\n\n');

% Steps 3-4: Matched filter (FFT circular corr) -> Range-Doppler map
%   Input is the coherently integrated signal yD_int
RD_before = rangeDopplerMap(yD_int, s_mls, M);

% Step 5: SIC -- detect dominant peak, reconstruct & subtract (on integrated signal)
[yD_int_sic, tgEst] = sic_remove_strongest(yD_int, s_mls, M, Tblock);
RD_after = rangeDopplerMap(yD_int_sic, s_mls, M);

% Display axes
rng_axis = (0:ND-1) * dR;
fdopp    = ((0:M-1) - floor(M/2)) / (M*Tblock);
vel_axis = fdopp * lamD/2;

fprintf('--- SIC result ---\n');
fprintf('Dominant peak removed (Target): ell_est=%d (true %d),  nu_est=%.2f Hz (true %.2f Hz)\n', ...
        tgEst.ell, ell_2DU, tgEst.nu, nu_DU);
fprintf('UE now unmasked at   : ell=%d (%.0f m),  nu_mono=%.2f Hz\n\n', ...
        ell_mono, ell_mono*dR, nu_mono);

%% ---- 7. Plot Range-Doppler heatmaps  (Figure 1) ----
figure('Name','Range-Doppler Map','Position',[100 100 1100 440]);

subplot(1,2,1);
imagesc(vel_axis, rng_axis, 20*log10(abs(RD_before)/max(abs(RD_before(:)))));
set(gca,'YDir','normal'); axis tight; caxis([-50 0]); colorbar;
xlabel('Velocity [m/s]'); ylabel('Range [m]');
title('Joint RD map (before SIC)'); colormap(jet);
hold on;
plot(nu_DU  *lamD/2, ell_2DU *dR, 'wo','MarkerSize',12,'LineWidth',1.5);
plot(nu_mono*lamD/2, ell_mono*dR, 'w^','MarkerSize',12,'LineWidth',1.5);
legend('UE (strong)','Target (weak)','TextColor','w','Location','northeast');

subplot(1,2,2);
imagesc(vel_axis, rng_axis, 20*log10(abs(RD_after)/max(abs(RD_before(:)))));
set(gca,'YDir','normal'); axis tight; caxis([-50 0]); colorbar;
xlabel('Velocity [m/s]'); ylabel('Range [m]');
title('After SIC (Target removed -> UE unmasked)'); colormap(jet);
hold on;
plot(nu_mono*lamD/2, ell_mono*dR, 'w^','MarkerSize',12,'LineWidth',1.5);
legend('Target','TextColor','w','Location','northeast');
saveas(gcf, 'C:\Users\AT30890\polymtl\Tri Nhu Do - NTN_ISAC\range_doppler.png');

%% ---- 8. UE received signal: two-period frame  (Sec. 3 & 4) ----
%
%  One-way received amplitudes at UE:
%    A_DU  = h_DU * sqrt(PD)              -- direct D->U link
%    A_DTU = eta_T * h_DT * h_TU * sqrt(PD) -- scattered D->T->U (unknown)
%
A_DU  = h_DU * sqrt(PD);
A_DTU = eta_T * h_DT * h_TU * sqrt(PD);

fprintf('--- UE link amplitudes ---\n');
fprintf('|A_DU|=%.4e  |A_DTU|=%.4e  direct/scatter ratio=%.1f dB\n\n', ...
        abs(A_DU), abs(A_DTU), 20*log10(abs(A_DU)/max(abs(A_DTU),1e-30)));

% Delayed MLS replicas at UE
sD_DU  = circshift(s_mls, ell_DU);    % direct path delay
sD_DTU = circshift(s_mls, ell_DTU);   % scattered path delay

% Slow-time Doppler phases at UE (reference Tblock)
ph_DU  = exp(1j*2*pi*fD_DU *m_idx*Tblock);   % 1 x M
ph_DTU = exp(1j*2*pi*nu_DTU*m_idx*Tblock);   % 1 x M

% Reference noise for Section 9 display only (SNR ~ 0 dB pre-despread)
sigU_ref = abs(A_DU);

% --- Transmit signal matrix towards UE (2*ND x M) ---
% Concatenated sounding (first ND) and data (last ND) periods
xU_tx = [s_mls * ones(1, M); s_mls * dD];   % 2ND x M

% UE received signal of full 2ND length per block
yU_full = A_DU  * circshift(xU_tx, [ell_DU,  0]) .* ph_DU ...   % 2ND x M
        + A_DTU * circshift(xU_tx, [ell_DTU, 0]) .* ph_DTU ...  % 2ND x M
        + sigU_ref/sqrt(2) * (randn(2*ND,M) + 1j*randn(2*ND,M));

% Separate the sounding and data halves at the UE receiver
yU_sound = yU_full(1:ND, :);
yU_data  = yU_full(ND+1:end, :);

%% ---- 9. Channel estimation from sounding  (Sec. 4.1, Step 1) ----
%
%  Correlate sounding y_U_sound with s_mls (via FFT circular cross-corr).
%  The UE estimates the propagation delay by locating the correlation peak.
%  At the estimated delay bin ell_DU_est, the output equals:
%    R_sound[ell_DU_est, m] = A_DU * exp(j2pi*fD_DU*m*Tblock) * N_D  + noise
%  Dividing by N_D gives the per-block channel estimate.
%
Sf      = fft(s_mls);                                       % ND x 1, pilot DFT
R_sound = ifft(fft(yU_sound,[],1) .* conj(Sf), [], 1);     % ND x M, corr output

% Estimate direct-link delay at UE via correlation peak detection
[~, peak_idx_U] = max(mean(abs(R_sound), 2));
ell_DU_est = peak_idx_U - 1;

h_hat   = R_sound(ell_DU_est+1, :) / ND;   % 1 x M  per-block ChEst
%          h_hat(:,m) ~ A_DU * exp(j2pi*fD_DU*m*Tblock) + O(sigU/sqrt(ND))

fprintf('--- Channel estimation at UE (Sec. 4.1, Step 1) ---\n');
fprintf('Estimated UE delay: %d chips (True: %d)\n', ell_DU_est, ell_DU);
fprintf('True |A_DU|=%.4e   Mean|h_hat|=%.4e\n\n', abs(A_DU), mean(abs(h_hat)));

%% ---- 10. UE data demodulation  (Sec. 4.1, Steps 2-6) ----

% Step 2-3: Matched filter on data period -> despread peak at delay ell_DU_est
%   z_mf[m] = sum_n y_data[n,m]*s[(n-ell_DU_est)]
%           ~ A_DU * exp(j2pi*fD_DU*m*Tblock) * N_D * d_m  + sidelobe + noise
R_data = ifft(fft(yU_data,[],1) .* conj(Sf), [], 1);   % ND x M
z_mf   = R_data(ell_DU_est+1, :);   % 1 x M  (peak at estimated delay bin)

% Step 4: Equalization -- divide by per-block channel estimate
%   Cancels both h_DU and slow-time Doppler exp(j2pi*fD_DU*m*Tblock)
%   z_eq[m] ~ N_D * d_m  + equalized noise
z_eq = z_mf ./ h_hat;    % 1 x M

% Steps 5-6: BPSK decision on real part of equalized peak
d_hat = sign(real(z_eq));          % {+1,-1}
b_hat = (1 - d_hat) / 2;           % {0,1}
BER_ref = mean(b_hat ~= bD);
fprintf('--- BER at reference noise (SNR~0 dB pre-despread) ---\n');
fprintf('BER = %.4f\n\n', BER_ref);

%% ---- 11. UE BER vs Eb/N0 sweep  (Sec. 4.1 SINR formula) ----
%
%  Eb/N0 definition at UE:
%    Eb/N0 = |A_DU|^2 * N_D / sigma_U^2
%  => sigma_U^2 = |A_DU|^2 * N_D / Eb/N0   (complex noise variance per chip)
%
EbN0_dB  = 0:2:20;
Nblocks  = 5000;   % Monte Carlo BPSK symbols per Eb/N0 point

BER_sim        = zeros(size(EbN0_dB));
BER_theory     = 0.5 * erfc(sqrt(10.^(EbN0_dB/10)));   % BPSK AWGN reference (Perfect CSI)
BER_theory_est = 0.5 * exp(-10.^(EbN0_dB/10));         % BPSK with Pilot ChEst (Analytical)

% Pre-compute slow-time phases for Nblocks
mm_blk   = 0:Nblocks-1;
ph_DU_n  = exp(1j*2*pi*fD_DU *mm_blk*Tblock);   % 1 x Nblocks
ph_DTU_n = exp(1j*2*pi*nu_DTU*mm_blk*Tblock);   % 1 x Nblocks
A_DU_mag = abs(A_DU);

for ki = 1:numel(EbN0_dB)
    EbN0 = 10^(EbN0_dB(ki)/10);
    % Per-dimension noise std: sigma_U^2 = A_DU_mag^2 * ND / EbN0
    sigU = A_DU_mag * sqrt(ND / (2*EbN0));

    b_sim = randi([0 1], 1, Nblocks);
    d_sim = 1 - 2*b_sim;

    % Combined transmit signal for simulation (2*ND x Nblocks)
    x_tx_sim = [s_mls * ones(1, Nblocks); s_mls * d_sim];

    % Full received signal at UE
    y_full_sim = A_DU  * circshift(x_tx_sim, [ell_DU,  0]) .* ph_DU_n ...
               + A_DTU * circshift(x_tx_sim, [ell_DTU, 0]) .* ph_DTU_n ...
               + sigU  * (randn(2*ND,Nblocks) + 1j*randn(2*ND,Nblocks));

    % Separate sounding and data halves at the receiver
    yS = y_full_sim(1:ND, :);
    yP = y_full_sim(ND+1:end, :);

    % Step 1: Channel estimation from sounding
    Rs = ifft(fft(yS,[],1) .* conj(Sf), [], 1);
    
    % Estimate delay per simulation trial (crucial at high noise / low Eb/N0)
    [~, peak_idx_sim] = max(mean(abs(Rs), 2));
    ell_DU_est_sim = peak_idx_sim - 1;
    
    hh = Rs(ell_DU_est_sim+1, :) / ND;                    % 1 x Nblocks

    % Steps 2-4: Despread data + equalize
    Rd = ifft(fft(yP,[],1) .* conj(Sf), [], 1);
    zd = Rd(ell_DU_est_sim+1, :) ./ hh;                   % 1 x Nblocks

    % Steps 5-6: BPSK decision
    BER_sim(ki) = mean(sign(real(zd)) ~= d_sim);
end

% Analytical SINR cross-check at highest Eb/N0 (Sec. 4.1 SINR formula)
rho_norm  = (sD_DTU.' * sD_DU) / ND;   % MLS cross-corr (code suppression)
EbN0_last = 10^(EbN0_dB(end)/10);
SINR_supp = A_DU_mag^2 / ((abs(A_DTU)*abs(rho_norm))^2 + A_DU_mag^2/EbN0_last);
fprintf('--- Analytical SINR cross-check (Sec. 4.1) ---\n');
fprintf('rho(ell_DTU)=%.5f  (1/N_D=%.5f)\n', rho_norm, 1/ND);
fprintf('At Eb/N0=%d dB: SINR_U(scatter suppressed)=%.1f dB\n\n', ...
        EbN0_dB(end), 10*log10(SINR_supp));

%% ---- 12. Plot BER  (Figure 2) ----
figure('Name','UE BER','Position',[150 150 680 500]);
semilogy(EbN0_dB, BER_theory, 'k-',  'LineWidth',1.5); hold on; grid on;
semilogy(EbN0_dB, BER_theory_est, 'r--', 'LineWidth',1.5);
semilogy(EbN0_dB, max(BER_sim,1e-7), 'bo-','LineWidth',1.2,'MarkerFaceColor','b');
xlabel('E_b/N_0 [dB]'); ylabel('BER'); ylim([1e-6 1]);
title('UE downlink BER -- physical channel (single-code PMCW)');
legend('Theory (BPSK AWGN - Perfect CSI)', ...
       'Theory (BPSK - Pilot ChEst - MGF Derivation)', ...
       'Sim: sounding ChEst \rightarrow despread \rightarrow equalize', ...
       'Location','southwest');
saveas(gcf, 'C:\Users\AT30890\polymtl\Tri Nhu Do - NTN_ISAC\ue_ber.png');

%% ================================================================
%  Local functions
%% ================================================================

function s = gen_mls(m)
% Maximal-length sequence (LFSR), binary {0,1} -> polar {+1,-1}.
    switch m
        case 7,  taps = [7 6];
        case 8,  taps = [8 6 5 4];
        case 9,  taps = [9 5];
        case 10, taps = [10 7];
        otherwise, error('Add primitive-polynomial taps for m=%d', m);
    end
    N = 2^m - 1;  reg = ones(1,m);  c = zeros(N,1);
    for i = 1:N
        c(i) = reg(end);
        fb   = mod(sum(reg(taps)), 2);
        reg  = [fb reg(1:end-1)];
    end
    s = 1 - 2*c;   % {0,1} -> {+1,-1}
end

function RD = rangeDopplerMap(yp, s_mls, M)
% Single matched filter (circular corr via FFT) + windowed slow-time DFT.
%   yp   : ND x M, data-stripped received signal
%   s_mls: ND x 1, reference MLS code
    ND = size(yp, 1);
    Sf = fft(s_mls);
    R  = ifft(fft(yp,[],1) .* conj(Sf), [], 1);    % range compression
    w  = 0.5 - 0.5*cos(2*pi*(0:M-1)/(M-1));        % Hann window (slow time)
    RD = fftshift(fft(R .* w, [], 2), 2);           % Doppler FFT
end

function [yp_sic, est] = sic_remove_strongest(yp, s_mls, M, Tblock)
% Detect dominant reflector peak, reconstruct its echo, subtract it (Sec. 5.1, Step 5).
%   yp    : ND x M, data-stripped signal
%   Tblock: full block duration [s] (used for Doppler frequency axis)
    ND = size(yp, 1);
    RD = rangeDopplerMap(yp, s_mls, M);
    [~, idx]   = max(abs(RD(:)));
    [pPk, qPk] = ind2sub(size(RD), idx);

    ell    = pPk - 1;                         % delay bin (0-based)
    qShift = qPk - 1 - floor(M/2);           % Doppler index (fftshifted)
    nuPk   = qShift / (M * Tblock);          % estimated Doppler [Hz]

    w  = 0.5 - 0.5*cos(2*pi*(0:M-1)/(M-1));
    A  = RD(pPk, qPk) / (ND * sum(w));       % coherent complex amplitude estimate

    % Reconstruct data-stripped echo and subtract
    m    = 0:M-1;
    sPk  = circshift(s_mls, ell);
    echo = A * (sPk * exp(1j*2*pi*nuPk*m*Tblock));   % ND x M
    yp_sic = yp - echo;
    est    = struct('ell',ell, 'nu',nuPk, 'A',A);
end
