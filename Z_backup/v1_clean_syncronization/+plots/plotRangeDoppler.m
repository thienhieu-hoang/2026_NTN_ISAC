function plotRangeDoppler(droneRx, geom, params, savePath)
% PLOTRANGEDOPPLER  Figure 1 — Joint Range-Doppler heatmaps before and
%   after SIC.
%
%   plots.plotRangeDoppler(droneRx, geom, params, savePath)
%
%   Inputs:
%     droneRx  : ntn.sensing.DroneReceiver  (must have RD_before, RD_after)
%     geom     : ntn.Geometry
%     params   : ntn.SystemParams
%     savePath : char  full path for the output PNG file

    figure('Name','Range-Doppler Map','Position',[100 100 1100 440]);

% -- -Left panel : before SIC-- - subplot(1, 2, 1);
normRef = max(abs(droneRx.RD_before( :)));
imagesc(droneRx.vel_axis, droneRx.rng_axis,
        ... 20 * log10(abs(droneRx.RD_before) / normRef));
set(gca, 'YDir', 'normal');
axis tight;
caxis([-50 0]);
colorbar;
xlabel('Velocity [m/s]');
ylabel('Range [m]');
title('Joint RD map (before SIC)');
colormap(jet);
hold on;
plot(geom.nu_DU *params.lamD / 2, geom.ell_2DU *params.dR, ... 'wo',
     'MarkerSize', 12, 'LineWidth', 1.5);
plot(geom.nu_mono *params.lamD / 2, geom.ell_mono *params.dR, ... 'w^',
     'MarkerSize', 12, 'LineWidth', 1.5);
legend('UE', 'Target', 'TextColor', 'w', 'Location', 'northeast');

% -- -Right panel : after SIC-- - subplot(1, 2, 2);
imagesc(droneRx.vel_axis, droneRx.rng_axis,
        ... 20 * log10(abs(droneRx.RD_after) / normRef));
set(gca, 'YDir', 'normal');
axis tight;
caxis([-50 0]);
colorbar;
xlabel('Velocity [m/s]');
ylabel('Range [m]');
title('After SIC');
colormap(jet);
hold on;

% Determine which reflector was removed and which one remains d_to_UE =
    abs(droneRx.tgEst.ell - geom.ell_2DU);
d_to_Target = abs(droneRx.tgEst.ell - geom.ell_mono);

if d_to_UE
  < d_to_Target % UE was removed,
      Target remains plot(geom.nu_mono *params.lamD / 2,
                          geom.ell_mono *params.dR, ... 'w^', 'MarkerSize', 12,
                          'LineWidth', 1.5);
legend('Target', 'TextColor', 'w', 'Location', 'northeast');
else % Target was removed,
    UE remains plot(geom.nu_DU *params.lamD / 2, geom.ell_2DU *params.dR,
                    ... 'wo', 'MarkerSize', 12, 'LineWidth', 1.5);
legend('UE', 'TextColor', 'w', 'Location', 'northeast');
end

    saveas(gcf, savePath);
fprintf('Saved: %s\n', savePath);
end end
