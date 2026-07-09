function plotGeometry(geom, savePath)
% PLOTGEOMETRY  Figure 3 — 3D Spatial Geometry of the Three-Node System.
%
%   plots.plotGeometry(geom, savePath)
%
%   Inputs:
%     geom     : ntn.Geometry
%     savePath : char  full path for the output PDF file

    fig = figure('Name', 'System Geometry', 'Position', [200 200 800 600]);
    
    % Use professional styling
    set(fig, 'Color', 'w');
    ax = axes('Parent', fig);
    grid(ax, 'on');
    hold(ax, 'on');
    box(ax, 'on');
    
    % Node coordinates
    pD = geom.pD;
    pU = geom.pU;
    pT = geom.pT;
    
    % Plot dashed links between nodes
    plot3(ax, [pD(1), pU(1)], [pD(2), pU(2)], [pD(3), pU(3)], ...
          'k--', 'LineWidth', 1.5, 'DisplayName', 'Direct Link (D \leftrightarrow U)');
    plot3(ax, [pD(1), pT(1)], [pD(2), pT(2)], [pD(3), pT(3)], ...
          'r--', 'LineWidth', 1.5, 'DisplayName', 'Radar Path (D \leftrightarrow T)');
    plot3(ax, [pT(1), pU(1)], [pT(2), pU(2)], [pT(3), pU(3)], ...
          'b--', 'LineWidth', 1.5, 'DisplayName', 'Scattered Path (T \leftrightarrow U)');
      
    % Plot node positions with premium markers
    plot3(ax, pD(1), pD(2), pD(3), ...
          'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'LineWidth', 2, ...
          'DisplayName', 'Drone (D)');
    plot3(ax, pU(1), pU(2), pU(3), ...
          'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2, ...
          'DisplayName', 'Ground UE (U)');
    plot3(ax, pT(1), pT(2), pT(3), ...
          'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2, ...
          'DisplayName', 'Moving Target (T)');

    % Draw velocity vectors using quiver3
    scale = 1.0; % Scaling factor for velocity arrows
    quiver3(ax, pD(1), pD(2), pD(3), geom.vD(1), geom.vD(2), geom.vD(3), scale, ...
            'Color', [0 0.4470 0.7410], 'LineWidth', 2, 'MaxHeadSize', 0.8, ...
            'DisplayName', 'Drone Vel (v_D)');
    quiver3(ax, pU(1), pU(2), pU(3), geom.vU(1), geom.vU(2), geom.vU(3), scale, ...
            'Color', [0.4660 0.6740 0.1880], 'LineWidth', 2, 'MaxHeadSize', 0.8, ...
            'DisplayName', 'UE Vel (v_U)');
    quiver3(ax, pT(1), pT(2), pT(3), geom.vT(1), geom.vT(2), geom.vT(3), scale, ...
            'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2, 'MaxHeadSize', 0.8, ...
            'DisplayName', 'Target Vel (v_T)');

    % Add text annotations offset slightly from the nodes
    text(ax, pD(1)+5, pD(2)+5, pD(3)+5, sprintf('Drone (%.0f, %.0f, %.0f) m', pD(1), pD(2), pD(3)), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', 'b');
    text(ax, pU(1)+5, pU(2)+5, pU(3)+5, sprintf('UE (%.0f, %.0f, %.0f) m', pU(1), pU(2), pU(3)), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
    text(ax, pT(1)+5, pT(2)+5, pT(3)+5, sprintf('Target (%.0f, %.0f, %.0f) m', pT(1), pT(2), pT(3)), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');

    % Set axes labels and properties
    xlabel(ax, 'X Position [m]', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel(ax, 'Y Position [m]', 'FontSize', 11, 'FontWeight', 'bold');
    zlabel(ax, 'Altitude Z [m]', 'FontSize', 11, 'FontWeight', 'bold');
    title(ax, '3D Spatial Geometry & Node Velocities', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Professional 3D view angle
    view(ax, -35, 30);
    axis(ax, 'equal');
    xlim(ax, [min([pD(1), pU(1), pT(1)])-30, max([pD(1), pU(1), pT(1)])+50]);
    ylim(ax, [min([pD(2), pU(2), pT(2)])-30, max([pD(2), pU(2), pT(2)])+50]);
    zlim(ax, [0, max([pD(3), pU(3), pT(3)])+30]);
    
    legend(ax, 'Location', 'northeastoutside', 'FontSize', 9);
    
    % Save image
    saveas(fig, savePath);
    fprintf('Saved: %s\n', savePath);
end
