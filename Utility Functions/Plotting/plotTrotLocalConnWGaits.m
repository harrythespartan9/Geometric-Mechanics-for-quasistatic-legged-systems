function plotTrotLocalConnWGaits( i, gaits, gaitStr, removeVecFcolFlag )
    %PLOTTROTLOCALCONN plot the local connection vector field for a trot
    %gait with gaits plotted on top for the given body sprawl angle

    % obtain the number of gaits and setup colors for each
    gNum = numel(gaits);
    load('Data\Plotting\qualitativeColorMap', '-mat', 'qualColMap');
    gCol = resizeColorMap( qualColMap, gNum );


    % load the data for plotting
    load('Data\Sinusoidal Gaits\clariConnVecF.mat');

    % if no information provide, keep the distinct colors for vector fields
    % on each stance phase
    if nargin < 4
        removeVecFcolFlag = false;
    end

    % if the vector field colors are requested to be removed, use black,
    % else saturate vector field colors to white
    switch removeVecFcolFlag
        case 1
            % black out the vector field
            col13s = zeros(1, 3); col24s = col13s;
        case 0
            % saturate the vector field colors before plotting to emphasize 
            % the stance phase trajectories
            % ... play around with the saturation factor to find the the 
            % ... most "whited-out" color you like
            satFact = 0.50;
            col13s = saturateColorArray(col13, satFact);
            col24s = saturateColorArray(col24, satFact);
    end

    % plot the local connection vector fields
    % ... plot it in 2 cols for each contact state
    fS = 12; scatS = 100;
    % 
    f = figure('units', 'pixels', 'position', [3200 -300 700 900]); 
    set(f,'Visible','on');
    tl = tiledlayout(f, 3, 2, "TileSpacing", "tight", "Padding", "tight");
    % %
    ax = nexttile(tl); hold(ax, "on"); % Ax for stance 13
    p = quiver(ax, ...
        rad2deg(nalpha_x), rad2deg(nalpha_y), ...
        nAu_13_X{i}, nAv_13_X{i}, ...
        "LineWidth", lW_Vector, "Color", col13s);
    set(get(get(p,'Annotation'),'LegendInformation'), ...
        'IconDisplayStyle','off');
    plotStanceITrajectory(ax, gaits, lW_s, scatS, gCol, gaitStr);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, angleLimits); ylim(ax, angleLimits);
    xticks(ax, angleTicks); yticks(ax, angleTicks);
    xlabel(ax, ...
        'FR angle $\alpha_{1}$ $({}^{\circ})$', ...
        FontSize=fS);
    ylabel(ax, ...
        'HL angle $\alpha_{3}$ $({}^{\circ})$', ...
        FontSize=fS);
    title(ax, 'Lateral body velocity $-\mathbf{A}^{x}_{13}$', FontSize=fS);
    Lgnd = legend(ax); Lgnd.Layout.Tile = 'east';
    % %
    ax = nexttile(tl); hold(ax, "on"); % Ax for stance 24
    quiver(ax, ...
        rad2deg(nalpha_x), rad2deg(nalpha_y), ...
        nAu_24_X{i}, nAv_24_X{i}, ...
        "LineWidth", lW_Vector, "Color", col24s);
    plotStanceJTrajectory(ax, gaits, lW_s, scatS, gCol);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, angleLimits); ylim(ax, angleLimits);
    xticks(ax, angleTicks); yticks(ax, angleTicks);
    xlabel(ax, ...
        'FL angle $\alpha_{2}$ $({}^{\circ})$', ...
        FontSize=fS);
    ylabel(ax, ...
        'HR angle $\alpha_{4}$ $({}^{\circ})$', ...
        FontSize=fS);
    title(ax, 'Lateral body velocity $-\mathbf{A}^{x}_{24}$', FontSize=fS);
    % %
    ax = nexttile(tl); hold(ax, "on"); % Ay for stance 13
    quiver(ax, ...
        rad2deg(nalpha_x), rad2deg(nalpha_y), ...
        nAu_13_Y{i}, nAv_13_Y{i}, ...
        "LineWidth", lW_Vector, "Color", col13s);
    plotStanceITrajectory(ax, gaits, lW_s, scatS, gCol);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, angleLimits); ylim(ax, angleLimits);
    xticks(ax, angleTicks); yticks(ax, angleTicks);
    xlabel(ax, ...
        'FR angle $\alpha_{1}$ $({}^{\circ})$', ...
        FontSize=fS);
    ylabel(ax, ...
        'HL angle $\alpha_{3}$ $({}^{\circ})$', ...
        FontSize=fS);
    title(ax, ...
        'Longitudinal body velocity $-\mathbf{A}^{y}_{13}$', FontSize=fS);
    % %
    ax = nexttile(tl); hold(ax, "on"); % Ay for stance 24
    quiver(ax, ...
        rad2deg(nalpha_x), rad2deg(nalpha_y), ...
        nAu_24_Y{i}, nAv_24_Y{i}, ...
        "LineWidth", lW_Vector, "Color", col24s);
    plotStanceJTrajectory(ax, gaits, lW_s, scatS, gCol);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, angleLimits); ylim(ax, angleLimits);
    xticks(ax, angleTicks); yticks(ax, angleTicks);
    xlabel(ax, ...
        'FL angle $\alpha_{2}$ $({}^{\circ})$', ...
        FontSize=fS);
    ylabel(ax, ...
        'HR angle $\alpha_{4}$ $({}^{\circ})$', ...
        FontSize=fS);
    title(ax, ...
        'Longitudinal body velocity $-\mathbf{A}^{y}_{24}$', FontSize=fS);
    % %
    ax = nexttile(tl); hold(ax, "on"); % Atheta for stance 13
    quiver(ax, ...
        rad2deg(nalpha_x), rad2deg(nalpha_y), ...
        nAu_13_TH{i}, nAv_13_TH{i}, ...
        "LineWidth", lW_Vector, "Color", col13s);
    plotStanceITrajectory(ax, gaits, lW_s, scatS, gCol);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, angleLimits); ylim(ax, angleLimits);
    xticks(ax, angleTicks); yticks(ax, angleTicks);
    xlabel(ax, ...
        'FR angle $\alpha_{1}$ $({}^{\circ})$', ...
        FontSize=fS);
    ylabel(ax, ...
        'HL angle $\alpha_{3}$ $({}^{\circ})$', ...
        FontSize=fS);
    title(ax, ...
        'Rotational body velocity $-\mathbf{A}^{\theta}_{13}$', ...
        FontSize=fS);
    % %
    ax = nexttile(tl); hold(ax, "on"); % Atheta for stance 24
    quiver(ax, ...
        rad2deg(nalpha_x), rad2deg(nalpha_y), ...
        nAu_24_TH{i}, nAv_24_TH{i}, ...
        "LineWidth", lW_Vector, "Color", col24s);
    plotStanceJTrajectory(ax, gaits, lW_s, scatS, gCol);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, angleLimits); ylim(ax, angleLimits);
    xticks(ax, angleTicks); yticks(ax, angleTicks);
    xlabel(ax, ...
        'FL angle $\alpha_{2}$ $({}^{\circ})$', ...
        FontSize=fS);
    ylabel(ax, ...
        'HR angle $\alpha_{4}$ $({}^{\circ})$', ...
        FontSize=fS);
    title(ax, ...
        'Rotational body velocity $-\mathbf{A}^{\theta}_{24}$', ...
        FontSize=fS);
end

%% AUXILIARY FUNCTIONS

% plot the stance 13 trajectory with a o-marker for the IC and x-marker for
% the FC
function plotStanceITrajectory(ax, gaits, lW_s, scatS, gCol, gStr)
    gNum = numel(gaits); % number of gaits
    I = 1; J = 3; % choose contact state: 13
    tStart = 0.0; tEnd = 0.5; % choose starting/ending time for 1st half
    for g = 1:gNum
        idx = (gaits{g}(1, :) >= tStart & gaits{g}(1, :) <= tEnd); 
        gNow = rad2deg(gaits{g}(2:end, idx)); % current gait cycle in degs
        gNowI = gNow(I, :); gNowJ = gNow(J, :);
        switch g/2 == floor(g/2) && g/2 == ceil(g/2)
            case 0
                linSty = '-';
            case 1
                linSty = '--';
        end
        switch nargin == 6
            case 1
                plot(ax, gNowI, gNowJ, linSty, ... % plot the trajectory
                    "LineWidth", lW_s, "Color", gCol(g, :), ...
                    "DisplayName", gStr{g});
                p = scatter(ax, gNowI(1), gNowJ(1), ... % o-marker at IC
                    scatS, gCol(g, :), "filled", "o", "LineWidth", lW_s);
                set(get(get(p,'Annotation'),'LegendInformation'), ...
                    'IconDisplayStyle','off');
                p = scatter(ax, gNowI(end), gNowJ(end), ... % x-marker @ FC
                    scatS, gCol(g, :), "Marker", "x", "LineWidth", lW_s);
                set(get(get(p,'Annotation'),'LegendInformation'), ...
                    'IconDisplayStyle','off');
            case 0
                plot(ax, gNowI, gNowJ, linSty, ... 
                    "LineWidth", lW_s, "Color", gCol(g, :));
                scatter(ax, gNowI(1), gNowJ(1), ...
                    scatS, gCol(g, :), "filled", "o", "LineWidth", lW_s);
                scatter(ax, gNowI(end), gNowJ(end), ...
                    scatS, gCol(g, :), "Marker", "x", "LineWidth", lW_s);
        end
        
    end
end

function plotStanceJTrajectory(ax, gaits, lW_s, scatS, gCol, gStr)
    gNum = numel(gaits); 
    I = 2; J = 4; % choose contact state: 24
    tStart = 0.5; tEnd = 1.0; % choose starting/ending time for 2nd half
    for g = 1:gNum
        idx = (gaits{g}(1, :) >= tStart & gaits{g}(1, :) <= tEnd); 
        gNow = rad2deg(gaits{g}(2:end, idx));
        gNowI = gNow(I, :); gNowJ = gNow(J, :);
        switch g/2 == floor(g/2) && g/2 == ceil(g/2) % is even
            case 1
                linSty = '--';
            case 0
                linSty = '-';
        end
        switch nargin == 6
            case 1
                plot(ax, gNowI, gNowJ, linSty, ...
                    "LineWidth", lW_s, "Color", gCol(g, :), ...
                    "DisplayName", gStr{g});
                p = scatter(ax, gNowI(1), gNowJ(1), ...
                    scatS, gCol(g, :), "filled", "o", "LineWidth", lW_s);
                set(get(get(p,'Annotation'),'LegendInformation'), ...
                    'IconDisplayStyle','off');
                p = scatter(ax, gNowI(end), gNowJ(end), ...
                    scatS, gCol(g, :), "Marker", "x", "LineWidth", lW_s);
                set(get(get(p,'Annotation'),'LegendInformation'), ...
                    'IconDisplayStyle','off');
            case 0
                plot(ax, gNowI, gNowJ, linSty, ... 
                    "LineWidth", lW_s, "Color", gCol(g, :));
                scatter(ax, gNowI(1), gNowJ(1), ...
                    scatS, gCol(g, :), "filled", "o", "LineWidth", lW_s);
                scatter(ax, gNowI(end), gNowJ(end), ...
                    scatS, gCol(g, :), "Marker", "x", "LineWidth", lW_s);
        end
    end
end