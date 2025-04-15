function [] = plotBodyTrajectoryTimeseries( BL, bodyTrajs, bodyStr)
    %PLOTBODYTRAJECTORYTIMESERIES given a cell array of body trajectories,
    %this function plots them in a timeseries-like fashion as a function of
    %the kinematic gait time

    % extract the number of body trajectories and obtain plotting styles
    % for each of them
    bNum = numel(bodyTrajs);
    load('Data\Plotting\qualitativeColorMap', '-mat', 'qualColMap');
    bCol = resizeColorMap( qualColMap, bNum );

    % load the data for plotting
    load('Data\Sinusoidal Gaits\clariConnVecF.mat', 'lW_s');

    % plotting limits
    gaitTimeLim = [0, 1]; gaitTimeTicks = linspace(0, 1, 11);
    tempXY = []; tempRot = [];
    for b = 1:bNum
        tempXY = [tempXY; % format this as a single column and convert!
                  bodyTrajs{b}(:, 1:2)/BL];
        tempRot =[tempRot; 
                  rad2deg(bodyTrajs{b}(:, 3))];
    end
    yLimPrct = 0.09;
    bLim = [ % limits and add a 'yLimPrct'% padding using the range
                [min(tempXY, [], 1), min(tempRot, [], 1)]; 
                [max(tempXY, [], 1), max(tempRot, [], 1)]
               ]';
    bRange = diff(bLim, 1, 2);
    bLim = bLim + yLimPrct*bRange*[-1, 1];

    % plot the trajectories
    fS = 12; scatS = 100;
    tNow = cell(1, bNum);
    % 
    f = figure('units', 'pixels', 'position', [3200 -300 900 600]); 
    set(f,'Visible','on');
    tl = tiledlayout(f, 3, 1, "TileSpacing", "tight", "Padding", "tight");
    % % 
    ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
    for b = 1:bNum
        tNow{b} = linspace(0, 1, size(bodyTrajs{b}, 1))';
        switch b/2 == floor(b/2) && b/2 == ceil(b/2)
            case 0
                linSty = '-';
            case 1
                linSty = '--';
        end
        plot(ax, tNow{b}, bodyTrajs{b}(:, 1)/BL, ...
            linSty, "LineWidth", lW_s, "Color", bCol(b, :), ...
            "DisplayName", bodyStr{b});
        p = scatter(ax, tNow{b}(1), bodyTrajs{b}(1, 1)/BL, ...
                    scatS, bCol(b, :), "filled", "o", "LineWidth", lW_s);
        set(get(get(p,'Annotation'),'LegendInformation'), ...
            'IconDisplayStyle','off');
        p = scatter(ax, tNow{b}(end), bodyTrajs{b}(end, 1)/BL, ...
            scatS, bCol(b, :), "Marker", "x", "LineWidth", lW_s);
        set(get(get(p,'Annotation'),'LegendInformation'), ...
            'IconDisplayStyle','off');
    end
    ax.FontSize = fS;
    xlim(ax, gaitTimeLim); ylim(ax, bLim(1, :));
    xticks(ax, gaitTimeTicks);
    xlabel(ax, 'Gait kinematic time', FontSize=fS);
    ylabel(ax, ...
        '$x_{b}$ (in BL)', ...
        FontSize=fS);
    Lgnd = legend(ax); Lgnd.Layout.Tile = 'east';
    % %
    ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
    for b = 1:bNum
        switch b/2 == floor(b/2) && b/2 == ceil(b/2)
            case 0
                linSty = '-';
            case 1
                linSty = '--';
        end
        plot(ax, tNow{b}, bodyTrajs{b}(:, 2)/BL, ...
            linSty, "LineWidth", lW_s, "Color", bCol(b, :));
        scatter(ax, tNow{b}(1), bodyTrajs{b}(1, 2)/BL, ...
                    scatS, bCol(b, :), "filled", "o", "LineWidth", lW_s);
        scatter(ax, tNow{b}(end), bodyTrajs{b}(end, 2)/BL, ...
            scatS, bCol(b, :), "Marker", "x", "LineWidth", lW_s);
    end
    ax.FontSize = fS;
    xlim(ax, gaitTimeLim); ylim(ax, bLim(2, :));
    xticks(ax, gaitTimeTicks);
    ylabel(ax, '$y_{b}$ (in BL)', FontSize=fS);
    % %
    ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
    for b = 1:bNum
        switch b/2 == floor(b/2) && b/2 == ceil(b/2)
            case 0
                linSty = '-';
            case 1
                linSty = '--';
        end
        plot(ax, tNow{b}, rad2deg(bodyTrajs{b}(:, 3)), ...
            linSty, "LineWidth", lW_s, "Color", bCol(b, :));
        scatter(ax, tNow{b}(1), rad2deg(bodyTrajs{b}(1, 3)), ...
                    scatS, bCol(b, :), "filled", "o", "LineWidth", lW_s);
        scatter(ax, tNow{b}(end), rad2deg(bodyTrajs{b}(end, 3)), ...
            scatS, bCol(b, :), "Marker", "x", "LineWidth", lW_s);
    end
    ax.FontSize = fS;
    xlim(ax, gaitTimeLim); ylim(ax, bLim(3, :));
    xticks(ax, gaitTimeTicks);
    ylabel(ax, '$\theta_{b}$ (in degs)', FontSize=fS);
    % % % 
    title(tl, 'Body trajectory', 'FontSize', fS, 'Interpreter', 'latex');
end