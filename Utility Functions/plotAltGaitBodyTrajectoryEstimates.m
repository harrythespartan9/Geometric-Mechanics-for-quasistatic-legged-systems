function plotAltGaitBodyTrajectoryEstimates(discJointStancePath, zoomFlag)
%PLOTALTGAITBODYTRAJECTORYESTIMATES plot the body trajectory of the system
%when performing an alternating gait and its estimates using the BCH
%formula
    % check if the jointStancePath information is provided
    if ~isfield(discJointStancePath, 'g') ||...
       ~isfield(discJointStancePath, 'z') ||...
       ~isfield(discJointStancePath, 'gHat') ||...
       ~isfield(discJointStancePath, 'zHat')
        error(['The struct "jointStancePath.discretized" does not contain ' ...
            'the fields required to plot the body trajectory and its ' ...
            'estimates.']);
    end
    % check if zoomFlag is provided
    if nargin < 2
        zoomFlag = false;
    end
    % unpack
    b = discJointStancePath.g;
    zb = discJointStancePath.z;
    % setups
    fS = 25; 
    appxPicewCol = [51,160,44]/255;
    appxCol = [31,120,180]/255; appxfA = 0.2*(1:4);
    gbCol = [0, 0, 0]/255;
    figure('Visible', 'on', 'Units', 'pixels', 'Position', [0 0 900 900]); ax = gca; box(ax, "on");
    % plot the body trajectory, represent the net displacement, and the body
    % orientation
    plot(ax, b(:, 1), b(:, 2), '-', 'LineWidth', 2.0, 'Color', gbCol, ...
        'DisplayName', 'simulated trajectory');
    grid(ax, "on"); hold(ax, "on"); set(ax,'TickLabelInterpreter','latex');
    p = scatter(ax, zb(1), zb(2), 100, 'o', "filled", "MarkerFaceColor", gbCol, 'MarkerFaceAlpha', 1.00);
    set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    p = quiver(ax, zb(1),  zb(2), -sin(zb(3)), cos(zb(3)),...
        'LineWidth', 3.0, 'Color', gbCol, 'LineStyle', '-',...
        'AutoScaleFactor', 0.1, 'MaxHeadSize', 1);
    set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    % plot the stancewise commutative estimate
    gHatb = discJointStancePath.gHat{1}; 
    zHatb = discJointStancePath.zHat{1}; % locally unpack
    plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0,...
        'Color', [ appxPicewCol, 1.0 ], ...
            'DisplayName', 'stancewise commutative');
    p = scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
            "MarkerFaceColor", appxPicewCol, 'MarkerFaceAlpha', 1.0, ...
            'MarkerEdgeColor', appxPicewCol, 'MarkerEdgeAlpha', 1.0);
    set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    % plot the single flow estimates
    for i = 2:numel(discJointStancePath.gHat)
        gHatb = discJointStancePath.gHat{i}; 
        zHatb = discJointStancePath.zHat{i}; % unpack for each case
        plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0, 'Color', [ appxCol, appxfA(i-1) ], ...
            'DisplayName', ['$$' num2str(i-1) '^{\circ}$$ approximation']);
        p = scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
                "MarkerFaceColor", appxCol, 'MarkerFaceAlpha', appxfA(i-1), ...
                'MarkerEdgeColor', appxCol, 'MarkerEdgeAlpha', appxfA(i-1));
        set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
    % remaining setup
    p = xline(0, 'k:', 'LineWidth', 0.1); 
    set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    p = yline(0, 'k:', 'LineWidth', 0.1);
    set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    legend(ax, 'location', 'bestoutside', 'Interpreter', 'latex');
    axis(ax, "equal", "padded"); 
    ax.FontSize = fS; ax.XTick = ''; ax.YTick = '';
    xlabel('$$x$$', 'FontSize', fS, 'Interpreter', 'latex');
    ylabel('$$y$$', 'FontSize', fS, 'Interpreter', 'latex');
    %%% ADDITIONAL PLOT WITHOUT LEGEND FOR ZOOMING IN, ETC.
    % ... same shit as the last plot, except without the labels, legends,
    % ... and any other text
    if zoomFlag
        figure('Visible', 'on', 'Units', 'pixels', 'Position', [0 0 600 600]); ax = gca; box(ax, "on");
        plot(ax, b(:, 1), b(:, 2), '-', 'LineWidth', 2.0, 'Color', gbCol);
        grid(ax, "on"); hold(ax, "on"); set(ax,'TickLabelInterpreter','latex');
        scatter(ax, zb(1), zb(2), 100, 'o', "filled", "MarkerFaceColor", gbCol, 'MarkerFaceAlpha', 1.00);
        gHatb = discJointStancePath.gHat{1}; zHatb = discJointStancePath.zHat{1};
        plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0, 'Color', [ appxPicewCol, 1.0 ]);
        scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
                "MarkerFaceColor", appxPicewCol, 'MarkerFaceAlpha', 1.0, ...
                'MarkerEdgeColor', appxPicewCol, 'MarkerEdgeAlpha', 1.0);
        for i = 2:numel(discJointStancePath.gHat)-1
            gHatb = discJointStancePath.gHat{i}; zHatb = discJointStancePath.zHat{i};
            plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0, 'Color', [ appxCol, appxfA(i) ]);
            scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
                    "MarkerFaceColor", appxCol, 'MarkerFaceAlpha', appxfA(i), ...
                    'MarkerEdgeColor', appxCol, 'MarkerEdgeAlpha', appxfA(i));
        end
        xline(0, 'k:', 'LineWidth', 0.1); yline(0, 'k:', 'LineWidth', 0.1);
        axis(ax, "equal", "padded"); 
        ax.FontSize = fS; ax.XTick = ''; ax.YTick = '';
    end

end

