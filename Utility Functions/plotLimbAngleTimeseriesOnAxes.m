function plotLimbAngleTimeseriesOnAxes(ax, tau, beta, limbAngle, ...
                                                stanceColor, swingColor)
%PLOTLIMBANGLETIMESERIESONAXES plots the limb angle timeseries on the axes
%   This function plots the limb angle as a timeseries when provided with
%   an axis object, time vector, contact state (boolean) timeseries vector, 
%   and limb angle timeseries vector.
    % get the points where there is a stance-swing contact state swap
    switchIdx = find(diff(beta));
    % if the final index of switchIdx is not equal to the final timestep,
    % then append it at the end of the switchIdx
    if switchIdx(end) < numel(tau)-1
        switchIdx(end+1) =  numel(tau)-1; % draw the last segment
    end
    % iterate and plot the trajectory
    for i = 1:numel(switchIdx)
        % get the start index
        switch i
            case 1
                startIdx = 1;
            otherwise
                startIdx = stopIdx;
        end
        % get the stop index
        stopIdx = switchIdx(i) + 1;
        % get the current color
        switch beta(startIdx)
            case 1
                scatterfaceStateColor = stanceColor;
                scatterEdgeStateColor = stanceColor;
                currentStateColor = stanceColor;
                currentStateAlpha = 1.0;
            case 0
                scatterfaceStateColor = 'none';
                scatterEdgeStateColor = swingColor;
                currentStateColor = swingColor;
                currentStateAlpha = 0.25;
        end
        % % get the current transparency
        % switch beta(startIdx)
        %     case 1
        %         currentStateAlpha = 1;
        %     case 0
        %         currentStateAlpha = 0.5;
        % end
        % plot the current trajectory
        scatter(ax, tau(startIdx), limbAngle(startIdx), 100, 'o', "filled", ...
            "MarkerFaceColor", scatterfaceStateColor, ...
            'MarkerFaceAlpha', currentStateAlpha, ...
            "MarkerEdgeColor", scatterEdgeStateColor, ...
            "LineWidth", 1.2, 'MarkerEdgeAlpha', currentStateAlpha);
        plot(ax, tau(startIdx:stopIdx), limbAngle(startIdx:stopIdx), '-', ...
            'LineWidth', 2.0, ...
            'Color', [currentStateColor, currentStateAlpha]);
    end

end

