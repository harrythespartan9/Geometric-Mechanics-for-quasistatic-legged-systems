function f = plotSE2bodytrajectory(in)
%PLOTSE2BODYTRAJECTORY this function plots the SE(2) body trajectory-- projection and estimates
%   Provided with an input body trajectory structure similar to "plotSE2timeseries.m" function, this function plots it on the SE(2) position space. 
    
    % Unpack
    b = in; % {1} - x, {2} - y, and {3} - \theta
    trace_num = numel(b); col = turbo(trace_num);
    
    % Iterate and plot
    f = figure('units','pixels','position',360*[0 0 1 1],'Color','w'); ax = gca;
    fS = 10;
    for i = 1:trace_num
        plot(ax, b{i}{1}, b{i}{2}, 'LineWidth', 2.0, 'Color', col(i, :));
        if i == 1
            grid on; hold on; set(ax,'TickLabelInterpreter','latex'); axis equal square;
        end
        quiver(ax, b{i}{1}, b{i}{2}, cos(b{i}{3}), sin(b{i}{3}),...
            'LineWidth', 2.0, 'Color', col(i, :), 'AutoScaleFactor', 0.45); % x-axis corresponds to heading
    end
    ylabel('$$y$$', 'FontSize', fS, 'Interpreter', 'latex'); ax.FontSize = fS;
    xlabel('$$x$$', 'FontSize', fS, 'Interpreter', 'latex');

end

