function f = plotSE2bodytrajectory(in)
%PLOTSE2BODYTRAJECTORY this function plots the SE(2) body trajectory-- projection and estimates
%   Provided with an input body trajectory structure similar to "plotSE2timeseries.m" function, this function plots it on the SE(2) position space. 
    
    % Unpack
    b = in{1}; % {1} - x, {2} - y, and {3} - \theta
    trace_num = numel(b); 
    switch numel(in)
        case 2
            col = in{2}{1}; sty = in{2}{2}; lW = in{2}{3};
        case 1
            col = turbo(trace_num); sty = cell(trace_num, 1); lW = sty;
            for i = 1:trace_num
                sty{i} = '-'; lW{i} = 2.0;
            end
    end

    % get the number of points
    num_traj = numel(b{1}{1});
    skp_percent = 0;            % set the number of points to skip when making the yaw quiver
    idxQ = 1:round(skp_percent/100*num_traj)+1:num_traj;
    
    % Iterate and plot
    f = figure('units','pixels','position',360*[0 0 1 1],'Color','w'); ax = gca; set(f, 'Visible', 'on');
    fS = 10;
    for i = 1:trace_num
        plot(ax, b{i}{1}, b{i}{2}, sty{i}, 'LineWidth', lW{i}, 'Color', col(i, :));
        if i == 1
            grid on; hold on; set(ax,'TickLabelInterpreter','latex'); axis equal;
        end
        quiver(ax, b{i}{1}(idxQ), b{i}{2}(idxQ), cos(b{i}{3}(idxQ)), sin(b{i}{3}(idxQ)),...
            'LineWidth', 0.5, 'Color', col(i, :), 'LineStyle', ':',...
            'ShowArrowHead', 'off','AutoScale', 'off'); % x-axis corresponds to heading % 'AutoScaleFactor', 0.45
    end
    ylabel('$$y$$', 'FontSize', fS, 'Interpreter', 'latex'); ax.FontSize = fS;
    xlabel('$$x$$', 'FontSize', fS, 'Interpreter', 'latex');

end

