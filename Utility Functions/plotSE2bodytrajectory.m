function f = plotSE2bodytrajectory(in, plotF)
%PLOTSE2BODYTRAJECTORY this function plots the SE(2) body trajectory-- projection and estimates
%   Provided with an input body trajectory structure similar to "plotSE2timeseries.m" function, this function plots it on the SE(2) position space. 
    
    % Unpack
    b = in{1}; % {1} - x, {2} - y, and {3} - \theta
    b_tau = in{2};
    trace_num = numel(b); 
    switch numel(in)
        case 3
            col = in{3}{1}; 
            % sty = in{3}{2}; 
            lW = in{3}{3};
        case 2
            col = turbo(trace_num); 
            % sty = cell(trace_num, 1); 
            lW = cell(trace_num, 1);
            for i = 1:trace_num
                % sty{i} = '-'; 
                lW{i} = 2.0;
            end
    end
    
    % Iterate and plot
    f = figure('units','pixels','position',480*[0 0 1 0.35],'Color','w'); ax = gca; set(f, 'Visible', 'on');
    fS = 10;
    for i = 1:trace_num
        if plotF{i}
            plot(ax, b{i}{1}, b{i}{2}, ':', 'LineWidth', 1.2, 'Color', col(i, :));
            if i == 1
                grid on; hold on; set(ax,'TickLabelInterpreter','latex'); axis equal tight;
            end
            scatter(ax, b_tau{i}{1}, b_tau{i}{2}, 50, col(i, :), "o", "filled", 'MarkerFaceAlpha', 0.75);
            quiver(ax, b_tau{i}{1}, b_tau{i}{2}, cos(b_tau{i}{3}), sin(b_tau{i}{3}),...
                'LineWidth', lW{i}, 'Color', col(i, :), 'LineStyle', '-',...
                'AutoScaleFactor', 0.20); % ,'ShowArrowHead', 'off','AutoScale', 'off'
        end
    end
    ylabel('$$y$$', 'FontSize', fS, 'Interpreter', 'latex'); ax.FontSize = fS;
    xlabel('$$x$$', 'FontSize', fS, 'Interpreter', 'latex');

end

