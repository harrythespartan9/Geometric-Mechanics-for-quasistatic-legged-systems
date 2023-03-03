% this function plots an arrow at the specified location of a path.
function h = plotpatharrow(ax, a1, a2, arrSize, arrAngle, lW, c)
    

    % find the slope of the path at the chosen point with backward difference
    delx = a1(ceil(numel(a1)/2)) - a1(ceil(numel(a1)/2)-1);
    dely = a2(ceil(numel(a2)/2)) - a2(ceil(numel(a2)/2)-1);

    % find the arrow plotting angle
    plot_angle = atan2(dely, delx)*ones(1,2) + arrAngle*[-1, 1];
    plot_from = nan(2,2);
    plot_to = repmat([a1(ceil(numel(a1)/2)), a2(ceil(numel(a2)/2))], 2, 1);

    % initialize the return object
    h = cell(1,numel(plot_angle)); % the arrow is drawn with two strokes

    % plot the arrow
    for i = 1:numel(plot_angle)

        % compute the point to plot the arrow from
        plot_from(i, :) = plot_to(i, :) - arrSize*[ cos(plot_angle(i)), sin(plot_angle(i)) ];

    end

    % Plot the arrow
    h = plot(ax, [plot_from(1, 1) plot_to(1, 1) plot_from(2, 1)], [plot_from(1, 2) plot_to(1, 2) plot_from(2, 2)], 'LineWidth', lW, 'Color', c);
    
end