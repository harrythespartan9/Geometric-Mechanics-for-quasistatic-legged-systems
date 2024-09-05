% this function plots an arrow at the specified location in a path
% ... this function accepts additional arguments like location on the path
% ... we are not doing any arguments-check to remove overhead
function h = plotPathArrowV2(ax, aX, aY, arrowSize, arrowAngle, lW, c,...
                                                            arrowLocation)

    % based on the arrow location, execute the following
    % ... 'mid' plots a forward-facing (in the discrete array start:end
    % ... facing) arrow at the ~midpoint of the path
    % ... 'front-end' and 'back-end' plots arrows at the front and back
    % ... while facing the corresponding directions
    % ... we find the location of the tip and find the offset location in
    % ... the direction opposite to that of the arrow
    switch arrowLocation
        case 'mid'
            tipLoc = [ceil(numel(aX)/2) ceil(numel(aY)/2)];
            offsetRelLoc = -1*ones(1, 2);
        case 'front_end'
            tipLoc = [numel(aX) numel(aY)];
            offsetRelLoc = -1*ones(1, 2);
        case 'back_end'
            tipLoc = ones(1, 2);
            offsetRelLoc = +1*ones(1, 2);
        case 'ends'
            tipLoc = [
                      [numel(aX) numel(aY)]; 
                       ones(1, 2)
                     ];
            offsetRelLoc = [
                            -1*ones(1, 2); 
                             +1*ones(1, 2)
                           ];
    end

    % based on one or more arrows needed, plot accordingly
    switch size(tipLoc, 1)

        case 1
            % the arrow quiver U and V values
            delx = aX(tipLoc(1)) - aX(tipLoc(1)+offsetRelLoc(1));
            dely = aY(tipLoc(2)) - aY(tipLoc(2)+offsetRelLoc(2));    
            % find the arrow plotting angle
            plot_angle = atan2(dely, delx)*ones(1,2) + arrowAngle*[-1, 1];
            plot_from = nan(2,2);
            plot_to = [aX(tipLoc(1)), aY(tipLoc(2))];
            % plot the arrow
            for i = 1:numel(plot_angle)
                % compute the point to plot the arrow from
                plot_from(i, :) = plot_to ...
                    - arrowSize*[ cos(plot_angle(i)), sin(plot_angle(i)) ];
            end
            % Plot the arrow
            h = plot(ax,...
                [plot_from(1, 1) plot_to(1) plot_from(2, 1)],...
                [plot_from(1, 2) plot_to(2) plot_from(2, 2)],...
                'LineWidth', lW, 'Color', c);

        otherwise

            % iterate over the number of arrows needed
            h = cell(size(tipLoc, 1), 1);
            for i = 1:size(tipLoc, 1)
                delx = aX(tipLoc(i, 1)) ...
                    - aX(tipLoc(i, 1)+offsetRelLoc(i, 1));
                dely = aY(tipLoc(i, 2)) ...
                    - aY(tipLoc(i, 2)+offsetRelLoc(i, 2));
                plot_angle = atan2(dely, delx)*ones(1,2) ...
                                                    + arrowAngle*[-1, 1];
                plot_from = nan(2,2);
                plot_to = [aX(tipLoc(i, 1)), aY(tipLoc(i, 2))];
                for j = 1:numel(plot_angle)
                    plot_from(j, :) = plot_to ...
                        - arrowSize*[cos(plot_angle(j)), sin(plot_angle(j))];
                end
                h{i} = plot(ax,...
                [plot_from(1, 1) plot_to(1) plot_from(2, 1)],...
                [plot_from(1, 2) plot_to(2) plot_from(2, 2)],...
                'LineWidth', lW, 'Color', c);
            end
            

    end

    
    
end