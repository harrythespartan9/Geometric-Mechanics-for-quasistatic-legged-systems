function plotModeSwitchingShapeTrajectory(ax, x, y, c, Lw, Sty, Col)
%PLOTMODESWITCHINGSHAPETRAJECTORY Given a shape-space trajectory with a contact state vector, this function plots the trajectory incrementally between the
%current time step and next time step. The style of the trace should reflect the contact states.
    
    % start iterating over the contact trajectory
    for i = 1:(numel(c)-1)
        switch c(i)
            case 1
                plot(ax, x(i:i+1), y(i:i+1), Sty{1}, 'LineWidth', Lw, 'Color', Col);
            case 0
                plot(ax, x(i:i+1), y(i:i+1), Sty{2}, 'LineWidth', Lw, 'Color', Col);
        end
    end

end

