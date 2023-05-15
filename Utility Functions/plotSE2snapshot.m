% This function plots the snapshot of the given SE(2) system kinematics. Since the SE(2) system follows the frame convention in "se2_toyproblems_case_1.mlx" we
% need to convert this into the HAMR dataset format.
function h = plotSE2snapshot(ax, traj, fno)
    
    % make sure the frame number is not an erroneous input
    if nargin > 2
        if isempty(fno)
            fno = 1;
        else
            if fno > numel(traj.th2_e__b)
                error('ERROR! The frame for the snapshot can''t be higher than the number of frames in the trajectory.');
            end
        end
    elseif nargin == 2
        fno = 1;
    else
        error(['ERROR! A current axis object (obtain using the "gca" command or "gcf.CurrentAxes" command), trajectory struct, and system''s kinematic ' ...
            'information is needed to plot the SE(3) configuration. Refer to "HAMR6_SE3.mlx" as a tutorial to generate compatible kinematics.']);
    end

    % Unpack
    tbody = traj.tbody{fno};
    tframes = traj.tframes{fno};
    tboxes = traj.tboxes{fno};
    tlegs = traj.tlegs{fno}; tlegs0 = traj.tlegs0{fno};
    tfootS = traj.tfootS(:, fno);
    plotlim = [min(traj.plotlim(3, :)), max(traj.plotlim(4, :)),...
            min(-traj.plotlim(2, :)), max(-traj.plotlim(1, :))] + 0.07*traj.bl*repmat([-1, 1], [1, 2]);
    if plotlim(1) > plotlim(2)
        plotlim = [plotlim(2), plotlim(1), plotlim(3:end)];
    elseif plotlim(3) > plotlim(4)
        plotlim = [plotlim(1:2), plotlim(4), plotlim(3)];
    end
    gr_col = [100, 100, 100]/255; % gray color used for robot body, etc
    if isfield(traj, 'exp')
        spcl_col = traj.exp.col{fno} ;
        foot = [tlegs(:,1), tlegs(:,2)] + [tlegs(:,3), tlegs(:,4)];
        if spcl_col == 100*ones(size(spcl_col))/255
            scatS = 12; FillF = false;
        elseif spcl_col == zeros(size(spcl_col))
            scatS = 25; FillF = true;
        else
            scatS = 75; FillF = true;
        end
    end

    % plot the swing and lift trajectories in a loop
    switch isfield(traj, 'exp')

        case 0 %%%%%%%%%%%%%%%% simple case with just a single configuration

            % plot the required configuration
            xline(0, ':', 'LineWidth', 0.5, 'Color', 'k');
            hold(ax, 'on'); grid(ax, 'off'); axis(ax, 'square', 'equal', 'padded', plotlim); 
            xlabel(ax, 'X'); ylabel(ax, 'Y'); xticklabels(ax, ''); yticklabels(ax, '');
            yline(0, ':', 'LineWidth', 0.5, 'Color', 'k');                                                          % axes at the origin and labels
            quiver(tframes(:,2), -tframes(:,1), tframes(:,4), -tframes(:,3),...
                'LineWidth', 1.2, 'Color', gr_col, 'AutoScale', 'off');                                             % all the frames-- body, hip, and legs
            quiver(tboxes(:,2), -tboxes(:,1), tboxes(:,4), -tboxes(:,3),...
                'LineWidth', 3.0, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                     % all the bounding boxes -- except feet
            for j = 1:4
                quiver(tlegs(j,2), -tlegs(j,1), tlegs(j,4), -tlegs(j,3),...
                    'LineWidth', 4.0, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                 % all legs from the hip
                quiver(tlegs0(j,2), -tlegs0(j,1), tlegs0(j,4), -tlegs0(j,3),...
                    'LineStyle', '-',...
                    'LineWidth', 1.2, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                 % all lges origin/eq position from the hip
                plot(tfootS{j}(2, :), -tfootS{j}(1, :), '--', 'LineWidth', 1.2, 'Color', gr_col);                   % swing angle highlight
            end

        case 1 %%%%%%%%%%%%%%%% with experimentally obtained contact states
            
            c = 1;
            xline(0, ':', 'LineWidth', 0.5, 'Color', 'k');
            hold(ax, 'on'); grid(ax, 'off'); axis(ax, 'square', 'equal', 'padded', plotlim); 
            xlabel(ax, 'X'); ylabel(ax, 'Y'); xticklabels(ax, ''); yticklabels(ax, '');
            yline(0, ':', 'LineWidth', 0.5, 'Color', 'k');
            h{c} = quiver(tframes(:,2), -tframes(:,1), tframes(:,4), -tframes(:,3),...
                'LineWidth', 1.2, 'Color', gr_col, 'AutoScale', 'off'); c=c+1;
            h{c} = quiver(tboxes(:,2), -tboxes(:,1), tboxes(:,4), -tboxes(:,3),...
                'LineWidth', 3.0, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off'); c=c+1;
            switch FillF
                case 1
                    scatter(tbody(2), -tbody(1), scatS, spcl_col, 'filled', 'o');                         % COM position
                case 0
                    scatter(tbody(2), -tbody(1), scatS, spcl_col, 'o');
            end
            for j = 1:4
                switch traj.exp.C_i{j}(fno)
                    case 0
                        j_col = gr_col; FillF = false; scatS = 25;
                    case 1
                        j_col = spcl_col; FillF = true;
                        if spcl_col == zeros(size(spcl_col))
                            scatS = 25; 
                        else
                            scatS = 75;
                        end
                end
                switch FillF
                    case 1
                        scatter(foot(j,2), -foot(j,1), scatS, j_col, 'filled', 'o');                     % estimated leg position
                    case 0
                        scatter(foot(j,2), -foot(j,1), scatS, j_col, 'o');
                end
                h{c} = quiver(tlegs(j,2), -tlegs(j,1), tlegs(j,4), -tlegs(j,3),...
                    'LineWidth', 4.0, 'Color', j_col, 'AutoScale', 'off', 'ShowArrowHead', 'off'); c=c+1;
                h{c} = quiver(tlegs0(j,2), -tlegs0(j,1), tlegs0(j,4), -tlegs0(j,3),...
                    'LineStyle', '-',...
                    'LineWidth', 1.2, 'Color', j_col, 'AutoScale', 'off', 'ShowArrowHead', 'off'); c=c+1;           
                h{c} = plot(tfootS{j}(2, :), -tfootS{j}(1, :), '--', 'LineWidth', 1.2, 'Color', j_col); c=c+1;
            end

    end                                                                                                 %%%%%%% plotting order in HAMR format

end