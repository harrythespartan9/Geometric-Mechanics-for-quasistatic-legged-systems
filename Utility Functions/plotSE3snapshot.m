% This function plots the snapshot of the given SE(3) system kinematics-- needs a bounding box formulation, 2DOF hip frame, and leg frame functions. Follow the 
% steps in "HAMR6_SE3.mlx" section 1 to generate compatible kinematics for your system.
function h = plotSE3snapshot(ax, traj, v, fno)
    
    % make sure the frame number is not an erroneous input
    if nargin > 3
        if isempty(fno)
            fno = 1;
        else
            if fno > numel(traj.tH3_e__b)
                error('ERROR! The frame for the snapshot can''t be higher than the number of frames in the trajectory.');
            end
        end
    elseif nargin == 3
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
    tfootL = traj.tfootL(:, fno);
    ht3_e__i_exp = traj.exp.ht3_e__i;
    plotlim = [min(traj.plotlim(1, :)), max(traj.plotlim(2, :)),...
            min(traj.plotlim(3, :)), max(traj.plotlim(4, :)),...
            min(traj.plotlim(5, :)), max(traj.plotlim(6, :))] + 0.07*traj.bl*repmat([-1, 1], [1, 3]);
    gr_col = [100, 100, 100]/255; % gray color used for robot body, etc
    if isfield(traj, 'exp')
        spcl_col = traj.exp.col{fno} ;
        foot = [tlegs(:,1), tlegs(:,2), tlegs(:,3)] + [tlegs(:,4), tlegs(:,5), tlegs(:,6)];
        plotlim = [min(traj.plotlim(1, :)), max(traj.plotlim(2, :)),...
                min(traj.plotlim(3, :)), max(traj.plotlim(4, :)),...
                min(traj.plotlim(5, :)), max(traj.plotlim(6, :))] + 0.07*traj.bl*repmat([-1, 1], [1, 3]);
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
            quiver3(tframes(:,1), tframes(:,2), tframes(:,3), tframes(:,4), tframes(:,5), tframes(:,6),...
                'LineWidth', 1.2, 'Color', gr_col, 'AutoScale', 'off');                                             % all the frames-- body, hip, and legs
            hold(ax, 'on'); grid(ax, 'off'); axis(ax, 'square', 'equal', 'padded', plotlim); xlabel(ax, 'X'); ylabel(ax, 'Y'); 
            zlabel(ax, 'Z'); xticklabels(ax, ''); yticklabels(ax, ''); zticklabels(ax, '');
            view(v); % set the camera angle
            quiver3(tboxes(:,1), tboxes(:,2), tboxes(:,3), tboxes(:,4), tboxes(:,5), tboxes(:,6),...
                'LineWidth', 3.0, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                     % all the bounding boxes -- except feet
            for j = 1:4
                quiver3(tlegs(j,1), tlegs(j,2), tlegs(j,3), tlegs(j,4), tlegs(j,5), tlegs(j,6),...
                    'LineWidth', 4.0, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                 % all legs from the hip
                quiver3(tlegs0(j,1), tlegs0(j,2), tlegs0(j,3), tlegs0(j,4), tlegs0(j,5), tlegs0(j,6),...
                    'LineStyle', '-',...
                    'LineWidth', 1.2, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                 % all lges origin/eq position from the hip
                plot3(tfootS{j}(:, 1), tfootS{j}(:, 2), tfootS{j}(:, 3), '--', 'LineWidth', 1.2, 'Color', gr_col); 
                plot3(tfootL{j}(:, 1), tfootL{j}(:, 2), tfootL{j}(:, 3), ':', 'LineWidth', 1.2, 'Color', gr_col);   % swing and lift trajectories of all legs
            end

        case 1 %%%%%%%%%%%%%%%% animation case
            
            c = 1;
            h{c} = quiver3(tframes(:,1), tframes(:,2), tframes(:,3), tframes(:,4), tframes(:,5), tframes(:,6),...
                'LineWidth', 1.2, 'Color', gr_col, 'AutoScale', 'off'); c=c+1;                                           
            hold(ax, 'on'); grid(ax, 'off'); axis(ax, 'square', 'equal', 'padded', plotlim); xlabel(ax, 'x'); ylabel(ax, 'y'); 
            zlabel(ax, 'z'); xticklabels(ax, ''); yticklabels(ax, ''); zticklabels(ax, '');
            view(v);                                                                                                % set the camera angle
            h{c} = quiver3(tboxes(:,1), tboxes(:,2), tboxes(:,3), tboxes(:,4), tboxes(:,5), tboxes(:,6),...
                'LineWidth', 3.0, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off'); c=c+1;
            switch FillF
                case 1
                    scatter3(tbody(1), tbody(2), tbody(3), scatS, spcl_col, 'filled', 'o');                         % COM position
                case 0
                    scatter3(tbody(1), tbody(2), tbody(3), scatS, spcl_col, 'o');
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
                        scatter3(foot(j,1), foot(j,2), foot(j,3), scatS, j_col, 'filled', 'o');                     % estimated leg position
                        scatter3(ht3_e__i_exp{j}(1, fno), ht3_e__i_exp{j}(2, fno),...
                            ht3_e__i_exp{j}(3, fno), scatS, j_col, 'filled', '+');                                  % actual leg position
                    case 0
                        scatter3(foot(j,1), foot(j,2), foot(j,3), scatS, j_col, 'o');
                        scatter3(ht3_e__i_exp{j}(1, fno), ht3_e__i_exp{j}(2, fno),...
                            ht3_e__i_exp{j}(3, fno), scatS, j_col, '+');
                end
                h{c} = quiver3(tlegs(j,1), tlegs(j,2), tlegs(j,3), tlegs(j,4), tlegs(j,5), tlegs(j,6),...
                    'LineWidth', 4.0, 'Color', j_col, 'AutoScale', 'off', 'ShowArrowHead', 'off'); c=c+1;
                h{c} = quiver3(tlegs0(j,1), tlegs0(j,2), tlegs0(j,3), tlegs0(j,4), tlegs0(j,5), tlegs0(j,6),...
                    'LineStyle', '-',...
                    'LineWidth', 1.2, 'Color', j_col, 'AutoScale', 'off', 'ShowArrowHead', 'off'); c=c+1;           
                h{c} = plot3(tfootS{j}(:, 1), tfootS{j}(:, 2), tfootS{j}(:, 3), '--', 'LineWidth', 1.2, 'Color', j_col); c=c+1;
                h{c} = plot3(tfootL{j}(:, 1), tfootL{j}(:, 2), tfootL{j}(:, 3), ':', 'LineWidth', 1.2, 'Color', j_col); c=c+1;
            end

    end

end