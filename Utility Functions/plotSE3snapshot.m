% This function plots the snapshot of the given SE(3) system kinematics-- needs a bounding box formulation, 2DOF hip frame, and leg frame functions. Follow the t_SE3out.
% steps in "HAMR6_SE3.mlx" section 1 to generate compatible kinematics for your system.
function plotSE3snapshot(ax, traj, fno)
    
    % make sure the frame number is not an erroneous input
    if nargin > 3
        if fno > numel(traj.tH3_e__b)
            error('ERROR! The frame for the snapshot can''t be higher than the number of frames in the trajectory.');
        end
    elseif nargin == 3
        fno = 1;
    else
        error(['ERROR! A current axis object (obtain using the "gca" command or "gcf.CurrentAxes" command), trajectory struct, and system''s kinematic ' ...
            'information is needed to plot the SE(3) configuration. Refer to "HAMR6_SE3.mlx" as a tutorial to generate compatible kinematics.']);
    end

    % Unpack the plotting structs and defining vars I need
    tframes = traj.tframes{fno};
    tboxes = traj.tboxes{fno};
    tlegs = traj.tlegs{fno}; tlegs0 = traj.tlegs0{fno};
    tfootS = traj.tfootS{:, fno};
    tfootL = traj.tfootL{:, fno};
    gr_col = [100, 100, 100]/255; % gray color used for robot body, etc
    
    % Compute plotting limits
    up = max(abs([tboxes(:,1:3); tlegs(:,1:3)]));
    plot_lim = 1.1*[-up(1), up(1), -up(2), up(2), -up(3), up(3)]; % scale the limits by 10%

    % plot the required configuration
    quiver3(tframes(:,1), tframes(:,2), tframes(:,3), tframes(:,4), tframes(:,5), tframes(:,6),...
        'LineWidth', 2, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                    % all the frames-- body, hip, and legs
    hold(ax, 'on'); axis(ax, 'square', 'equal', plot_lim); xlabel('X'); ylabel('Y'); zlabel('Z');
    quiver3(tboxes(:,1), tboxes(:,2), tboxes(:,3), tboxes(:,4), tboxes(:,5), tboxes(:,6),...
        'LineWidth', 1.5, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                  % all the bounding boxes -- except feet
    quiver3(tlegs(:,1), tlegs(:,2), tlegs(:,3), tlegs(:,4), tlegs(:,5), tlegs(:,6),...
        'LineWidth', 1.5, 'Color', gr_col, 'AutoScale', 'off', 'ShowArrowHead', 'off');                  % all legs from the hip

    
    

end