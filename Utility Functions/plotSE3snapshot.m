% This function plots the snapshot of the given SE(3) system kinematics-- needs a bounding box formulation, 2DOF hip frame, and leg frame functions. Follow the t_SE3out.
% steps in "HAMR6_SE3.mlx" section 1 to generate compatible kinematics for your system.
function plotSE3snapshot(ax, traj, kin, fno)
    
    % make sure the frame number is not an erroneous input
    if nargin > 2
        if fno > numel(traj.tH3_e__b)
            error('ERROR! The frame for the snapshot can''t be higher than the number of frames in the trajectory.');
        end
    elseif nargin == 2
        fno = 1;
    else
        error(['ERROR! A current axis object and trajectory struct information is needed to plot the SE(3) configuration. Refer to "HAMR6_SE3.mlx" to generate' ...
            ' compatible kinematics']);
    end

    % Unpack the trajectory
    tH3_e__b = traj.tH3_e__b(fno); taxH3_e__b = traj.taxH3_e__b(fno); % body and axes
    tH3_e__ib = traj.tH3_e__ib(:, fno); taxH3_e__ib = traj.taxH3_e__ib(:, fno); % hip and axes
    tH3_e__icT = traj.tH3_e__icT(:, fno); % top box
    tH3_e__icB = traj.tH3_e__icB(:, fno); % bot box
    tH3_icB__icT = traj.tH3_icB__icT(:, fno); % bot 2 top box
    tH3_ib__i = traj.tH3_ib__i(:, fno); % hip to leg
    tH3_e__i = traj.tH3_e__i(:, fno); taxH3_e__i = traj.taxH3_e__i(:, fno); 
    tH3_e__i_swing = traj.tH3_e__i_swing(:, fno);
    tH3_e__i_lift = traj.tH3_e__i_lift(:, fno);

    % Unpack bounding values
    hip_bound = kin.hip_bound;
    bodywoleg_bound = kin.bodywoleg_bound;

    
    % Start plotting
    

end