function [r, r_dot] = generateNoslipTrajectory(dpsi_ij, traj, robot_params, gait_type)
%GENERATENOSLIPTRAJECTORY given the two-beat gait type, compute the noslip shape trajectory for one of the legs in each stance pair.
%   
%   Inputs: The no-slip shape velocity direction, 'dpsi_ij', locomotion kinematics, 'traj', physical parameters of the robot, 'robot_params', and a 
%           character array describing the type of gait, 'gait_type'.
%   
%   Outputs: 'r' a set of shape trajectories that respect the no-slip constraint for each level-2 contact state and 'r_dot' the corresponding shape-velocity
%           trajectory
%   
%   Assumptions: This function assumes only two-beat gaits with level-2 contact states-- also the format of the input data must follow the form used in 
%               "HAMR6_SE3.mlx".
    
    % list of level-2 submanifolds
    S = [1, 2;
         2, 3;
         3, 4;
         4, 1;
         1, 3;
         2, 4];
    
    % define the stance phases in terms of leg indices i and j
    switch gait_type
        case 'trot'
            C = [1, 3;
                 2, 4]; % the two stance phases are defined along rows
        case 'pace'
            C = [2, 3;
                 4, 1];
        case 'bound'
            C = [1, 2;
                 3, 4];
    end

    % Unpack
    aa = robot_params{1}; ll = robot_params{2}; keep_char = robot_params{3};
    t = traj.exp.t; rp = traj.est.r1_params;
    [r, r_dot] = genSine_r_rdot(rp, t); % assign the H1 time series for now
    r_swing = r(1:2:end);
    rp = rp(1:2:end); % just swing sine parameters

    % iterate over each stance phase and adjust the phase of the jth leg to minimize slippage
    temp = r(1:2:end); temp_dot = r_dot(1:2:end);
    for iter = 1:size(C, 1)
        
        i = C(iter, 1); j = C(iter, 2); % get the leg indices
        fidx = S(:, 1) == i & S(:, 2) == j ; % get the function indices
        
        % get the inputs that remain constant and then use that to compute the outputs
        switch keep_char
            case 'i'
                in = {rp{i}, t, dpsi_ij{fidx}, r_swing{j}(1), aa, ll}; idx = i; idx_noslip = j;
            case 'j'
                in = {rp{j}, t, dpsi_ij{fidx}, r_swing{i}(1), aa, ll}; idx = j; idx_noslip = i;
        end
        if idx == 2 || idx == 3
            in{1}(1) = -in{1}(1); in{1}(end) = -in{1}(end); % negate the sine params for conversion to case 1
        end
        if idx_noslip == 2 || idx_noslip == 3
            in{4} = -in{4}; % negate the initial condition
        end
        
        % compute the corresponding no-slip satisfying r and r_dot
        [temp{idx_noslip}, temp_dot{idx_noslip}] = Path2.compute_noslip_trajectory(in);
        if idx_noslip == 2 || idx_noslip == 3
            temp{idx_noslip} = -temp{idx_noslip}; temp_dot{idx_noslip} = -temp_dot{idx_noslip}; % bring it back to HAMR format
        end
        
    end
    r(1:2:end) = temp; r_dot(1:2:end) = temp_dot;

end

