function r_dot = generateNoslipVelocity(dpsi_ij, input, robot_params, gait_type)
%GENERATENOSLIPTRAJECTORY given the two-beat gait type, compute the noslip shape velocities for a stance phase-sharing, leg pair.
%   
%   Inputs: The no-slip shape velocity direction, 'dpsi_ij', time vector and pure sine fit params, 'input', physical parameters of the robot, 
%           'robot_params', and a character array describing the type of gait, 'gait_type'.
%   
%   Outputs: 'r_dot' the corresponding shape-velocity trajectory of the gait obtained by projecting on the no-slip basis
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
    aa = robot_params{1}; ll = robot_params{2};
    T = input{1};
    if numel(input) < 3
        r_params = input{2};
        flag = 0; 
        r = cell(size(r_params)); r_dot = r;
    else
        switch input{3}
            case 'Sine'
                r_params = input{2};
                r = cell(size(r_params)); r_dot = r;
                flag = 0;
            case 'Traj'
                re = input{2}{1}; re_dot = input{2}{1};
                r = cell(size(re)); r_dot = cell(size(re_dot));
                flag = 1;
            otherwise
                error('ERROR! Only supports "Sine" and "Traj" so far.');
        end
    end

    % Compute the no-slip trajectory for each two beat input gait from the current shape velocity trajectory
    for iter = 1:size(C, 1)
        % get the leg pair for the current contact state, the initial conditions for the ode integration, and the current shape vel constraint
        i = C(iter, 1); j = C(iter, 2); muli = mul2case1(i); mulj = mul2case1(j);
        dpsiNow = dpsi_ij{i == S(:, 1) & j == S(:, 2)};
        % get the projected shape velocities at each time-step
        switch flag
            case 0
                r{i} = muli*genswing_t( T, r_params{i} ); r{i} = r{i}(:)';
                r{j} = mulj*genswing_t( T, r_params{j} ); r{j} = r{j}(:)'; 
                temp_dot = projShapeVel2NoSlipVel(   [muli*genswingrate_t( T, r_params{i} ); mulj*genswingrate_t( T, r_params{j} )],...
                    dpsiNow(  aa, ll, muli*r{i},  mulj*r{j}  )   );
            case 1
                temp_dot = projShapeVel2NoSlipVel(   [muli*re_dot{i}; mulj*re_dot{j}],...
                    dpsiNow(  aa, ll, muli*re{i},  mulj*re{j}  )   );
        end
        r_dot{i} = muli*temp_dot(1, :); % back to HAMR format
        r_dot{j} = mulj*temp_dot(2, :);
    end

end