function [r, r_dot] = generateNoslipTrajectory(dpsi_ij, input, robot_params, gait_type)
%GENERATENOSLIPTRAJECTORY given the two-beat gait type, compute the noslip shape trajectory for one of the legs in each stance pair.
%   
%   Inputs: The no-slip shape velocity direction, 'dpsi_ij', inputs-- time vector and pure sine fit params, 'input', physical parameters of the robot, 
%           'robot_params', and a character array describing the type of gait, 'gait_type'.
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
    aa = robot_params{1}; ll = robot_params{2};
    T = input{1}; T_0 = T(1); % starting point
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
        if ~flag
            alpha_0 = [muli*getNowSignal({0, r_params{i}}, @genswing_t);  mulj*getNowSignal({0, r_params{j}}, @genswing_t)];
            % compute the no-slip trajectory for alpjha_ij
            [~, temp] = ode45(    @(t,x) projShapeVel2NoSlipVel(   [ muli*getNowSignal({(t+T_0), r_params{i}}, @genswingrate_t); ...
                mulj*getNowSignal({(t+T_0), r_params{j}}, @genswingrate_t) ],...
                dpsiNow(  aa, ll, x(1),  x(2)  )   ),...
                T - T_0, alpha_0, odeset('RelTol',1e-5)    );
            % , odeset('RelTol',1e-5)
            %%%%%%%%%%%%%%%%% [muli*genswingrate_t( (t+t_0), r_params{i} ); mulj*genswingrate_t( (t+t_0), r_params{j} )]
        else
            alpha_0 = [muli*re{i}(1);  mulj*re{j}(1)];
            [~, temp] = ode45(    @(t,x) projShapeVel2NoSlipVel(   [ muli*getNowSignal( {(t+T_0), T, re_dot{i}} ); ...
                mulj*getNowSignal( {(t+T_0), T, re_dot{j}} ) ],...
                dpsiNow(  aa, ll, x(1),  x(2)  )   ),...
                T - T_0, alpha_0, odeset('RelTol',1e-5)    );
        end
        %%%%%%%%%%%%%%%%% alpha_0 = [muli*genswing_t(0, r_params{i});  mulj*genswing_t(0, r_params{j})];
        r{i} = muli*temp(:, 1); r{i} = r{i}(:)'; % multiplying again by the muli and mulj because it is a involution and we want them back in the HAMR format
        r{j} = mulj*temp(:, 2); r{j} = r{j}(:)'; % also ensure they are 1xt time-series in each cell
        % get the projected shape velocities at each time-step
        switch flag
            case 0
                temp_dot = projShapeVel2NoSlipVel(   [muli*genswingrate_t( T, r_params{i} ); mulj*genswingrate_t( T, r_params{j} )],...
                    dpsiNow(  aa, ll, muli*r{i},  mulj*r{j}  )   );
            case 1
                temp_dot = projShapeVel2NoSlipVel(   [muli*re_dot{i}; mulj*re_dot{j}],...
                    dpsiNow(  aa, ll, muli*r{i},  mulj*r{j}  )   );
        end
        r_dot{i} = muli*temp_dot(1, :); % back to HAMR format
        r_dot{j} = mulj*temp_dot(2, :);
    end

end

%% UTILITY FUNCTIONS

function out = getNowSignal(in, f)
%GETSWINGORSWINGVEL this function computes the value of the input variable at a certain point in time

    % Unpack and compute the return value
    switch nargin
        case 2
            t_ref = in{1};
            xparams = in{2};
            out = f(t_ref, xparams);
        case 1
            t_ref = in{1};
            t = in{2};
            x = in{3};
            out = interp1( t, x, t_ref );
        otherwise
            error('ERROR! Wrong number of inputs.');
    end

end
