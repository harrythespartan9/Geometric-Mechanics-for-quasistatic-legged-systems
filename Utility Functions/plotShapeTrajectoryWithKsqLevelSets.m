function f = plotShapeTrajectoryWithKsqLevelSets(finp, input, robot_params, gait_type, idxT)
%GENERATENOSLIPTRAJECTORY given the two-beat gait type, compute the noslip shape trajectory for one of the legs in each stance pair.
%   
%   Inputs: The squared inter-leg distance scalar field that is a function of the current stancing legs, 'ksq_ij', inputs-- time vector and pure sine fit params
%           , 'input', physical parameters of the robot, 'robot_params', a character array describing the type of gait, 'gait_type', and index corresponding to
%           one time-period, 'idxT'.
%   
%   Outputs: 'f' the figure handle to be used to save the figure
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
    aa = robot_params{1}; ll = robot_params{2}; % robot case 1 kinematics params
    r = input{1};                               % swing trajectories
    r_dot = input{2};                           % swing velocity trajectories
    c = input{3};                               % contact trajectories
    rlim = input{6};                            % plotting limits
    ksq_ij = finp{1};                           % functions to plot ksq contours and dpsi noslip
    % dpsi_ijV = finp{2};
    
    % compute the scalar field as a function of the reduced shape space
    discNum = 100;
    riS = linspace(-rlim, rlim, discNum); rjS = riS; [riS, rjS] = meshgrid(riS, rjS);
    % skipIdx = (1:10:discNum); % plot only every 10th vector

    % iterate over the contact states and plot the shape-space trajectories
    f = figure('units','pixels','position',480*[0 0 2 1],'Color','w');
    set(f, 'Visible', 'on');
    tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'tight');
    for iter = 1:size(C, 1)
        % compute
        i = C(iter, 1); j = C(iter, 2); % each contact index
        muli = mul2case1(i); mulj = mul2case1(j); % corresponding multiplier to go from HAMR to case 1 format
        ksqS = ksq_ij{i == S(:, 1) & j == S(:, 2)}(aa, ll, riS, rjS); % get the scalar field
        % plot
        ax = nexttile;
        contour(ax, riS, rjS, ksqS, 100, 'LineWidth', 0.2, 'FaceAlpha', 0.25); colormap(ax, input{5}{4});
        hold on; axis equal square; set(ax,'TickLabelInterpreter','latex');
        plotShape2(ax, {muli, r{i}, r_dot{i}, c{i}, mulj, r{j}, r_dot{j}, c{j}, idxT},...
            {['$$\alpha_' num2str(i) '$$'], ['$$\alpha_' num2str(j) '$$']}, input{5});
    end

end

%% HELPER FUNCTIONS

function plotShape2(ax, in, in_str, in_col_sty_Fs_lW) % legend_str
%PLOTSE2TIMESERIES_SNAPSHOT this function plots a single shape space trajectory data
    
    % unpack
    mulx = in{1}; muly = in{5};
    x = in{2}; y = in{6}; % data 
    x_dot = in{3}; y_dot = in{7};
    cx = in{4}; cy = in{8};
    idxT = in{9}; numT = find(idxT, 1, "last"); % indices and last index belonging to the first time period
    col = in_col_sty_Fs_lW{1}; 
    sty = in_col_sty_Fs_lW{2}; 
    lW = in_col_sty_Fs_lW{3};
    % plotting style
    switch numel(in_col_sty_Fs_lW)
        case 4
            lineF = num2cell(true*ones(size(1, numel(lW))));
            vecF = num2cell(false*ones(size(1, numel(lW)))); % the velocities are not plotted by default
            plot_str = mat2cell(repmat("forall_t", 1, numel(lW)), 1, numel(lW)); % the whole trajectory is plotted by default
        otherwise
            lineF = in_col_sty_Fs_lW{5};
            vecF = in_col_sty_Fs_lW{6}; 
            plot_str = in_col_sty_Fs_lW{7};
    end

    % plotting data
    num = numel(x{1}); 
    skipV = 0.60;  % skipping (skipV)% of points when plotting the shape velocities

    % plot
    fS = 10;
    for i = 1:numel(x)
        % compute the current component plotting data %%%%%%%%
        idxC = cx{i} & cy{i}; % indices where the current contact state is active
        idxV = zeros(1, num); idxV( round(linspace( 1, num, (1-skipV)*num )) ) = 1; 
        idxV = idxV & idxC; % active sub-indicies where contact state is active
        % plotting %%%%%%%%
        if lineF{i}
            scatter(ax, mulx*x{i}(1), muly*y{i}(1), 100, col(i, :), 'filled', 'Marker', 'o'); % scatter a circle at the initial condition
            StyTraj = {":", ":"};
            switch plot_str{i}
                case "forall_t"
                    % plot the trajectories
                    plotModeSwitchingShapeTrajectory(ax, mulx*x{i}, muly*y{i}, idxC, 0.5, StyTraj, col(i, :));
                case "T"
                    plotModeSwitchingShapeTrajectory(ax, mulx*x{i}(1:numT), muly*y{i}(1:numT), idxC(1:numT), lW{i}, StyTraj, col(i, :));
                    idxV = idxV & idxT;
                otherwise
                    error('ERROR! Only "forall_t" and "T" cases are supported.');
            end
        end
        if vecF{i}
            dotNow = [x_dot{i}(idxV); y_dot{i}(idxV)]; dotNow = dotNow./repmat(vecnorm(dotNow), 2, 1);
            quiver(ax, mulx*x{i}(idxV), muly*y{i}(idxV), mulx*dotNow(1, :), muly*dotNow(2, :),...
                'LineStyle', sty{i}, 'LineWidth', lW{i}, 'Color', col(i, :), 'AutoScaleFactor', 0.35);
        end
    end
    xlabel(ax, in_str{1}, 'FontSize', fS, 'Interpreter', 'latex'); ax.FontSize = fS;
    ylabel(ax, in_str{2}, 'FontSize', fS, 'Interpreter', 'latex');

end

