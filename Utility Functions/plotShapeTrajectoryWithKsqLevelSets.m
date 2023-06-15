function f = plotShapeTrajectoryWithKsqLevelSets(finp, input, robot_params, gait_type)
%GENERATENOSLIPTRAJECTORY given the two-beat gait type, compute the noslip shape trajectory for one of the legs in each stance pair.
%   
%   Inputs: The squared inter-leg distance scalar field that is a function of the current stancing legs, 'ksq_ij', inputs-- time vector and pure sine fit params
%           , 'input', physical parameters of the robot, 'robot_params', and a character array describing the type of gait, 'gait_type'.
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
    rlim = input{5};                            % plotting limits
    ksq_ij = finp{1};                           % functions to plot ksq contours and dpsi noslip
    % dpsi_ijV = finp{2};
    
    % compute the scalar field as a function of the reduced shape space
    discNum = 100;
    riS = linspace(-rlim, rlim, discNum); rjS = riS; [riS, rjS] = meshgrid(riS, rjS);
    % skipIdx = (1:10:discNum); % plot only every 10th vector

    % iterate over the contact states and plot the shape-space trajectories
    f = figure('units','pixels','position',480*[0 0 2 1],'Color','w');
    tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'tight');
    for iter = 1:size(C, 1)
        % compute
        i = C(iter, 1); j = C(iter, 2); % each contact index
        muli = mul2case1(i); mulj = mul2case1(j); % corresponding multiplier to go from HAMR to case 1 format
        ksqS = ksq_ij{i == S(:, 1) & j == S(:, 2)}(aa, ll, riS, rjS); % get the scalar field
        % temp = dpsi_ijV{i == S(:, 1) & j == S(:, 2)}(aa, ll, riS(:)', rjS(:)'); % get the dphi direction
        % uS = reshape(temp(1, :), size(riS)); vS = reshape(temp(2, :), size(riS));
        % plot
        ax = nexttile;
%         contour(ax, riS, rjS, ksqS, 10, 'EdgeColor', [189, 189, 189]/255, 'LineStyle', '--');
        contourf(ax, riS, rjS, ksqS, 100, 'LineStyle', 'none', 'FaceAlpha', 0.65); colormap(ax, input{4}{4});
        hold on; axis equal square; set(ax,'TickLabelInterpreter','latex');
        % quiver( ax, riS(skipIdx, skipIdx), rjS(skipIdx, skipIdx), uS(skipIdx, skipIdx), vS(skipIdx, skipIdx), 'Color', [189, 189, 189]/255 );
        plotShape2(ax, {muli, r{i}, r_dot{i}, mulj, r{j}, r_dot{j}}, {['$$\alpha_' num2str(i) '$$'], ['$$\alpha_' num2str(j) '$$']}, input{4}); % , input{3}
    end

end

%% HELPER FUNCTIONS

function plotShape2(ax, in, in_str, in_col_sty_lW, ~) % legend_str
%PLOTSE2TIMESERIES_SNAPSHOT this function plots a single shape space trajectory data
    
    % unpack
    mulx = in{1}; x = in{2}; muly = in{4}; y = in{5}; % data % y_dot = in{6}; % x_dot = in{3}; 
    col = in_col_sty_lW{1}; sty = in_col_sty_lW{2}; lW = in_col_sty_lW{3}; % plotting style

    % plot
    fS = 10;
    for i = 1:numel(x)
        plot(ax, mulx*x{i}, muly*y{i}, sty{i}, 'LineWidth', lW{i}, 'Color', col(i, :));
%         quiver(ax, mulx*x{i}, muly*y{i}, mulx*x_dot{i}, muly*y_dot{i}, 'LineStyle', sty{i}, 'LineWidth', lW{i}, 'Color', col(i, :));
    end
    xlabel(ax, in_str{1}, 'FontSize', fS, 'Interpreter', 'latex'); ax.FontSize = fS;
    ylabel(ax, in_str{2}, 'FontSize', fS, 'Interpreter', 'latex');
    % if nargin > 4 % if the legend_str is provided
    %     legend(ax, legend_str, 'Location', 'bestoutside', 'FontSize', fS);
    % end

end

