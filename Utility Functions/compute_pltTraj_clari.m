function pltTraj = compute_pltTraj_clari(kin, traj)
%COMPUTE_PLTTRAJ compute the plotting trajectory for an SE(2) snapshot function
%   Given the configuration trajectory of a planar shape-changing, quadrupedal system under the noslip constraint (divides into shape or actuation space and 
%   SE(2) position space), this function computes the things to be plotted.
%   Inputs: "kin"-- functions to compute things needed to plot and "traj"-- configuration trajectory of the SE(2) system.
%   Output: "pltTraj"-- plotting trajectory needed to plot the system snapshot

    % unpack everything
    x = traj{1}(1,:); 
    y = traj{1}(1,:); 
    theta = traj{1}(3,:);                                                                               % SE(2) trajectories
    alpha_b = traj{1}(4,:);                                                                             % body shape (sprawl) trajectory
    alpha = traj{1}(5:end,:);                                                                           % shape trajectory
    col_t = traj{2};                                                                                    % color trajectory
    phi_tau = traj{3};                                                                                  % submanifold index trajectory
    pbq = traj{5};                                                                                      % body configuration snapshot points
    dnum = traj{6};                                                                                     % trajectory discretization number
    a = kin.params.aa; l = kin.params.ll; bl = kin.params.bl;                                           % body params
    alpha_1 = alpha(1, :); alpha_2 = alpha(2, :); alpha_3 = alpha(3, :); alpha_4 = alpha(4, :);         % splitting into individual leg swing trajectories

    % level-2 submanifold ordering
    S = [1, 2;
         2, 3;
         3, 4;
         4, 1;
         1, 3;
         2, 4];

    % initialize and compute the plotting trajectory
    body_outline_x_t = kin.functions.body_outline_x_f(a, l, x, y, theta, alpha_b);
    body_outline_y_t = kin.functions.body_outline_y_f(a, l, x, y, theta, alpha_b);
    shape_outline_x_t = kin.functions.shape_outline_x_f(a, l, x, y, theta, alpha_b);
    shape_outline_y_t = kin.functions.shape_outline_y_f(a, l, x, y, theta, alpha_b);
    frame_x_t = kin.functions.frame_x_f(a, l, x, y, theta);
    frame_y_t = kin.functions.frame_y_f(a, l, x, y, theta);
    frame_u_t = kin.functions.frame_u_f(a, l, x, y, theta);
    frame_v_t = kin.functions.frame_v_f(a, l, x, y, theta);
    legs_x_t = kin.functions.legs_x_f(a, l, x, y, theta, alpha_b, ...
                                                         alpha_1, alpha_2, alpha_3, alpha_4);
    legs_y_t = kin.functions.legs_y_f(a, l, x, y, theta, alpha_b, ...
                                                         alpha_1, alpha_2, alpha_3, alpha_4);
    legsQ_x_t = kin.functions.legsQ_x_f(a, l, x, y, theta, alpha_b, ...
                                                           alpha_1, alpha_2, alpha_3, alpha_4);
    legsQ_y_t = kin.functions.legsQ_y_f(a, l, x, y, theta, alpha_b, ...
                                                           alpha_1, alpha_2, alpha_3, alpha_4);
    legsQ_u_t = kin.functions.legsQ_u_f(a, l, x, y, theta, alpha_b, ...
                                                           alpha_1, alpha_2, alpha_3, alpha_4);
    legsQ_v_t = kin.functions.legsQ_v_f(a, l, x, y, theta, alpha_b, ...
                                                           alpha_1, alpha_2, alpha_3, alpha_4);
    legsQ0_x_t = kin.functions.legsQ_x_f(a, l, x, y, theta, alpha_b, ...
                                                           zeros(size(alpha_1)), zeros(size(alpha_2)), zeros(size(alpha_3)), zeros(size(alpha_4)));
    legsQ0_y_t = kin.functions.legsQ_y_f(a, l, x, y, theta, alpha_b, ...
                                                           zeros(size(alpha_1)), zeros(size(alpha_2)), zeros(size(alpha_3)), zeros(size(alpha_4)));
    legsQ0_u_t = kin.functions.legsQ_u_f(a, l, x, y, theta, alpha_b, ...
                                                           zeros(size(alpha_1)), zeros(size(alpha_2)), zeros(size(alpha_3)), zeros(size(alpha_4)));
    legsQ0_v_t = kin.functions.legsQ_v_f(a, l, x, y, theta, alpha_b, ...
                                                           zeros(size(alpha_1)), zeros(size(alpha_2)), zeros(size(alpha_3)), zeros(size(alpha_4)));

    % initialize and find the squared inter-leg distance vectors-- for each level-2 submanifold
    k_x_t = cell(6, 1); k_y_t = k_x_t;
    for i = 1:size(S, 1)
        k_x_t{i} = [legs_x_t(S(i,1), :); legs_x_t(S(i,2), :)];
        k_y_t{i} = [legs_y_t(S(i,1), :); legs_y_t(S(i,2), :)];
    end

    % compute the animation limits
    thresh = 1.25; % 1.25*body_length boundary
    anim_lim = [min(x)-thresh*bl, max(x)+thresh*bl, min(y)-thresh*bl, max(y)+thresh*bl];

    % pack everything and send it back
    pltTraj = [];
    pltTraj.dnum = dnum;
    pltTraj.anim_lim = anim_lim;                    % all snapshot/animation related data
    pltTraj.body_outline_x_t = body_outline_x_t;
    pltTraj.body_outline_y_t = body_outline_y_t;
    pltTraj.shape_outline_x_t = shape_outline_x_t;
    pltTraj.shape_outline_y_t = shape_outline_y_t;
    pltTraj.frame_x_t = frame_x_t;
    pltTraj.frame_y_t = frame_y_t;
    pltTraj.frame_u_t = frame_u_t;
    pltTraj.frame_v_t = frame_v_t;
    pltTraj.legs_x_t = legs_x_t;
    pltTraj.body_outline_x_t = body_outline_x_t;
    pltTraj.legs_y_t = legs_y_t;
    pltTraj.legsQ_x_t = legsQ_x_t;
    pltTraj.legsQ_y_t = legsQ_y_t;
    pltTraj.legsQ_u_t = legsQ_u_t;
    pltTraj.legsQ_v_t = legsQ_v_t;          % current leg position in quiver format
    pltTraj.legsQ0_x_t = legsQ0_x_t;        % leg origin position in quiver format
    pltTraj.legsQ0_y_t = legsQ0_y_t;
    pltTraj.legsQ0_u_t = legsQ0_u_t;
    pltTraj.legsQ0_v_t = legsQ0_v_t;
    pltTraj.alpha_body_nom = kin.params.ab0; % the nominal (rest) body sprawl angle
    pltTraj.alpha_body_t = alpha_b; % add the body sprawl trajectory for plotting the body trajectory
    pltTraj.k_x_t = k_x_t;
    pltTraj.k_y_t = k_y_t;
    pltTraj.pbq = pbq; % append the body configuration plot points (binary trajectory-- 1 => plot config)
    pltTraj.x = x;
    pltTraj.y = y; % append the translational trajectory
    pltTraj.col_t = col_t; % append the color trajectory
    pltTraj.phi_tau = phi_tau; % append the submanifold index trajectory
    pltTraj.S = S; % append the submanifold list


end