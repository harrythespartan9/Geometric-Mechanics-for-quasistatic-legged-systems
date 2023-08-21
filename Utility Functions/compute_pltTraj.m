function pltTraj = compute_pltTraj(pltkin, traj, params)
%COMPUTE_PLTTRAJ compute the plotting trajectory for an SE(2) snapshot function
%   Given the configuration trajectory of a planar locomoting, quadrupedal system under the noslip constraint (divides into shape or actuation space and SE(2) 
%   position space), this function computes the things to be plotted.
%   Inputs: pltkin-- functions to compute things, traj-- configuration trajectory of the SE(2) system, and params-- body params.
%   Output: pltTraj-- plotting trajectory needed to plot the system snapshot

    % unpack everything
    x = traj{1}(1, :); 
    y = traj{1}(2, :); 
    theta = traj{1}(3, :);  % SE(2) traj
    alpha = cell(4, 1);
    for i = 1:4
        alpha{i} = traj{2}(i, :); % shape trajectory
    end
    dnum = traj{3};
    a = params{1}; l = params{2}; bl = params{3};

    % level-2 submanifold ordering
    S = [1, 2;
         2, 3;
         3, 4;
         4, 1;
         1, 3;
         2, 4];

    % initialize and compute the plotting trajectory
    leg__x = cell(4, 1); leg__y = cell(4, 1);
    legtip__x = cell(4, 1); legtip__y = cell(4, 1);
    O_leg__x = cell(4, 1); O_leg__y = cell(4, 1);
    body_link__x = cell(4, 1); body_link__y = cell(4, 1);
    bodyLinkStr = {'topright2topleft_', 'topleft2botleft_', 'botleft2botright_', 'botright2topright_'}; % need to find a better way of doing this for CLARI
    for i = 1:4
        % compute the leg vector
        leg__x{i} = pltkin.(['legbase' num2str(i) '_leg' num2str(i) '_x'])(a, l, alpha{i}, x, y, theta); 
                            leg__x{i} = [leg__x{i}(1:dnum); leg__x{i}(dnum+1:end)];
        leg__y{i} = pltkin.(['legbase' num2str(i) '_leg' num2str(i) '_y'])(a, l, alpha{i}, x, y, theta); 
                            leg__y{i} = [leg__y{i}(1:dnum); leg__y{i}(dnum+1:end)];
        % compute the leg tips (circs will be scatted when the contact state is active)
        legtip__x{i} = pltkin.(['leg' num2str(i) '_x'])(a, l, alpha{i}, x, y, theta);
        legtip__y{i} = pltkin.(['leg' num2str(i) '_y'])(a, l, alpha{i}, x, y, theta);
        % leg origin pose (helps visualize the current leg angle)
        O_leg__x{i} = pltkin.(['legbase' num2str(i) '_leg' num2str(i) '_x'])(a, l, zeros(size(alpha{i})), x, y, theta);
        O_leg__y{i} = pltkin.(['legbase' num2str(i) '_leg' num2str(i) '_y'])(a, l, zeros(size(alpha{i})), x, y, theta);
        % body links
        body_link__x{i} = pltkin.([bodyLinkStr{i} '_x'])(l, x, y, theta); body_link__x{i} = [body_link__x{i}(1:dnum); body_link__x{i}(dnum+1:end)];
        body_link__y{i} = pltkin.([bodyLinkStr{i} '_y'])(l, x, y, theta); body_link__y{i} = [body_link__y{i}(1:dnum); body_link__y{i}(dnum+1:end)];
    end

    % body frame location and frame quivers
    body__x = pltkin.body_x(x, y, theta); 
    body__y = pltkin.body_y(x, y, theta);
    bodyf__x = pltkin.bodyf_x(x, y, theta); 
    bodyf__y = pltkin.bodyf_y(x, y, theta);

    % find the squared inter-leg distance vectors-- for each level-2 submanifold
    for i = 1:size(S, 1)
        ksq__x{i} = pltkin.(['k_leg' S(i, 1) '_leg' S(i, 2) '_x'])(a, l, alpha{S(i, 1)}, alpha{S(i, 2)}, x, y, theta); 
                            ksq__x{i} = [ksq__x{i}(1:dnum); ksq__x{i}(dnum+1:end)];
        ksq__y{i} = pltkin.(['k_leg' S(i, 1) '_leg' S(i, 2) '_x'])(a, l, alpha{S(i, 1)}, alpha{S(i, 2)}, x, y, theta); 
                            ksq__y{i} = [ksq__y{i}(1:dnum); ksq__y{i}(dnum+1:end)];
    end

    % compute the animation limits
    thresh = 1.25; % 1.25*body_length boundary
    anim_lim = [min(x)-thresh*bl, max(x)+thresh*bl, min(y)-thresh*bl, max(y)+thresh*bl];

    % pack everything and send it back
    pltTraj = [];
    pltTraj.anim_lim = anim_lim;
    pltTraj.leg__x = leg__x;
    pltTraj.leg__y = leg__y;
    pltTraj.legtip__x = legtip__x;
    pltTraj.legtip__y = legtip__y;
    pltTraj.O_leg__x = O_leg__x;
    pltTraj.O_leg__y = O_leg__y;
    pltTraj.body_link__x = body_link__x;
    pltTraj.body_link__y = body_link__y;
    pltTraj.body__x = body__x;
    pltTraj.body__y = body__y;
    pltTraj.bodyf__x = bodyf__x;
    pltTraj.bodyf__y = bodyf__y;
    pltTraj.ksq__x = ksq__x;
    pltTraj.ksq__y = ksq__y;

end

