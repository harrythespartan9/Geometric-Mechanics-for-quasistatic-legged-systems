% This function plots the snapshot of the given SE(3) quadrupedal system kinematics-- needs a bounding box formulation, 2DOF hip frame, and leg frame functions. 
% Follow the steps in "HAMR6_SE3.mlx" section 1 to generate compatible kinematics for your system.
% Inputs: r-- shape trajectory; for a 2DOF leg quadruped this would be 1x8 cell array with each each cell being t-vector long
%         b-- body trajectory; since this is SE(3), this would be a collection of body displacements and YPR angles (look up ZYX Tait-Bryan angles)

function out = computeSE3trajectory(r, b, kin)
    
    % Ensure that the lengths of the kinematic data provided is the same
    if iscell(b) && iscell(r)
        if numel(b) == 6 && numel(r) == 8
            verifylength(b); verifylength(r); % will error out if the trajectory discretizations are unequal
            if numel(b{1}) ~= numel(r{1})
                error('ERROR! The body and shape trajectories should have the same number of discretizations.');
            else
                t = numel(b{1});
            end
        else
            error('ERROR! The number of cells in b and r should be 6 and 8 respectively.');
        end
    else
        error('ERROR! The body and shape trajectories need to be cell arrays.');
    end
    
    % Let's unpack the kinematics
    H3_b__ib = kin.num.H3_b__ib;      % body to each hip transforms
    H3_b__icT = kin.num.H3_b__icT;    % body to top bounding box transforms
    H3_b__icB = kin.num.H3_b__icB;    % body to bot bounding box transforms
    fH3_ib__i = kin.fun.fH3_ib__i;    % each hip to corresponding feet functions
    H3_b__i = kin.sym.H3_b__i;        % body to each foot functions
    fH3_b__i = kin.fun.fH3_b__i;      % body to each foot functions
    fH3_e__b = kin.fun.fH3_e__b;      % origin to body function

    % Initialize your containers
    tH3_e__b = cell(1, t); taxH3_e__b = cell(1, t); % body and frames
    tH3_e__ib = cell(4, t); taxH3_e__ib = cell(4, t); % hip and frames
    tH3_ib__i = cell(4, t); % hip to foot
    tH3_e__icT = cell(4, t); tH3_e__icB = cell(4, t); tH3_icB__icT = cell(4, t); % top and bot boxes and bot2top displacements
    tH3_e__i = cell(4, t); taxH3_e__i = cell(4, t); % leg and frames
    tH3_e__i_swing = cell(4, t); tH3_e__i_lift = cell(4, t); % leg in swing direction first, then lift direction-- shows the trajectory in current leg


    % iterate over the number of frames
    for i = 1:t
        
        % Compute the body location from origin
        tH3_e__b{i} = fH3_e__b( b{1}(i), b{2}(i), b{3}(i), b{4}(i), b{5}(i), b{6}(i) );
        
        % Compute the body frames from origin-- assume 5mm length
        taxH3_e__b{i} = generateSE3frames(tH3_e__b{i}, 5);
    
        % let's perform the compute for the other parts of the body iteratively
        for j = 1:4
            
            tH3_e__ib{j, i} = tH3_e__b{i}*H3_b__ib{j}; % hip and frames
            taxH3_e__ib{j, i} = generateSE3frames(tH3_e__ib{j, i}, 2);
            
            tH3_e__icT{j, i} = tH3_e__b{i}*H3_b__icT{j}; % top
            
            tH3_e__icB{j, i} = tH3_e__b{i}*H3_b__icB{j}; % bot
            
            tH3_icB__icT{j, i} = tH3_e__b{i}*H3_b__icT{j} - tH3_e__b{i}*H3_b__icB{j}; % bot to top

            tH3_ib__i{j, i} = fH3_ib__i{j}(r{1}(i), r{2}(i), r{3}(i), r{4}(i), r{5}(i), r{6}(i), r{7}(i), r{8}(i)); % hip to foot
            
            tH3_e__i{j, i} = tH3_e__b{i}*fH3_b__i{j}(r{1}(i), r{2}(i), r{3}(i), r{4}(i), r{5}(i), r{6}(i), r{7}(i), r{8}(i)); % leg and frames
            taxH3_e__i{j, i} = generateSE3frames(tH3_e__i{j, i}, 2);

            [tH3_e__i_swing{j, i}, tH3_e__i_lift{j, i}] = interpswingliftSE3(tH3_e__b{i}, H3_b__i{j}, i, j, r); % leg interpolation
    
        end

    end

    % Pack up the results and return
    out.tH3_e__b = tH3_e__b; out.taxH3_e__b = taxH3_e__b;
    out.tH3_e__ib = tH3_e__ib; out.taxH3_e__ib = taxH3_e__ib;
    out.tH3_e__icT = tH3_e__icT;
    out.tH3_e__icB = tH3_e__icB;
    out.tH3_icB__icT = tH3_icB__icT;
    out.tH3_ib__i = tH3_ib__i;
    out.tH3_e__i = tH3_e__i; out.taxH3_e__i = taxH3_e__i;
    out.tH3_e__i_swing = tH3_e__i_swing;
    out.tH3_e__i_lift = tH3_e__i_lift;

end

