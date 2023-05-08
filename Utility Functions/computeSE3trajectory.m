% This function computes the 3D reconstruction of a given SE(3) quadrupedal system kinematics along it's trajectory q(t) 
% (configuration 'q' = body 'b' x shape 'r')-- needs symbolic and function SE(3) kinematics of the underlying system.
% Follow the steps in "HAMR6_SE3.mlx" section 1 to generate compatible kinematics for your system.
% Inputs: r-- shape trajectory; for a 2DOF leg quadruped this would be 1x8 cell array with each each cell being t-vector long
%         b-- body trajectory; since this is SE(3), this would be a collection of body displacements and YPR angles in XYZ convention (look up Euler angles)

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
    H3_b__ib = kin.transforms.num.H3_b__ib;      % body to each hip transforms
    H3_b__icT = kin.transforms.num.H3_b__icT;    % body to top bounding box transforms
    H3_b__icB = kin.transforms.num.H3_b__icB;    % body to bot bounding box transforms
    fH3_ib__i = kin.transforms.fun.fH3_ib__i;    % each hip to corresponding feet functions
    H3_b__i = kin.transforms.sym.H3_b__i;        % body to each foot functions
    fH3_b__i = kin.transforms.fun.fH3_b__i;      % body to each foot functions
    fH3_e__b = kin.transforms.fun.fH3_e__b;      % origin to body function
    hip_bound = t_SE3(kin.bounds.hip);           % closed-shape vectors of the hip bounding box
    body_bound = t_SE3(kin.bounds.body);         % closed-shape vectors of the hip bounding box

    % Initialize your containers
    tH3_e__b = cell(1, t); taxH3_e__b = cell(1, t); % body and frames
    tH3_e__ib = cell(4, t); taxH3_e__ib = cell(4, t); % hip and frames
    tH3_ib__i = cell(4, t); % hip to foot
    tH3_e__icT = cell(4, t); tH3_e__icB = cell(4, t); tH3_icB__icT = cell(4, t); % top and bot boxes and bot2top displacements
    tH3_e__i = cell(4, t); taxH3_e__i = cell(4, t); % leg and frames
    tH3_e__i_swing = cell(4, t); tH3_e__i_lift = cell(4, t); % leg in swing direction first, then lift direction-- shows the trajectory in current leg
    plotlim = nan(6, t);
    tbody = cell(1, t); % plotting structs
    tframes = cell(1, t);
    tboxes = tframes; 
    tlegs = tframes; tlegs0 = tlegs;
    tfootS = cell(4, t); tfootL = tframes; % needs to be separate because the trajectories for swing and lift are disjointed

    % iterate over the number of frames
    for i = 1:t
        
        % Compute the body location from origin
        tH3_e__b{i} = fH3_e__b( b{1}(i), b{2}(i), b{3}(i), b{4}(i), b{5}(i), b{6}(i) );
        
        % Compute the body frames from origin-- assume 5mm length
        taxH3_e__b{i} = generateSE3frames(tH3_e__b{i}, 5);

        % Compute the frames structure
        xyzSE3 = return_stacked_tSE3_coords(tH3_e__b{i});
        uvwSE3 = [taxH3_e__b{i}{1}(:)'; taxH3_e__b{i}{2}(:)'; taxH3_e__b{i}{3}(:)'];
        framesSE3 = [repmat(xyzSE3, 3, 1), uvwSE3];
        limSE3 = xyzSE3 + uvwSE3;
        tbody{i} = xyzSE3;
        
        % Initialize other plotting containers
        boxesSE3 = double.empty(0, 6); legsSE3 = boxesSE3; legs0SE3 = legsSE3;

        % let's perform the compute for the other parts of the body iteratively
        for j = 1:4
            
            tH3_e__ib{j, i} = tH3_e__b{i}*H3_b__ib{j}; % hip and frames
            taxH3_e__ib{j, i} = generateSE3frames(tH3_e__ib{j, i}, 2);
            xyzSE3 = return_stacked_tSE3_coords(tH3_e__ib{j, i});                                    %%%% hip frame plotting structure
            uvwSE3 = [taxH3_e__ib{j, i}{1}(:)'; taxH3_e__ib{j, i}{2}(:)'; taxH3_e__ib{j, i}{3}(:)'];
            framesSE3 = [ framesSE3; [repmat(xyzSE3, 3, 1), uvwSE3] ];
            uvwSE3 = return_stacked_tSE3_coords(tH3_e__ib{j, i}*hip_bound{j} - tH3_e__ib{j, i});     %%%% hip box plotting structure
            boxesSE3 = [ boxesSE3; [xyzSE3, uvwSE3] ];
            
            tH3_e__icT{j, i} = tH3_e__b{i}*H3_b__icT{j}; % top
            xyzSE3 = return_stacked_tSE3_coords(tH3_e__icT{j, i});
            uvwSE3 = return_stacked_tSE3_coords(tH3_e__icT{j, i}*body_bound{j} - tH3_e__icT{j, i});
            limSE3 = [limSE3; xyzSE3 + uvwSE3];
            boxesSE3 = [ boxesSE3; [xyzSE3, uvwSE3] ];
            
            tH3_e__icB{j, i} = tH3_e__b{i}*H3_b__icB{j}; % bot
            xyzSE3 = return_stacked_tSE3_coords(tH3_e__icB{j, i});
            uvwSE3 = return_stacked_tSE3_coords(tH3_e__icB{j, i}*body_bound{j} - tH3_e__icB{j, i});
            boxesSE3 = [ boxesSE3; [xyzSE3, uvwSE3] ];
            
            tH3_icB__icT{j, i} = tH3_e__b{i}*H3_b__icT{j} - tH3_e__b{i}*H3_b__icB{j}; % bot to top
            uvwSE3 = return_stacked_tSE3_coords(tH3_icB__icT{j, i});
            boxesSE3 = [ boxesSE3; [xyzSE3, uvwSE3] ];

            tH3_ib__i{j, i} = fH3_ib__i{j}(r{1}(i), r{2}(i), r{3}(i), r{4}(i), r{5}(i), r{6}(i), r{7}(i), r{8}(i)); % hip to foot
            % xyzSE3 = return_stacked_tSE3_coords(tH3_e__ib{j, i}); % old way, prolly incorrect
            % uvwSE3 = return_stacked_tSE3_coords(tH3_ib__i{j, i});
            xyzSE3 = return_stacked_tSE3_coords(tH3_e__ib{j, i});                                                                    %%%% plotting legs-r and r0
            uvwSE3 = return_stacked_tSE3_coords(tH3_e__ib{j, i}*tH3_ib__i{j, i}) - xyzSE3;
            legsSE3 = [ legsSE3; [xyzSE3, uvwSE3] ];
            
            tH3_e__i{j, i} = tH3_e__b{i}*fH3_b__i{j}(r{1}(i), r{2}(i), r{3}(i), r{4}(i), r{5}(i), r{6}(i), r{7}(i), r{8}(i)); % leg and frames
            taxH3_e__i{j, i} = generateSE3frames(tH3_e__i{j, i}, 2);
            xyzSE3 = return_stacked_tSE3_coords(tH3_e__i{j, i});
            uvwSE3 = [taxH3_e__i{j, i}{1}(:)'; taxH3_e__i{j, i}{2}(:)'; taxH3_e__i{j, i}{3}(:)'];
            framesSE3 = [ framesSE3; [repmat(xyzSE3, 3, 1), uvwSE3] ];
            limSE3 = [limSE3; xyzSE3 + uvwSE3];

            [tH3_e__i_swing{j, i}, tH3_e__i_lift{j, i}] = interpswingliftSE3(tH3_e__b{i}, H3_b__i{j}, i, j, r); % leg interpolation
            tfootS{j ,i} = return_stacked_tSE3_coords(tH3_e__i_swing{j, i});                                                         %%%% swing lift traj
            tfootL{j ,i} = return_stacked_tSE3_coords(tH3_e__i_lift{j, i});
            xyzSE3 = return_stacked_tSE3_coords(tH3_e__ib{j, i});                                                                    %%%% leg origin
            uvwSE3 = tfootS{j ,i}(1, :) - xyzSE3; % subtracttion to get the vector from the hip
            legs0SE3 = [ legs0SE3; [xyzSE3, uvwSE3] ];

        end

        % Compute the plotting limits in each frame
        plotlim(:, i) = [min(limSE3(:,1)), max(limSE3(:,1)),...
            min(limSE3(:,2)), max(limSE3(:,2)),...
            min(limSE3(:,3)), max(limSE3(:,3))]';

        % Assign the frames, boxes, and legs objects for plotting
        tframes{i} = framesSE3;
        tboxes{i} = boxesSE3;
        tlegs{i} = legsSE3;
        tlegs0{i} = legs0SE3;

    end

    % Pack up the results and return
    out.plotlim = plotlim;
    out.tH3_e__b = tH3_e__b; out.taxH3_e__b = taxH3_e__b;
    out.tH3_e__ib = tH3_e__ib; out.taxH3_e__ib = taxH3_e__ib;
    out.tH3_e__icT = tH3_e__icT;
    out.tH3_e__icB = tH3_e__icB;
    out.tH3_icB__icT = tH3_icB__icT;
    out.tH3_ib__i = tH3_ib__i;
    out.tH3_e__i = tH3_e__i; out.taxH3_e__i = taxH3_e__i;
    out.tH3_e__i_swing = tH3_e__i_swing;
    out.tH3_e__i_lift = tH3_e__i_lift;
    out.tbody = tbody;
    out.tframes = tframes;
    out.tboxes = tboxes;
    out.tlegs = tlegs;
    out.tlegs0 = tlegs0;
    out.tfootS = tfootS;
    out.tfootL = tfootL;
    

end

