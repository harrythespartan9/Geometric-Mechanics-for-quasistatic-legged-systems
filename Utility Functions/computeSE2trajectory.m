% This function computes the system's 2D reconstruction along it's trajectory q(t) (configuration 'q' = body 'b' x shape 'r')-- needs functional kinematic 
% transforms defined for the system.
% Follow the steps in "se2_toyproblems_case_1.mlx" section 1 to generate compatible kinematics for your system.
% Inputs: r-- shape trajectory; swing DOF for the leg as a 4x1 cell array with each cell being t-vector long
%         b-- body trajectory; since this is S2(3), this would be a collection of body displacements x(t) and y(t) and yaw rotation '\theta (t)' about 
%             out-of-plane axis Z (defined using RH rule)
function out = computeSE2trajectory(r, b, kin)
    
    % Ensure that the lengths of the kinematic data provided is the same
    if iscell(b) && iscell(r)
        if numel(b) == 3 && numel(r) == 4
            verifylength(b); verifylength(r); % will error out if the trajectory discretizations are unequal
            if numel(b{1}) ~= numel(r{1})
                error('ERROR! The body and shape trajectories should have the same number of discretizations.');
            else
                t = numel(b{1});
            end
        else
            error('ERROR! The number of cells in b and r should be 3 and 4 respectively.');
        end
    else
        error('ERROR! The body and shape trajectories need to be cell arrays.');
    end
    
    % Let's unpack the kinematics
    fh2_e__b = kin.transforms.fun.fH2_e__b;      % origin to body function
    fh2_b__ib = kin.transforms.fun.fH2_b__ib;    % body to hip function
    % fh2_ib__i = kin.transforms.fun.fH2_ib__i;  % hip to leg function
    fh2_b__i = kin.transforms.fun.fH2_b__i;      % body to leg function
    fh2_b__ic = kin.transforms.fun.fH2_e__ic;    % body to corners function
    bl = kin.params.bl;                          % system's body length
    ll = kin.params.ll;                           % system's smallest length parameter from body frame to hip bounds, 'l'
    aa = kin.params.aa;                          % system's leg to 'l' ratio, 'a'

    % Initialize your containers-- SE(2) transformation matrices
    th2_e__b = cell(1, t); taxh2_e__b = cell(1, t);
    th2_e__ib = cell(4, t); taxh2_e__ib = cell(4, t);
    th2_e__i = cell(4, t); taxh2_e__i = cell(4, t);
    th2_e__ic = cell(4, t);
    plotlim = nan(4, t);
    tbody = cell(1, t);
    tframes = cell(1, t);
    tboxes = tframes; 
    tlegs = tframes; tlegs0 = tlegs;
    tfootS = cell(4, t);

    % iterate over each time-step
    for j = 1:t
        
        th2_e__b{j} = fh2_e__b( b{1}(j), b{2}(j), b{3}(j) );
        taxh2_e__b{j} = generateSE2frames(th2_e__b{j}, bl/10); % frame is 10% body-length

        xyzSE2 = th2_e__b{j}'; uvwSE2 = [taxh2_e__b{j}{1}(:)'; taxh2_e__b{j}{2}(:)'];
        framesSE2 = [repmat(xyzSE2, 2, 1), uvwSE2];
        tbody{j} = xyzSE2;

        boxesSE2 = double.empty(0, 6); legsSE2 = boxesSE2; legs0SE2 = legsSE2; limSE2 = double.empty(0, 3);
        
        for i = 1:4
            
            th2_b__ib = fh2_b__ib{i}(ll);
            th2_e__ib{i, j} = seqSE2transformation([th2_e__b{j}, th2_b__ib]);
            taxh2_e__ib{i, j} = generateSE2framesgenerateSE2frames(th2_e__ib{i, j}, bl/20); % frame is 5% body_length
            xyzSE2 = th2_e__ib{i, j}'; uvwSE2 = [taxh2_e__ib{i, j}{1}(:)'; taxh2_e__ib{i, j}{2}(:)']; % HIP
            framesSE2 = [framesSE2; [repmat(xyzSE2, 2, 1), uvwSE2] ];

            th2_b__i = fh2_b__i(aa, ll, r{i}(j)); tfootS{i, j} = interpswingSE2(th2_e__b{j}, fh2_b__i, {aa, ll, r{i}(j)});
            th2_e__i{i, j} = seqSE2transformation([th2_e__b{j}, th2_b__i]);
            taxh2_e__i{i, j} = generateSE2frames(th2_e__i{i, j}, bl/20);
            xyzSE2 = th2_e__i{i, j}'; uvwSE2 = [taxh2_e__i{i, j}{1}(:)'; taxh2_e__i{i, j}{2}(:)']; % LEG
            framesSE2 = [framesSE2; [repmat(xyzSE2, 2, 1), uvwSE2] ];
            limSE2 = [limSE2; xyzSE2 + uvwSE2 ];
            temp_leg = th2_e__i{i, j}' - th2_e__ib{i, j}';
            legsSE2 = [legsSE2; [th2_e__ib{i, j}', temp_leg] ];
            th2_b__i0 = fh2_b__i(aa, ll, 0);
            th2_e__i0 = seqSE2transformation([th2_e__b{j}, th2_b__i0]); % LEG-ORIGIN
            temp_leg = th2_e__i0{i, j}' - th2_e__ib{i, j}';
            legs0SE2 = [legs0SE2; [th2_e__ib{i, j}', temp_leg] ];
            limSE2 = [limSE2; xyzSE2 + uvwSE2 ];
            
            th2_b__ic = fh2_b__ic{i}(ll);
            th2_e__ic{i, j} = seqSE2transformation([th2_e__b{j}, th2_b__ic]);
            xyzSE2 = th2_e__ic{i, j}'; limSE2 = [limSE2; xyzSE2 ];
            if i == 4
                uvwSE2 = fh2_b__ic{1}(ll)' - th2_b__ic';
            else
                uvwSE2 = fh2_b__ic{i+1}(ll)' - th2_b__ic';
            end
            boxesSE2 = [ boxesSE2; [xyzSE2, uvwSE2] ];

        end

        % Compute the plotting limits in each frame
        plotlim(:, j) = [min(limSE2(:,1)), max(limSE2(:,1)),...
            min(limSE2(:,2)), max(limSE2(:,2))]';

        % Assign the frames, boxes, and legs objects for plotting
        tframes{j} = framesSE2;
        tboxes{j} = boxesSE2;
        tlegs{j} = legsSE2;
        tlegs0{j} = legs0SE2;

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

