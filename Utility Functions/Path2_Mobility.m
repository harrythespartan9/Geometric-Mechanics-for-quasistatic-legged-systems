% This is a "Path2_Mobility" class for defining paths on level-2 no-slip 
% contact submanifolds for a rigid quadrupedal robot. It takes the initial 
% condition for the shape-space slice (2 dim), gait constraint vector field
% (2 dim), and integration time for the path to construct a Path2 object. 
% It inherits properties from the abstract class "RigidGeomQuad".

classdef Path2_Mobility

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (SetAccess = public)

        sIdx      % the contact state that this path belongs to, i; an 
                  % ... integer in the range (1, 6) the contact state 
                  % ... ordering is as follows: {12, 23, 34, 41, 13, 24}

        kin       % kinematics of the system 

        kinfunc   % kinematics of level-2 contact submanifolds

        p_kin     % kinematics as a function of discretized shape space for 
                  % ... plotting

        p_info    % plotting parameters

        sRow      % row index in two-beat gait list

        sCol      % column index in two-beat gait list

        cs        % current lvl-2 stance phase legs

        a         % link to leg length ratio

        l         % link length defined as the x and y offsets from body 
                  % ... frame to hip frame

        aLimits   % the componentwise (along row) limits of limb positions 
                  % ... for the current shape subspace

        refPt     % the reference point for computing the slip and nonslip 
                  % ... coordinates

        ai        % the shape variable encoding the first limb in the 
                  % ... contact state

        aj        % the shape variable encoding the second limb in the 
                  % ... contact state

        F_fxn     % function to compute F given body parameters

        dz        % stratified panel function

        dQ        % configuration velocity along the nonslip paths

        aInf      % limb coordinates at the infimum of F

        aSup      % limb coordinates at the supremum of F

        F_inf     % infimum of F

        F_sup     % supremum of F

        F_bounds  % bounds on the accessible values of F and is a 
                  % ... subinterval of [F_inf, F_sup]

        slipMode  % mode for slipping axis construction: 'legacy' and 'branched'

        perpFdirn % unit vector field generator for paths along the 
                  % gradient of F

        paraFdirn % unit vector field generator for paths along a level-set 
                  % of F

        intTime   % time to integrate along the forward and backward directions

        aPerpF    % new coordinate 1: Slip directions
                  % ... the direction perpendicular to the
                  % ... level-sets of F or along the gradient of F

        aParallF  % new coordinate 2: Nonslip directions
                  % ... the direction parallel to the level-sets of F 
                  % ... computed at along each point in 'aPerpF'

        aParaRef  % new coordinate 2: Nonslip directions
                  % ... similar to 'aParallF', except only about a
                  % ... requested reference point
                  % ... gets overwritten everytime the corresponding method
                  % ... is called
                  % ... the constructor initializes this with the refernce
                  % ... point provided for computing the new coordinates

        phaseReq  % whether phase bounds are used to stop integration when 
                  % the F levelsets connect back on themselves

    end
    
    % just constructor %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        
        % Constructor
        function thisPath2 = Path2_Mobility(sIdx, ...
                                        kin, kinfunc, p_kin, p_info, ...
                                        intTime, refPt, phaseReq, slipMode)
            % assign the props
            thisPath2.sIdx = sIdx;
            thisPath2.kin = kin;
            thisPath2.kinfunc = kinfunc{sIdx};
            thisPath2.p_kin = p_kin{sIdx};
            thisPath2.p_info = p_info{sIdx};
            thisPath2.intTime = intTime;
            thisPath2.slipMode = slipMode;
            if ~exist("phaseReq", "var")
                thisPath2.phaseReq = [];
            else
                thisPath2.phaseReq = phaseReq;
            end
            % general reference for the limb position subspace as provided
            thisPath2.refPt = refPt;
            % get the leg indices that for the current contact state
            [thisPath2.sRow, thisPath2.sCol] = ...
                                    find(kin.cs_IdxFor2bGaits == sIdx);
            thisPath2.cs = kin.cs( kin.cs_IdxFor2bGaits(...
                            thisPath2.sRow, thisPath2.sCol), : );
            % obtain the discretized coordinates for the two limbs in
            % stance
            % ... this will be what the meshgrid outputs for a 2D space
            thisPath2.ai = thisPath2.p_kin.ai;
            thisPath2.aj = thisPath2.p_kin.aj;
            thisPath2.aLimits=[p_info{sIdx}.xlimits; 
                               p_info{sIdx}.ylimits];
            % supremum and infimum of F and the corresponding limb
            % locations
            % ... F is squared, inter-leg distance for the stancing feet
            % ...  finding the limb locations are easier through fmincon
            a = thisPath2.kinfunc.aa;
            l = thisPath2.kinfunc.ll;
            F_fxn = thisPath2.kin.ksq_ij{sIdx};
            aInf = fmincon(@(x) F_fxn(a, l, x(1), x(2)), ...
                refPt, [], [], [], [], ...
                -pi*ones(1, 2), pi*ones(1, 2), [],... % find infimum loc
                optimoptions("fmincon", "Display", "none"));
            aSup = fmincon(@(x) -F_fxn(a, l, x(1), x(2)), ...
                refPt, [], [], [], [], ...
                -pi*ones(1, 2), pi*ones(1, 2), [],... % find supremum loc
                optimoptions("fmincon", "Display", "none")); 
            F_inf = F_fxn(a, l, aInf(1), aInf(2)); % F infimum location
            F_sup = F_fxn(a, l, aSup(1), aSup(2)); % F supremum location
            thisPath2.a = a; thisPath2.l = l; thisPath2.F_fxn = F_fxn;
            thisPath2.F_inf = F_inf; thisPath2.F_sup = F_sup;
            thisPath2.aInf = aInf; thisPath2.aSup = aSup; % assign props
            % find the allowable range for F
            % ... this constrains the accessible shape subspace and
            % ... excludes the singularities that occur at 'aInf' and 
            % ... 'aSup'; can't plan there.
            % ... the subrange is obtained by setting an inner threshold
            % ... on [F_inf, F_sup]
            F_range  = F_sup - F_inf; prctRm = 2; % SET PERCENTAGE HERE
            thisPath2.F_bounds= prctRm/100*F_range*[+1 -1] + [F_inf F_sup];
            % generator vector fields
            % ... we need generators for paths along the level-set of F and
            % ... along the gradient of F
            % ... these generators also have must have unit magnitudes at
            % ... all points in the shape space
            thisPath2.perpFdirn = thisPath2.kin.dksq_dirn_ij{sIdx};
            thisPath2.paraFdirn = matlabFunction(thisPath2.kinfunc.dpsi,...
                "Vars", [ sym("a"), sym("l"),...
                sym(['alpha_' num2str(thisPath2.cs(1))]),...
                sym(['alpha_' num2str(thisPath2.cs(2))]) ]);
            % startified panel
            % ... we obtain the symbolic expression and convert it into a
            % ... function for later use
            thisPath2.dz = matlabFunction(thisPath2.kin.dz_psi{sIdx}', ...
                "Vars", [ sym("a"), sym("l"),...
                sym(['alpha_' num2str(thisPath2.cs(1))]),...
                sym(['alpha_' num2str(thisPath2.cs(2))]) ]);
            % configuration velocity
            % ... we construct this velocity using symbolic variables and
            % ... then use that to create a function for later use
            % ... ENSURE to negate the stratified panels because they are
            % ... defined '-'ly because of tradition
            dQ_now = matlabFunction( [rot_SE2(sym("theta")), zeros(3, 2); 
                                            zeros(2, 3), eye(2, 2)]*...
                      [thisPath2.kin.dz_psi{sIdx}; thisPath2.kinfunc.dpsi], ...
                      "Vars", [ sym("a"), sym("l"),... % body params
                                sym("x"), sym("y"), sym("theta"),... % position
                                sym(['alpha_' num2str(thisPath2.cs(1))]),... % shape vars
                                sym(['alpha_' num2str(thisPath2.cs(2))]) ]);
            thisPath2.dQ = dQ_now;
            % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
            % compute the nonslip-slip shape coordinates
            % ... we first compute the slip coordinates from the specified
            % ... reference point
            % ... the colors at each F-lvl is also computed after obtain
            % ... these coordinates
            % ... then, about the slipping trajectory we compute the
            % ... nonslip paths and provide a flag which checks if
            % ... sufficient integration time is achieved
            % ... for more info, check the definitions of functions called
            % ... below
            % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
            thisPath2 = Path2_Mobility.computePerpendicularCoordinates...
                                                    ( thisPath2, slipMode );
            thisPath2.aPerpF.color = cell(size(thisPath2.aPerpF.F));
            for i = 1:numel(thisPath2.aPerpF.color)
                thisPath2.aPerpF.color{i} = interpColorAndCondition...
                    ( ...
                    linspace(F_inf, F_sup, size(thisPath2.p_info.jetDark, 1))', ...
                    thisPath2.p_info.jetDark, thisPath2.aPerpF.F{i} ...
                    );
            end
            thisPath2 = Path2_Mobility.computeParallelCoordinates...
                                                            ( thisPath2 );
            thisPath2 = Path2_Mobility.computeSpecificParallelCoordinates...
                                    ( thisPath2, refPt, [], true );
            % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
            % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
            % increment the number of objects
            Path2.SetGet_static(1);
        end
        
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Static methods below %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Static)


        
        % static function to icnrement the number of objects
        function out = SetGet_static(~)
            persistent var
            if isempty(var)
                var = 0;
            end
            if nargin == 1
                var = var + 1;
            else
                out = var;
            end
        end

        % function to compute perpendicular limb shape coordinate
        % ... this is the path along the gradient of F that helps us select 
        % ... a specific level-set of F
        function thisPath2 = computePerpendicularCoordinates...
                                                    ( thisPath2, method )
            % if the method is not provided, execute the legacy method by
            % default
            if nargin < 2
                method = 'legacy';
            end
            % do the computation based on the method requested
            switch method
                case 'legacy'
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % unpack properties and pack 
                    % ... first required parameters followed by the bounds 
                    % ... needed and finally the functions needed
                    % ... this argument structure is used to stop the ODE
                    % ... integration process when shape space constraints 
                    % ... are violated
                    % ... an event will choose the necessary EVENTs to check for to
                    % ... stop the ODE integration process: we specifically need
                    % ... shape bounds check and F bounds check as we have hard
                    % ... constraints on those
                    % ... for more information, see: "nonslipShapeCoordsEvents.m"
                    argStruct = [];
                    argStruct.parameters.a = thisPath2.a; 
                        argStruct.parameters.l = thisPath2.l; % params
                    argStruct.bounds.alphaLimits = thisPath2.aLimits; % bounds
                    argStruct.bounds.F_bounds = thisPath2.F_bounds;
                    argStruct.functions.F_fxn = thisPath2.F_fxn; % function
                    eventList = {'shape_bounds', 'F_bounds'};
                    % ... set the ode EVENT options
                    odeOptions = odeset('Events', ...
                    @(t, y) nonslipShapeCoordsEvents(t, y, argStruct, eventList));
                    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
                    % compute the 'perp' coordinate
                    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
                    % ... we integrate for 100 duration because we want to stop
                    % ... using events, and exact stopping duration is almost
                    % ... always unknown
                    % ... finally, we integrate from the backward position all the
                    % ... way till the forward position to obtain the solution
                    [~, ~, teNow, ~, ~] = ...
                        ode89( @(t, y) +thisPath2.perpFdirn(...
                                thisPath2.a, thisPath2.l, y(1), y(2)), ...
                                [0 100], ... 
                                thisPath2.refPt, ... % go FORWARD from reference
                                odeOptions ); 
                    tFwd = teNow;
                    [~, ~, teNow, yeNow, ~] = ...
                        ode89( @(t, y) -thisPath2.perpFdirn(...
                                thisPath2.a, thisPath2.l, y(1), y(2)), ...
                                [0 100], ... 
                                thisPath2.refPt, ... % go BACKWARD from reference
                                odeOptions );
                    tBwd = teNow; yBwd = yeNow;
                    tMain = tFwd + tBwd; % integration duration for MAIN solution
                    [tNow, yNow, teNow, ~, ~] = ...
                        ode89( @(t, y) +thisPath2.perpFdirn(...
                                thisPath2.a, thisPath2.l, y(1), y(2)), ...
                                linspace(0, tMain, size(thisPath2.ai, 1)), ... 
                                yBwd, ... % MAIN solution: integrate b/w ends
                                odeOptions );
                    % ... check if an event occured and condition the solution
                    % ... before storing
                    if ~isempty(teNow)
                        yNow = yNow(tNow <= teNow, :);
                        tNow = tNow(tNow <= teNow);
                    end
                    % ... store the solution and return
                    thisPath2.aPerpF.t = tNow;
                    thisPath2.aPerpF.y = yNow;
                    thisPath2.aPerpF.F = thisPath2.F_fxn...
                        (thisPath2.a, thisPath2.l, yNow(:, 1), yNow(:, 2));
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                case 'branched'
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % obtain stuff needed to obtain the branches
                    aa = thisPath2.kinfunc.aa;
                    ll = thisPath2.kinfunc.ll;
                    aInf = thisPath2.aInf;
                    aSup = thisPath2.aSup;
                    r = thisPath2.refPt;
                    ankX = thisPath2.aLimits(1, :); ankXr = diff(ankX);
                    ankY = thisPath2.aLimits(2, :); ankYr = diff(ankY);
                    ankr = mean([ankXr, ankYr]);
                    F_fxn = thisPath2.F_fxn;
                    F_bounds_full = [thisPath2.F_inf, thisPath2.F_sup];
                    F_bounds = thisPath2.F_bounds;
                    numPtsTotal = size(thisPath2.ai, 1);
                    % arrange the points a certain way
                    pts = {aInf, r, aSup};
                    % iterate over each ordered pair and get the intervals of interest
                    slipAxisPoints = [];
                    % check which iteration a branch belongs to
                    % ... is it from the infimum to the reference
                    % ... is it from the reference to the supremum
                    branchDirn = [];
                    % ... we do not know how many branches might exist
                    % ... beforehand: anywhere between 2 to 4
                    for i = 1:(numel(pts)-1)
                        % get a vector from point 1 (tail of arrow above) to point 2 (head)
                        vec = pts{i+1} - pts{i};
                        % ensure that this vector is not too small
                        % ... for now I'm setting the threshold to 1% of the allowable
                        % ... symmetric range that we have assumed
                        % ... if the threshold is not cleared, use the next vector or the
                        % ... previous vector
                        if norm(vec) - 2*ankr*1e-2 < 0
                            switch i
                                case 1
                                    vec = pts{i+2} - pts{i+1};
                                case 2
                                    vec = pts{i} - pts{i-1};
                            end
                        end
                        % get the phasor info
                        phasorAngle1 = atan2(vec(2), vec(1));
                        % get the x and y components for the line generator
                        delX = cos(phasorAngle1);
                        delY = sin(phasorAngle1);
                        % find the a value at which the next point occurs
                        aTemp = vec./[delX, delY]; 
                        aTemp(isinf(aTemp)) = nan; % nan INF values
                        aPtNext = mean(aTemp, "omitnan"); % omit nans
                        % current pt is at 0
                        aPtCurr = 0;
                        % find the intersection to actual shape space bounds
                        aAnkX = (ankX - repmat(pts{i}(1), 1, 2))/delX;
                        aAnkX(isinf(aAnkX)) = nan; % NaNize
                        aAnkY = (ankY - repmat(pts{i}(2), 1, 2))/delY;
                        aAnkY(isinf(aAnkY)) = nan; % NaNize
                        % accessible shape space and allowable points
                        % ... find the "ank" on the boundary
                        aAnk = [aAnkX, aAnkY];
                        % ... get the points on the boundary
                        % ... ... do this for the x and y points separately
                        ptAnk = returnPtAlongLineFromIC(pts{i}, ...
                                                [delX, delY], ...
                                                aAnk);
                        ptAnkX = ptAnk(1, :); ptAnkY = ptAnk(2, :);
                        validAnkX = all...
                            (ptAnkX >= ankX(1)-0.01*ankXr ...
                           & ptAnkX <= ankX(2)+0.01*ankXr, 1);
                        validAnkY = all...
                            (ptAnkY >= ankY(1)-0.01*ankYr ...
                           & ptAnkY <= ankY(2)+0.01*ankYr, 1);
                        validAnk = validAnkX & validAnkY;
                        aAnk = aAnk(validAnk);
                        switch i
                            case 1 % if the next point is the refernce point
                                aAnk = aAnk(aAnk < aPtNext);
                            case 2 % if the current point is the reference point
                                aAnk = aAnk(aAnk > aPtCurr);
                        end
                        % ... if the are still multiple points, take the mean to get a single
                        % ... point
                        aAnk = mean(aAnk);
                        % ... define a list and sort the points
                        aList = [aPtNext, aPtCurr, aAnk]; % initialize the list
                        switch i
                            case 1 % inf(F) singularity to reference
                                aList = sort( aList(aList >= aAnk & aList <= aPtNext) );
                            case 2 % reference to sup(F) singularity
                                aList = sort( aList(aList >= aPtCurr & aList <= aAnk) );
                        end
                        % ... get the points along the limits
                        ptList = returnPtAlongLineFromIC(pts{i}, [delX, delY], aList);
                        ptList = mat2cell(ptList, size(ptList, 1), ones(1, size(ptList, 2)));
                        % iterate over the number of points and do stuff
                        validFlagList = false(1, numel(ptList));
                        for j = 1:numel(ptList)
                            % current point
                            ptNow = ptList{j};
                            % evaluate the F-value at these points and see if the value is
                            % being violated
                            FvalNow = F_fxn(aa, ll, ptNow(1), ptNow(2));
                            % check if any of the points are violating the condition, if
                            % neither are, then there is no weird point in between
                            % write an optimizer to check if there is a lower bound or an
                            % upper bound
                            if FvalNow >= F_bounds(1) && FvalNow <= F_bounds(2)
                                validFlagList(j) = true;
                            end
                        end
                        validNum = nnz(validFlagList);
                        % based on the number of points we shall do the following
                        % ... if two points it is relatively straightforward
                        % ... else, if it is three points, then we need to iterate over the two
                        % ... branches that occur and obtain the solutions separately to see if
                        % ... there is a F-value violation
                        switch numel(aList)
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            case 2 % ONE branch
                                % if both points are valid generate a normal branch
                                switch validNum
                                    case 1
                                        % only one of the points is valid, find the point where
                                        % the bounds are violated, then it is that point to the
                                        % valid point
                                        aViolation = ptViolatingFbounds...
                                            (aa, ll, ...
                                            F_fxn, F_bounds, F_bounds_full, ...
                                            pts{i}, delX, delY, ...
                                            aList(1), [aList(1), aList(2)]);
                                        % if the violation point is valid, then use
                                        if exist("aViolation", "var")
                                            [aList, sortIdx] = sort([aList, aViolation]);
                                            validFlagList = [validFlagList, true];
                                            validFlagList = validFlagList(sortIdx); % obtain the locations
                                            aList = aList( ...
                                                find(validFlagList, 1, "first"):... % starting point
                                                find(validFlagList, 1, "last")... % end point
                                                            ); % obtain the valid branch
                                            slipAxisPoints{end+1} = ...
                                                returnPtAlongLineFromIC(...
                                                        pts{i}, [delX, delY], ...
                                                        [aList(1), aList(2)]...
                                                                        )';
                                            branchDirn{end+1} = i;
                                            clear aViolation;
                                        else
                                            error("ERROR! Not sure what's going on here.");
                                        end
                                    case 2
                                        % both points are valid, obtain the slipping axis
                                        slipAxisPoints{end+1} = ...
                                            returnPtAlongLineFromIC(...
                                                    pts{i}, [delX, delY], ...
                                                    [aList(1), aList(2)]...
                                                                    )';
                                        branchDirn{end+1} = i;
                                    otherwise
                                        error(['ERROR! The reference point pts{2} should always be valid. ' ...
                                            'Even if it is, do not place it closer to the corners of the ' ...
                                            'valid shape space.']);
                                end
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            case 3 % TWO branches
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                % get the working indices
                                switch validNum
                                    case 1
                                        workingIdx = find(validFlagList, 1, "first");
                                        notWorkingIdx = find(~validFlagList, 2, "first");
                                        % there can only be one branch here
                                        aViolation = ptViolatingFbounds...
                                            (aa, ll, ...
                                            F_fxn, F_bounds, F_bounds_full, ...
                                            pts{i}, delX, delY, ...
                                            aList(2), [aList(1), aList(3)]);
                                        if exist("aViolation", "var")
                                            [aList, sortIdx] = sort([aList(workingIdx), aViolation]);
                                            validFlagList = [validFlagList, true];
                                            validFlagList = validFlagList(sortIdx);
                                            slipAxisPoints{end+1} = ...
                                                returnPtAlongLineFromIC(...
                                                        pts{i}, [delX, delY], ...
                                                        [aList(1), aList(2)]...
                                                                        )';
                                            branchDirn{end+1} = i;
                                            clear aViolation;
                                        else
                                            error("ERROR! Not sure what's going on here.");
                                        end
                                    case 2
                                        workingIdx = find(validFlagList, 2, "first");
                                        notWorkingIdx = find(~validFlagList, 1, "first");
                                        % see how many branches are there, and then do it accordingly
                                        switch any(notWorkingIdx == 2)
                                            case 1 % 2 branches
                                                for j = 1:(numel(validFlagList)-1)
                                                    aSubList = aList(j:j+1);
                                                    validFlagSubList = validFlagList(j:j+1);
                                                    aViolation = ptViolatingFbounds...
                                                        (aa, ll, ...
                                                        F_fxn, F_bounds, F_bounds_full, ...
                                                        pts{i}, delX, delY, ...
                                                        aSubList(1), ...
                                                        [aSubList(1), ...
                                                        aSubList(2)]);
                                                    if exist("aViolation", "var")
                                                        [aSubList, sortIdx] = sort([aSubList(workingIdx(j)-j+1), aViolation]);
                                                        validFlagSubList = [validFlagSubList(workingIdx(j)-j+1), true];
                                                        validFlagSubList = validFlagSubList(sortIdx);
                                                        slipAxisPoints{end+1} = ...
                                                            returnPtAlongLineFromIC(...
                                                                    pts{i}, [delX, delY], ...
                                                                    [aSubList(1), aSubList(2)]...
                                                                                    )';
                                                        branchDirn{end+1} = i;
                                                        clear aViolation;
                                                    else
                                                        error("ERROR! Not sure what's going on here.");
                                                    end
                                                end
                                            case 0 % 1 branch
                                                endWorkingIdx = workingIdx(workingIdx == 1 || workingIdx == 3);
                                                aEndWorkingPt = aList(endWorkingIdx);
                                                validFlagEndWorking = validFlagList(endWorkingIdx);
                                                aViolation = ptViolatingFbounds...
                                                        (aa, ll, ...
                                                        F_fxn, F_bounds, F_bounds_full, ...
                                                        pts{i}, delX, delY, ...
                                                        aList(2), ...
                                                        [aList(1), ...
                                                        aList(3)]);
                                                if exist("aViolation", "var")
                                                    [aList, sortIdx] = sort([aEndWorkingPt, aViolation]);
                                                    validFlagList = [validFlagList, true];
                                                    validFlagList = validFlagList(sortIdx);
                                                    slipAxisPoints{end+1} = ...
                                                        returnPtAlongLineFromIC(...
                                                                pts{i}, [delX, delY], ...
                                                                [aList(1), aList(2)]...
                                                                                )';
                                                    branchDirn{end+1} = i;
                                                    clear aViolation;
                                                else
                                                    error("ERROR! Not sure what's going on here.");
                                                end
                                            otherwise
                                                error('ERROR! You should not be here.');
                                        end
                                    otherwise
                                        error('ERROR! You really should not be here.');
                                end
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        end
                    end
                    % Check for the number of branches and return the
                    % perperndicular coodinates based on path length
                    % ... first step find the distance to the branches from
                    % ... 'r'
                    dist = cell(1, numel(slipAxisPoints));
                    for i = 1:numel(slipAxisPoints)
                        switch branchDirn{i}
                            case 1
                                mul = -1;
                            case 2
                                mul = +1;
                        end
                        dist{i} = mul*vecnorm(...
                            slipAxisPoints{i} - repmat(r, 2, 1), ...
                                            2, 2);
                    end
                    % ... based on this distance, merge branches if
                    % ... required
                    modifiedDist = [];
                    modifiedSlipAxisPoints = [];
                    branchLength = [];
                    branchFlag = false(1, numel(slipAxisPoints));
                    for i = 1:numel(slipAxisPoints)-1
                        % check if both points are not added
                        if ~branchFlag(i) && ~branchFlag(i+1)
                            % get the ending point and starting point 
                            % between two branches
                            currEndDist = dist{i}(2); 
                            nextStartDist = dist{i+1}(1);
                            % see if they are within a certain 
                            % threshold
                            branchDistDiff= nextStartDist-currEndDist;
                            if branchDistDiff < 1e-3*ankr % <0.1% of range
                                modifiedSlipAxisPoints{end+1} = ...
                                    [slipAxisPoints{i}(1, :); 
                                    slipAxisPoints{i+1}(2, :)];
                                modifiedDist{end+1} = ...
                                    [dist{i}(1); 
                                    dist{i+1}(2)];
                                branchLength{end+1} = ...
                                    norm(...
                                    diff(modifiedDist{end}, 1, 1)...
                                            );
                                branchFlag(i) = true;
                                branchFlag(i+1) = true;
                            end
                        else % IF NO MERGING, JUST ADD NORMALLY
                            % check point by point and add them separately
                            % since the branches are not "connected"
                            if ~branchFlag(i)
                                modifiedSlipAxisPoints{end+1} = ...
                                    slipAxisPoints{i};
                                modifiedDist{end+1} = dist{i};
                                branchLength{end+1} = ...
                                    norm(...
                                    diff(modifiedDist{end}, 1, 1)...
                                            );
                                branchFlag(i) = true;
                            end
                            if ~branchFlag(i+1)
                                modifiedSlipAxisPoints{end+1} = ...
                                    slipAxisPoints{i+1};
                                modifiedDist{end+1} = dist{i+1};
                                branchLength{end+1} = ...
                                    norm(...
                                    diff(modifiedDist{end}, 1, 1)...
                                            );
                                branchFlag(i+1) = true;
                            end
                        end
                    end
                    % compute the total branch length
                    thisPath2.aPerpF.totalLength = 0;
                    thisPath2.aPerpF.numPtsTotal = numPtsTotal;
                    for i = 1:numel(branchLength)
                        thisPath2.aPerpF.totalLength = ...
                            thisPath2.aPerpF.totalLength + branchLength{i};
                    end
                    totalLength = thisPath2.aPerpF.totalLength;
                    % store the modified slip axis points in struct, and
                    % then construct the usual slipping axis or
                    % perpendicular coordinates
                    thisPath2.aPerpF = []; 
                    remainingNumPts = numPtsTotal;
                    for i = 1:numel(modifiedSlipAxisPoints)
                        numPtsNow = ...
                            round(branchLength{i}/totalLength*numPtsTotal);
                        if numPtsNow < 1
                            numPtsNow = 1; % condition
                        end
                        % ... cap the last branch to whatever is remaining
                        if i == numel(modifiedSlipAxisPoints)
                            numPtsNow = remainingNumPts; 
                        else % remove points
                            remainingNumPts = remainingNumPts - numPtsNow; 
                        end
                        % % % % % % % % % %  compute
                        tNow = linspace(...
                            modifiedDist{i}(1), modifiedDist{i}(2), ...
                            numPtsNow)';
                        yNow = interp1(...
                            modifiedDist{i}, modifiedSlipAxisPoints{i}, ...
                            tNow, ...
                            "linear");
                        Fnow = F_fxn(aa, ll, yNow(:, 1), yNow(:, 2));
                        thisPath2.aPerpF.branches.pts{i} = numPtsNow;
                        thisPath2.aPerpF.branches.modifiedSlipAxisPoints{i} = ...
                            modifiedSlipAxisPoints{i};
                        thisPath2.aPerpF.branches.modifiedDist{i} = ...
                            modifiedDist{i};
                        thisPath2.aPerpF.branches.branchLength{i} = ...
                            branchLength{i};
                        thisPath2.aPerpF.t{i} = tNow;
                        thisPath2.aPerpF.y{i} = yNow;
                        thisPath2.aPerpF.F{i} = Fnow;
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
        end

        % function to compute the level-set colors at each 

        % function to compute parallel limb shape coordinate
        % ... this is the path along the level-sets of F, obtained by
        % ... rotating the gradient 90degs clockwise (ensures an 
        % ... anticlockwise generator vector field around the singularity) 
        function thisPath2 = computeParallelCoordinates( thisPath2 )
            % unpack somethings we use more frequently
            aa = thisPath2.a; ll = thisPath2.l;
            aInf = thisPath2.aInf; aSup = thisPath2.aSup;
            ai = thisPath2.ai; aLimits = thisPath2.aLimits;
            yPerp = thisPath2.aPerpF.y;
            tPerp = thisPath2.aPerpF.t;
            intTime = thisPath2.intTime;
            paraFdirn = thisPath2.paraFdirn;
            dz = thisPath2.dz;
            phaseReq = thisPath2.phaseReq; % check phase requirements
            slipMode = thisPath2.slipMode; % check slipping
            % iterate and find centroid of level-set
            % ... this has to accomodate the number of branches you have
            % ... we iterate over each solution point and try to identify
            % ... the centroid about which they connect on themselves
            % ... for our case, the centroid is either the infimum or
            % ... supremum point in the reduced shape space
            % ... this can be sufficiently coarse, so we are using 'ode23'
            % ... because we only want to identify which direction (towards
            % ... infimum or supremum) the centroid is moving towards
            % ... to do this, we shall initialize two quantities and store
            % ... that information:
            switch phaseReq
                case "NoPhaseLimits" % no centroid needed
                    eventList = {'shape_bounds'};
                otherwise
                    eventList = {'shape_bounds', 'phase_bounds'};
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    switch slipMode
                        case 'legacy'
                            yExtremal = nan(size(yPerp));
                            yRefPhase = nan(size(tPerp));
                            tInt = 100*[1 1]; % choose large integration time
                            for i = 1:size(yPerp, 1)
                                [~, yNow] = ode23(@(t, y) -paraFdirn(...
                                    aa, ll, y(1), y(2)),...
                                    [0 tInt(1)], yPerp(i, :)); % backward pass
                                [~, yNow] = ode23(@(t, y) +paraFdirn(...
                                    aa, ll, y(1), y(2)),...
                                    [0 sum(tInt)], yNow(end, :)); % forward pass
                                errInf = repmat(aInf, size(yNow, 1), 1) - yNow; 
                                errInf = min(vecnorm(errInf, 2, 2)); % distance from Inf
                                errSup = repmat(aSup, size(yNow, 1), 1) - yNow; 
                                errSup = min(vecnorm(errSup, 2, 2)); % distance from Sup
                                switch errInf < errSup
                                    case 1
                                        yExtremal(i, :) = aInf; 
                                    case 0
                                        yExtremal(i, :) = aSup;
                                end
                                yPhasorNow = (yPerp(i, :)-yExtremal(i, :))/...
                                         norm(yPerp(i, :)-yExtremal(i, :)); 
                                yRefPhase(i) = atan2(yPhasorNow(2), yPhasorNow(1));
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        case 'branched'
                            yExtremal = cell(size(yPerp));
                            yRefPhase = nan(size(tPerp));
                            for bItr = 1:numel(yPerp)
                                yPerpNow = yPerp{bItr};
                                tPerpNow = tPerp{bItr};
                                yExtremal{bItr} = nan(size(yPerpNow));
                                yRefPhase{bItr} = nan(size(tPerpNow));
                                tInt = 100*[1 1]; % choose large integration time
                                for i = 1:size(yPerpNow, 1)
                                    [~, yNow] = ode23(@(t, y) -paraFdirn(...
                                        aa, ll, y(1), y(2)),...
                                        [0 tInt(1)], yPerpNow(i, :)); % backward pass
                                    [~, yNow] = ode23(@(t, y) +paraFdirn(...
                                        aa, ll, y(1), y(2)),...
                                        [0 sum(tInt)], yNow(end, :)); % forward pass
                                    errInf = repmat(aInf, size(yNow, 1), 1) - yNow; 
                                    errInf = min(vecnorm(errInf, 2, 2)); % distance from Inf
                                    errSup = repmat(aSup, size(yNow, 1), 1) - yNow; 
                                    errSup = min(vecnorm(errSup, 2, 2)); % distance from Sup
                                    switch errInf < errSup
                                        case 1
                                            yExtremal{bItr}(i, :) = aInf; 
                                        case 0
                                            yExtremal{bItr}(i, :) = aSup;
                                    end
                                    yPhasorNow = (yPerpNow(i, :)-yExtremal{bItr}(i, :))/...
                                             norm(yPerpNow(i, :)-yExtremal{bItr}(i, :)); 
                                    yRefPhase{bItr}(i) = atan2(yPhasorNow(2), yPhasorNow(1));
                                end
                            end
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
            % the main solution
            % ... also make this compatible with the slipMode
            % ... the main solution is going to have a very similar
            % ... structure to the perpendicular solution, except at each
            % ... perpendicular solution, we need to compute a whole
            % ... level-set of F
            % ... the angle threshold is set to 5 degrees meaing from the
            % ... self-connection point of the F level-set, the ode 
            % ... integration stops about +-5 degrees earlier
            % ... the integration time provided in the 'intTime' property
            % ... needs to be met ny the solution before a terminating
            % ... EVENT occurs and this is stored as a validation flag
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            switch slipMode
                case 'legacy'
                    thisPath2.aParallF = [];
                    thisPath2.aParallF.isValid = true(size(tPerp));
                    thisPath2.aParallF.eventInfo = cell(size(tPerp));
                    thisPath2.aParallF.y = cell(size(tPerp)); % init 
                            thisPath2.aParallF.t = thisPath2.aParallF.y;
                    thisPath2.aParallF.dz = thisPath2.aParallF.y;
                    argStruct = []; argStruct.functions = []; % struct init
                    argStruct.bounds.alphaLimits = aLimits; % bounds
                    if strcmp(phaseReq, 'NoPhaseLimits')
                        argStruct.bounds.angleThreshold = deg2rad(5);
                    end
                    for i = 1:numel(thisPath2.aParallF.y)
                        % ... initialize current event info cell
                        thisPath2.aParallF.eventInfo{i} = cell(1, 2);
                        % ... select initial conditions
                        y0now = yPerp(i, :);
                        % ... select the current EVENT options
                        if strcmp(phaseReq, 'NoPhaseLimits') % params
                            argStruct.parameters.yExtremal =yExtremal(i,:);
                            argStruct.parameters.yRefPhase = yRefPhase(i);
                        end
                        optionsNow = odeset(  ...
                            'Events', ...
                            @(t, y) nonslipShapeCoordsEvents(t, y, argStruct, ...
                            eventList) ...
                            );
                        % ... integrate forward and backward to find the limits of
                        % ... the F level-set in terms of path length within the 
                        % ... allowable shape space bounds and phase bounds as 
                        % ... considered earlier
                        tMax = nan(1, 2);
                        % ... ... integrate backward and get the time bounds
                        [~, ~, teNow, ~, ~] = ...
                        ode89( @(t,y) -paraFdirn(aa, ll, y(1), y(2)), ...
                        [0, 1e2], y0now, optionsNow ); % for a long time,
                                                       % event will happen quicker
                        if ~isempty(teNow)
                            tMax(1) = teNow;
                        end
                        % ... ... same for forward path
                        [~, ~, teNow, ~, ~] = ...
                        ode89( @(t,y) +paraFdirn(aa, ll, y(1), y(2)), ...
                        [0, 1e2], y0now, optionsNow ); % for a long time, even will 
                                                       % happen quicker
                        if ~isempty(teNow)
                            tMax(2) = teNow;
                        end
                        % ... integrate backward path for the main solution
                        [tNow, yNow, teNow, ~, ieNow] = ...
                        ode89( @(t,y) -paraFdirn(aa, ll, y(1), y(2)), ...
                        [0, intTime(1)], y0now, optionsNow );
                        if isempty(teNow)
                            tBwd = tNow(end);
                            thisPath2.aParallF.eventInfo{i}{1} = '';
                        else
                            thisPath2.aParallF.isValid(i) = false; % not a valid level-set
                            tBwd = teNow; ieBwd = ieNow;
                            switch ieBwd
                                case 1
                                    thisPath2.aParallF.eventInfo{i}{1} = [...
                                                                'shape bounds ' ...
                                                                    'violation'...
                                                                            ];
                                case 2
                                    thisPath2.aParallF.eventInfo{i}{1} = [...
                                                        'self-connected sets' ...
                                                                    ' violation'...
                                                                            ];
                            end
                        end
                        % ... integrate full path from end of backward path
                        tMain = tBwd + intTime(2);
                        yNow = yNow(tNow <= tBwd, :);
                        [tNow, yNow, teNow, ~, ieNow] = ...
                            ode89( @(t,y) +paraFdirn(aa, ll, y(1), y(2)), ...
                            linspace(0, tMain, size(ai, 1)),...
                            yNow(end, :), optionsNow );
                        if isempty(teNow)
                            tFull = tNow(end);
                            thisPath2.aParallF.eventInfo{i}{2} = '';
                        else
                            thisPath2.aParallF.isValid(i) = false; % not a valid level-set
                            tFull = teNow; ieFull = ieNow;
                            switch ieFull
                                case 1
                                    thisPath2.aParallF.eventInfo{i}{2} = [...
                                                                'shape bounds ' ...
                                                                    'violation'...
                                                                            ];
                                case 2
                                    thisPath2.aParallF.eventInfo{i}{2} = [...
                                                        'self-connected sets' ...
                                                                    ' violation'...
                                                                            ];
                            end
                        end
                        % ... offset the time array such that the reference point
                        % ... provided by 'aPerpF', 'y0now' is at time == 0
                        % ... do this for both 'tFull' (cutoff threshold) and the
                        % ... 'tNow' time vector
                        tNow = tNow - tBwd; tFull = tFull - tBwd;
                        % ... store the main solution
                        thisPath2.aParallF.y{i} = yNow(tNow <= tFull, :);
                        thisPath2.aParallF.t{i} = tNow(tNow <= tFull); % final solution
                        % ... store the maximum times until shape bounds
                        thisPath2.aParallF.tMax{i} = tMax;
                        % ... compute the stratified panel along this solution
                        thisPath2.aParallF.dz{i} = dz(aa, ll,...
                                            thisPath2.aParallF.y{i}(:, 1), ...
                                            thisPath2.aParallF.y{i}(:, 2));
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                case 'branched'
                    thisPath2.aParallF = [];
                    for bItr = 1:numel(yPerp)
                        % init the current perpendicular coordinates
                        yPerpNow = yPerp{bItr};
                        tPerpNow = tPerp{bItr};
                        % rest of the computation identical to the legacy
                        % mode
                        thisPath2.aParallF.isValid{bItr} = true(size(tPerpNow));
                        thisPath2.aParallF.eventInfo{bItr} = cell(size(tPerpNow));
                        thisPath2.aParallF.y{bItr} = cell(size(tPerpNow));
                        thisPath2.aParallF.t{bItr} = thisPath2.aParallF.y{bItr};
                        thisPath2.aParallF.dz{bItr} = thisPath2.aParallF.y{bItr};
                        argStruct = []; argStruct.functions = [];
                        argStruct.parameters = [];
                        argStruct.bounds.alphaLimits = aLimits;
                        if ~strcmp(phaseReq, 'NoPhaseLimits')
                            argStruct.bounds.angleThreshold = deg2rad(5);
                        end
                        for i = 1:numel(thisPath2.aParallF.y{bItr})
                            thisPath2.aParallF.eventInfo{bItr}{i} = cell(1, 2);
                            y0now = yPerpNow(i, :);
                            if ~strcmp(phaseReq, 'NoPhaseLimits')
                                argStruct.parameters.yExtremal=yExtremal(i,:);
                                argStruct.parameters.yRefPhase=yRefPhase(i);
                            end
                            optionsNow = odeset(  ...
                                'Events', ...
                                @(t, y) nonslipShapeCoordsEvents(t, y, argStruct, ...
                                eventList) ...
                                );
                            tMax = nan(1, 2);
                            [~, ~, teNow, ~, ~] = ...
                            ode89( @(t,y) -paraFdirn(aa, ll, y(1), y(2)), ...
                            [0, 1e2], y0now, optionsNow );
                            if ~isempty(teNow)
                                tMax(1) = teNow;
                            end
                            [~, ~, teNow, ~, ~] = ...
                            ode89( @(t,y) +paraFdirn(aa, ll, y(1), y(2)), ...
                            [0, 1e2], y0now, optionsNow );
                            if ~isempty(teNow)
                                tMax(2) = teNow;
                            end
                            [tNow, yNow, teNow, ~, ieNow] = ...
                            ode89( @(t,y) -paraFdirn(aa, ll, y(1), y(2)), ...
                            [0, intTime(1)], y0now, optionsNow );
                            if isempty(teNow)
                                tBwd = tNow(end);
                                thisPath2.aParallF.eventInfo{bItr}{i}{1} = '';
                            else
                                thisPath2.aParallF.isValid{bItr}(i) = false;
                                tBwd = teNow; ieBwd = ieNow;
                                switch ieBwd
                                    case 1
                                        thisPath2.aParallF.eventInfo{bItr}{i}{1} = [...
                                                                    'shape bounds ' ...
                                                                        'violation'...
                                                                                ];
                                    case 2
                                        thisPath2.aParallF.eventInfo{bItr}{i}{1} = [...
                                                            'self-connected sets' ...
                                                                        ' violation'...
                                                                                ];
                                end
                            end
                            tMain = tBwd + intTime(2);
                            yNow = yNow(tNow <= tBwd, :);
                            [tNow, yNow, teNow, ~, ieNow] = ...
                                ode89( @(t,y) +paraFdirn(aa, ll, y(1), y(2)), ...
                                linspace(0, tMain, size(ai, 1)),...
                                yNow(end, :), optionsNow );
                            if isempty(teNow)
                                tFull = tNow(end);
                                thisPath2.aParallF.eventInfo{bItr}{i}{2} = '';
                            else
                                thisPath2.aParallF.isValid{bItr}(i) = false;
                                tFull = teNow; ieFull = ieNow;
                                switch ieFull
                                    case 1
                                        thisPath2.aParallF.eventInfo{bItr}{i}{2} = [...
                                                                    'shape bounds ' ...
                                                                        'violation'...
                                                                                ];
                                    case 2
                                        thisPath2.aParallF.eventInfo{bItr}{i}{2} = [...
                                                            'self-connected sets' ...
                                                                        ' violation'...
                                                                                ];
                                end
                            end
                            tNow = tNow - tBwd; tFull = tFull - tBwd;
                            thisPath2.aParallF.y{bItr}{i} = yNow(tNow <= tFull, :);
                            thisPath2.aParallF.t{bItr}{i} = tNow(tNow <= tFull);
                            thisPath2.aParallF.tMax{bItr}{i} = tMax;
                            thisPath2.aParallF.dz{bItr}{i} = dz(aa, ll,...
                                    thisPath2.aParallF.y{bItr}{i}(:, 1), ...
                                    thisPath2.aParallF.y{bItr}{i}(:, 2));
                        end
                    end
            end
        end

        % find the complimentary submanifold
        % ... to analyze the two-beat quadrupedal gait, we need to
        % ... compute similar properties for the other complementary 
        % ... stance phase in the mutually exclusive shape subspace
        % ... eg::: if 1,3 (FR-HL stance), we need similar data by
        % ... creating an object for 2,4 (FL-HR stance).
        % ... the function obtains the complementary submanifold index,
        % ... and calls the constructor creating the new object
        function [ thatPath2 ] = constructComplementaryStance(thisPath2,...
                                        kin, kinfunc, p_kin, p_info, ...
                                        intTime, refPt, phaseReq, slipMode)
            % compute the complimentary submanifold index
            temp = zeros(1, 2); temp(thisPath2.sCol) = 1;
            thatCol = find(~temp);
            sThatIdx = kin.cs_IdxFor2bGaits(thisPath2.sRow, thatCol);
            % construct the complimentary submanifold "Path2_Mobility" obj
            thatPath2 = Path2_Mobility(sThatIdx,...
                                       kin, kinfunc, p_kin, p_info,...
                                       intTime, refPt, phaseReq, slipMode);
        end

        % check if the provided stance submanifolds are complementary
        function checkFlag = areComplementaryStances(thisPath2, thatPath2)
            % compute the complimentary submanifold index
            temp = zeros(1, 2); temp(thisPath2.sCol) = 1;
            thatColEstimate = find(~temp);
            % obtain the actual submanifold index
            thatColActual = thatPath2.sCol;
            % compare the two and return the flag
            checkFlag = (thatColEstimate == thatColActual);
        end

        % find the parallel coordinates specifically at a specific point
        % ... this function is setup to compute the nonslip directions for
        % ... the specified integration time about the refernce point and
        % ... all of this information is stored in the 'aParaRef' property
        % ... Everytime this function is called with new arguments, the
        % ... corresponding property fields get updated and replaced
        % ... for more information on the exact computations performed,
        % ... refer to "computeParallelCoordinates" method defined above.
        % ... The "fullFLvlSetFlag" argument chooses whether we want to 
        % ... integrate for the path-lengths specified in "intTime" 
        % ... argument or we want the entire level-set.
        function thisPath2 = computeSpecificParallelCoordinates...
                            ( thisPath2, refPt, intTime, fullFLvlSetFlag )
            if nargin < 4
                fullFLvlSetFlag = false;
            end
            if nargin < 3
                intTime = thisPath2.intTime;
            end
            if nargin < 2
                refPt = thisPath2.refPt;
            end
            if nargin > 4 || nargin == 0
                error('ERROR! Incorrect number of arguments.');
            end
            if isempty(intTime)
                intTime = thisPath2.intTime;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % unpacking is similar to the overall parallel coordinates
            % function
            aa = thisPath2.a; ll = thisPath2.l;
            aInf = thisPath2.aInf; aSup = thisPath2.aSup;
            ai = thisPath2.ai; aLimits = thisPath2.aLimits;
            paraFdirn = thisPath2.paraFdirn;
            F_fxn = thisPath2.F_fxn;
            dz = thisPath2.dz;
            phaseReq = thisPath2.phaseReq;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            argStruct = []; argStruct.functions = []; 
                argStruct.parameters = [];
            argStruct.bounds.alphaLimits = aLimits;
            thisPath2.aParaRef = [];
            thisPath2.aParaRef.refPt = refPt;
            thisPath2.aParaRef.isValid = true;
            thisPath2.aParaRef.eventInfo = cell(1, 2);
            switch phaseReq
                case "NoPhaseLimits"
                    eventList = {'shape_bounds'};
                otherwise
                    eventList = {'shape_bounds', 'phase_bounds'};
                    tInt = 100*[1 1];
                    [~, yNow] = ode23(@(t, y) -paraFdirn(aa, ll, y(1), y(2)),...
                        [0 tInt(1)], refPt);
                    [~, yNow] = ode23(@(t, y) +paraFdirn(aa, ll, y(1), y(2)),...
                        [0 sum(tInt)], yNow(end, :));
                    errInf = repmat(aInf, size(yNow, 1), 1) - yNow; 
                    errInf = min(vecnorm(errInf, 2, 2));
                    errSup = repmat(aSup, size(yNow, 1), 1) - yNow; 
                    errSup = min(vecnorm(errSup, 2, 2));
                    switch errInf < errSup
                        case 1
                            yExtremal = aInf; 
                        case 0
                            yExtremal = aSup;
                    end
                    yPhasorNow = (refPt - yExtremal)/norm(refPt - yExtremal); 
                    yRefPhase = atan2(yPhasorNow(2), yPhasorNow(1));
                    argStruct.bounds.angleThreshold = deg2rad(5);
                    argStruct.parameters.yExtremal = yExtremal; 
                    argStruct.parameters.yRefPhase = yRefPhase;
            end
            options = odeset(  ...
                'Events', ...
                @(t, y) nonslipShapeCoordsEvents(t, y, argStruct, ...
                eventList) ...
                );
            tMax = nan(1, 2);
            [~, ~, teNow, ~, ~] = ...
            ode89( @(t,y) -paraFdirn(aa, ll, y(1), y(2)), ...
                        [0, 1e2], refPt, options );
            if ~isempty(teNow)
                tMax(1) = teNow;
            end
            [~, ~, teNow, ~, ~] = ...
            ode89( @(t,y) +paraFdirn(aa, ll, y(1), y(2)), ...
            [0, 1e2], refPt, options );
            if ~isempty(teNow)
                tMax(2) = teNow;
            end
            % ... % ... % ... % ... % ... % ... % ... % ... % ... % ... 
            % ... if we need the whole level-set, then we modify the
            % ... "intTime" with 99% of tMax's range (remove 0.5% on either
            % ... end to not trigger the events)
            % ... this is the only difference between the full parallel
            % ... coordinates computation for each point in the slipping
            % ... coordinates
            if fullFLvlSetFlag
                intTime = tMax - 1e-2/2*sum(tMax);
            end
            % ... % ... % ... % ... % ... % ... % ... % ... % ... % ... 
            [tNow, yNow, teNow, ~, ieNow] = ...
            ode89( @(t,y) -paraFdirn(aa, ll, y(1), y(2)), ...
            [0, intTime(1)], refPt, options );
            if isempty(teNow)
                tBwd = tNow(end);
                thisPath2.aParaRef.eventInfo{1} = '';
            else
                thisPath2.aParaRef.isValid = false;
                tBwd = teNow; ieBwd = ieNow;
                switch ieBwd
                    case 1
                        thisPath2.aParaRef.eventInfo{1} = [...
                                                    'shape bounds ' ...
                                                        'violation'...
                                                                ];
                    case 2
                        thisPath2.aParaRef.eventInfo{1} = [...
                                            'self-connected sets' ...
                                                        ' violation'...
                                                                ];
                end
            end
            tMain = tBwd + intTime(2);
            yNow = yNow(tNow <= tBwd, :);
            [tNow, yNow, teNow, ~, ieNow] = ...
                ode89( @(t,y) +paraFdirn(aa, ll, y(1), y(2)), ...
                linspace(0, tMain, size(ai, 1)),...
                yNow(end, :), options );
            if isempty(teNow)
                tFull = tNow(end);
                thisPath2.aParaRef.eventInfo{2} = '';
            else
                thisPath2.aParaRef.isValid = false;
                tFull = teNow; ieFull = ieNow;
                switch ieFull
                    case 1
                        thisPath2.aParaRef.eventInfo{2} = [...
                                                    'shape bounds ' ...
                                                        'violation'...
                                                                ];
                    case 2
                        thisPath2.aParaRef.eventInfo{2} = [...
                                            'self-connected sets' ...
                                                        ' violation'...
                                                                ];
                end
            end
            tNow = tNow - tBwd; tFull = tFull - tBwd;
            thisPath2.aParaRef.y = yNow(tNow <= tFull, :);
            thisPath2.aParaRef.t = tNow(tNow <= tFull);
            thisPath2.aParaRef.tMax = tMax;
            thisPath2.aParaRef.dz = dz(aa, ll,...
                                    thisPath2.aParaRef.y(:, 1), ...
                                    thisPath2.aParaRef.y(:, 2));
            % ... compute the velocity and acceleration of the 
            % ... shape trajectory
            % ... ... just first order differences for now
            pthOrder = 1; % two-norm for now (identity inner product of 
                          % velocity and accelerations in the shape space)
            thisPath2.aParaRef.aVel = ...
                vecnorm(diff(thisPath2.aParaRef.y)./...
                    repmat(diff(thisPath2.aParaRef.t), 1, 2), ...
                    pthOrder, 2); % compute along the column with last arg
            thisPath2.aParaRef.aAccln = ...
                vecnorm(diff(thisPath2.aParaRef.aVel)./...
                repmat(diff(thisPath2.aParaRef.t(1:end-1)), 1, 2), ...
                pthOrder, 2);
            F_ref = F_fxn(aa, ll, refPt(1), refPt(2));
            thisPath2.aParaRef.F = F_ref;
            thisPath2.aParaRef.color = ...
                interpColorAndCondition...
                (...
                linspace(thisPath2.F_inf, thisPath2.F_sup,...
                                    size(thisPath2.p_info.jetDark, 1)), ...
                thisPath2.p_info.jetDark, F_ref...
                 );
        end

        % check if a given point is inside the accessible shape space
        % ... for this, we use the "nonslipShapeCoordsEvents.m" EVENT
        % ... function in the shape bounds mode to check if the queried
        % ... point is inside or outside the shape space bounds
        % ... for more information, refer to 
        % ... "computePerpendicularCoordinates" or 
        % ... "computeParallelCoordinates" methods defined above
        function flag = checkInsideAccessibleShapeSpace(thisPath2, queryPt)
            argStruct = []; 
            argStruct.parameters = []; 
                argStruct.bounds = []; argStruct.functions = [];
            argStruct.bounds.alphaLimits = thisPath2.aLimits;
            switch ( ...
                    nonslipShapeCoordsEvents...
                    (0, queryPt, argStruct, {'shape_bounds'}) > 0 ...
                   )
                case 1
                    flag = false;
                case 0
                    flag = true;
            end
        end
        
        % this function plpots the computed nonslip level sets from the
        % slip-nonslip coordinates
        function visualizeNonslipLevelSets(thisPath2)
            % unpack information for plotting
            % ... basic plot info and instance props
            pInfo = thisPath2.p_info; 
            lW = pInfo.lW; 
            colLimits = [thisPath2.F_inf, thisPath2.F_sup];
            stanceColor = pInfo.gc_col;
            scatSingSize = pInfo.circS;
            % plot the figure .............................................
            figure('Visible', 'on', 'Units', 'pixels',...
                'Position', [0 0 600 600]); ax = gca; box(ax, "on");
            ax.XColor = stanceColor; ax.YColor = stanceColor;
            axis(ax, "equal", "tight"); hold(ax, "on"); view(2);
            set(ax, 'Color',pInfo.col_backg);
            % ... plot the singularity if within accessible shape space
            if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aSup)
                scatter(ax, thisPath2.aSup(1), thisPath2.aSup(2), 0.5*scatSingSize, ...
                    'MarkerEdgeColor', pInfo.jetDark(end, :), 'MarkerFaceColor', 'none', ...
                    'LineWidth', lW);
            end
            if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aInf)
                scatter(ax, thisPath2.aInf(1), thisPath2.aInf(2), 0.5*scatSingSize, ...
                    'MarkerEdgeColor', pInfo.jetDark(1, :), 'MarkerFaceColor', 'none', ...
                    'LineWidth', lW);
            end
            switch thisPath2.slipMode
                case 'legacy'
                    % ... find starting and ending points along the slip axis for valid
                    % ... level-sets of F
                    startIdx = find(thisPath2.aParallF.isValid, 1, 'first');
                    endIdx = find(thisPath2.aParallF.isValid, 1, 'last');
                    % ... contourf plot the level-sets of F
                    % ... ... basic, level-set based plot method
                    % ... ... providing the alpha value at the end helps with this
                    for i = startIdx:endIdx
                        plot(ax, thisPath2.aParallF.y{i}(:, 1), ...
                            thisPath2.aParallF.y{i}(:, 2), 'LineWidth', lW,...
                            'Color', thisPath2.aPerpF.color(i, :));
                    end
                case 'branched'
                    for bItr = 1:numel(thisPath2.aParallF.y)
                        startIdx = find(thisPath2.aParallF.isValid{bItr},...
                                                            1, 'first');
                        endIdx = find(thisPath2.aParallF.isValid{bItr}, ...
                                                            1, 'last');
                        for i = startIdx:endIdx
                            plot(ax, thisPath2.aParallF.y{bItr}{i}(:, 1), ...
                                thisPath2.aParallF.y{bItr}{i}(:, 2), ...
                                'LineWidth', lW,...
                                'Color', thisPath2.aPerpF.color{bItr}(i, :));
                        end
                    end
            end
            % ... setup rest of the figure
            colormap(ax, pInfo.jetDark); clim(ax, colLimits); 
            colorbar(ax, 'TickLabelInterpreter', 'latex',...
                'FontSize', pInfo.cbarFS);
            set(get(ax, 'YLabel'),'rotation',0,'VerticalAlignment','middle');
            xticks(ax, pInfo.xtickval); yticks(ax, pInfo.ytickval);
            xticklabels(ax, pInfo.xticklab); yticklabels(ax, pInfo.yticklab);
            xlabel(ax, pInfo.x_label_txt,FontSize=pInfo.labelFS); 
                        ylabel(ax, pInfo.y_label_txt,FontSize=pInfo.labelFS);
            ax.XAxis.FontSize = pInfo.tickFS; 
                        ax.YAxis.FontSize = pInfo.tickFS;
            xlim(pInfo.xlimits); ylim(pInfo.ylimits);
        end
        
        % this function plpots the computed nonslip level sets from the
        % slip-nonslip coordinates and highlights the axes of the two
        % coordinates
        function visualizeSlipNonslipShapeCoordinates(thisPath2)
            % unpack information for plotting
            % ... basic plot info and instance props
            pInfo = thisPath2.p_info; 
            lW = pInfo.lW; 
            colLimits = [thisPath2.F_inf, thisPath2.F_sup];
            stanceColor = pInfo.gc_col;
            scatSingSize = pInfo.circS;
            domainPercentage = 5; arrowAngle = deg2rad(18);
            arrowSize = domainPercentage/100*...
                            mean(diff(thisPath2.aLimits, 1, 2), 1)*...
                                                sum(thisPath2.intTime)/2;
            % ... setup a linealpha value to make the slip and nonslip axes
            % ... more visible
            % ... lA: lineAlpha, c: contours
            lAc = 0.5; % init
            if ~isfield(thisPath2.p_info, 'fAc')
                thisPath2.p_info.fAc = lAc; % store if not present
            end
            % .............................................................
            figure('Visible', 'on', 'Units', 'pixels',...
                'Position', [0 0 600 600]); ax = gca; box(ax, "on");
            ax.XColor = stanceColor; ax.YColor = stanceColor;
            axis(ax, "equal", "tight"); hold(ax, "on"); view(2);
            set(ax, 'Color',pInfo.col_backg);
            if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aSup)
                scatter(ax, thisPath2.aSup(1), thisPath2.aSup(2), 0.5*scatSingSize, ...
                    'MarkerEdgeColor', pInfo.jetDark(end, :), 'MarkerFaceColor', 'none', ...
                    'LineWidth', lW);
            end
            if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aInf)
                scatter(ax, thisPath2.aInf(1), thisPath2.aInf(2), 0.5*scatSingSize, ...
                    'MarkerEdgeColor', pInfo.jetDark(1, :), 'MarkerFaceColor', 'none', ...
                    'LineWidth', lW);
            end
            switch thisPath2.slipMode

                case 'legacy'
                    startIdx = find(thisPath2.aParallF.isValid, 1, 'first');
                    endIdx = find(thisPath2.aParallF.isValid, 1, 'last');
                    for i = startIdx:endIdx
                        plot(ax, thisPath2.aParallF.y{i}(:, 1), ...
                            thisPath2.aParallF.y{i}(:, 2), ...
                            'LineWidth', lW,...
                            'Color', [thisPath2.aPerpF.color(i, :), lAc]);
                    end
                    % ... plot the slip axis
                    % ... ... first plot the axis and then add an arrow
                    plot(ax, thisPath2.aPerpF.y(startIdx:endIdx, 1), ...
                        thisPath2.aPerpF.y(startIdx:endIdx, 2), ...
                        'LineWidth', lW,...
                        'Color', stanceColor);
                    plotPathArrowV2(ax, ...
                        thisPath2.aPerpF.y(startIdx:endIdx, 1), ...
                        thisPath2.aPerpF.y(startIdx:endIdx, 2),...
                        arrowSize, arrowAngle, lW, stanceColor, 'front_end');
                    % ... plot the nonslip axis
                    plot(ax, thisPath2.aParaRef.y(:, 1), ...
                        thisPath2.aParaRef.y(:, 2), 'LineWidth', lW,...
                        'Color', stanceColor);
                    plotPathArrowV2(ax, thisPath2.aParaRef.y(:, 1), ...
                        thisPath2.aParaRef.y(:, 2),...
                        arrowSize, arrowAngle, lW, stanceColor, 'front_end');

                case 'branched'
                    for bItr = 1:numel(thisPath2.aParallF.y)
                        startIdx = find(thisPath2.aParallF.isValid{bItr}, 1, 'first');
                        endIdx = find(thisPath2.aParallF.isValid{bItr}, 1, 'last');
                        for i = startIdx:endIdx
                            plot(ax, thisPath2.aParallF.y{bItr}{i}(:, 1), ...
                                thisPath2.aParallF.y{bItr}{i}(:, 2), ...
                                'LineWidth', lW,...
                                'Color', [thisPath2.aPerpF.color{bItr}(i, :), lAc]);
                        end
                        % ... plot the slip axis
                        % ... ... first plot the axis and then add an arrow
                        plot(ax, thisPath2.aPerpF.y{bItr}(startIdx:endIdx, 1), ...
                            thisPath2.aPerpF.y{bItr}(startIdx:endIdx, 2), ...
                            'LineWidth', lW,...
                            'Color', stanceColor);
                        plotPathArrowV2(ax, ...
                            thisPath2.aPerpF.y{bItr}(startIdx:endIdx, 1), ...
                            thisPath2.aPerpF.y{bItr}(startIdx:endIdx, 2),...
                            arrowSize, arrowAngle, lW, stanceColor, 'front_end');
                        % ... plot the nonslip axis
                        plot(ax, thisPath2.aParaRef.y(:, 1), ...
                            thisPath2.aParaRef.y(:, 2), 'LineWidth', lW,...
                            'Color', stanceColor);
                        plotPathArrowV2(ax, thisPath2.aParaRef.y(:, 1), ...
                            thisPath2.aParaRef.y(:, 2),...
                            arrowSize, arrowAngle, lW, stanceColor, 'front_end');
                    end
            end
            colormap(ax, pInfo.jetDark); clim(ax, colLimits); 
            colorbar(ax, 'TickLabelInterpreter', 'latex',...
                'FontSize', pInfo.cbarFS);
            set(get(ax, 'YLabel'),'rotation',0,'VerticalAlignment','middle');
            xticks(ax, pInfo.xtickval); yticks(ax, pInfo.ytickval);
            xticklabels(ax, pInfo.xticklab); yticklabels(ax, pInfo.yticklab);
            xlabel(ax, pInfo.x_label_txt,FontSize=pInfo.labelFS); 
                        ylabel(ax, pInfo.y_label_txt,FontSize=pInfo.labelFS);
            ax.XAxis.FontSize = pInfo.tickFS; 
                        ax.YAxis.FontSize = pInfo.tickFS;
            xlim(pInfo.xlimits); ylim(pInfo.ylimits);
            % .............................................................
        end
        
        % this function plots highlights F level-set solution at the chosen 
        % point
        function highlightNonslipLevelSet(thisPath2, refPtNow)
            % obtain the level-set at the requested point
            % ... we are creating a temporary instance because we don't
            % ... want to overwrite existing properties
            thisPath2Now = ... % copying the instance with new properties
                Path2_Mobility.computeSpecificParallelCoordinates...
                                            ( thisPath2, refPtNow );
            aParaRefNow = thisPath2Now.aParaRef;
            pInfo = thisPath2.p_info; 
            lW = pInfo.lW; 
            colLimits = [thisPath2.F_inf, thisPath2.F_sup];
            stanceColor = pInfo.gc_col;
            scatSingSize = pInfo.circS;
            lAc = 0.6;
            if ~isfield(thisPath2.p_info, 'fAc')
                thisPath2.p_info.fAc = lAc;
            end
            % .............................................................
            figure('Visible', 'on', 'Units', 'pixels',...
                'Position', [0 0 600 600]); ax = gca; box(ax, "on");
            ax.XColor = stanceColor; ax.YColor = stanceColor;
            axis(ax, "equal", "tight"); hold(ax, "on"); view(2);
            set(ax, 'Color',pInfo.col_backg);
            if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aSup)
                scatter(ax, thisPath2.aSup(1), thisPath2.aSup(2), 0.5*scatSingSize, ...
                    'MarkerEdgeColor', pInfo.jetDark(end, :), 'MarkerFaceColor', 'none', ...
                    'LineWidth', lW);
            end
            if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aInf)
                scatter(ax, thisPath2.aInf(1), thisPath2.aInf(2), 0.5*scatSingSize, ...
                    'MarkerEdgeColor', pInfo.jetDark(1, :), 'MarkerFaceColor', 'none', ...
                    'LineWidth', lW);
            end
            switch thisPath2.slipMode
                case 'legacy'
                    startIdx = find(thisPath2.aParallF.isValid, 1, 'first');
                    endIdx = find(thisPath2.aParallF.isValid, 1, 'last');
                    for i = startIdx:endIdx
                        plot(ax, thisPath2.aParallF.y{i}(:, 1), ...
                            thisPath2.aParallF.y{i}(:, 2), ...
                            'LineWidth', lW,...
                            'Color', [thisPath2.aPerpF.color(i, :), lAc]);
                    end
                case 'branched'
                    for bItr = 1:numel(thisPath2.aParallF.y)
                        startIdx = find(thisPath2.aParallF.isValid{bItr}, 1, 'first');
                        endIdx = find(thisPath2.aParallF.isValid{bItr}, 1, 'last');
                        for i = startIdx:endIdx
                            plot(ax, thisPath2.aParallF.y{bItr}{i}(:, 1), ...
                                thisPath2.aParallF.y{bItr}{i}(:, 2), ...
                                'LineWidth', lW,...
                                'Color', ...
                                [thisPath2.aPerpF.color{bItr}(i, :), lAc]);
                        end
                    end
            end
            % ... highlight the requested level-set
            plot(ax, aParaRefNow.y(:, 1), aParaRefNow.y(:, 2), 'LineWidth', lW,...
                'Color', stanceColor);
            colormap(ax, pInfo.jetDark); clim(ax, colLimits); 
            colorbar(ax, 'TickLabelInterpreter', 'latex',...
                'FontSize', pInfo.cbarFS);
            set(get(ax, 'YLabel'),'rotation',0,'VerticalAlignment','middle');
            xticks(ax, pInfo.xtickval); yticks(ax, pInfo.ytickval);
            xticklabels(ax, pInfo.xticklab); yticklabels(ax, pInfo.yticklab);
            xlabel(ax, pInfo.x_label_txt,FontSize=pInfo.labelFS); 
                        ylabel(ax, pInfo.y_label_txt,FontSize=pInfo.labelFS);
            ax.XAxis.FontSize = pInfo.tickFS; 
                        ax.YAxis.FontSize = pInfo.tickFS;
            xlim(pInfo.xlimits); ylim(pInfo.ylimits);
            % .............................................................
        end

        % this function plots the requested stance phase path parameterized
        % by the starting and stopping times relative to a reference point 
        % in the limb angle subspace as done by the visualization functions 
        % defined earlier.
        function plotStanceOnNonslipLevelSets...
                (ref, startEndTimes, thisPath2, shapeStanceTraj, status)
            % .............................................................
            % create a local instanace to highlight the current level-set
            refPtNow = ref.P;
            thisPath2Now = ... 
                Path2_Mobility.computeSpecificParallelCoordinates...
                                            ( thisPath2, refPtNow );
            aParaRefNow = thisPath2Now.aParaRef;
            % .............................................................
            % create the stance path that needs to be plotted
            % ... 1) we dont have the stance path
            % ... 2) we do have the stance path, and we pass it along to 
            % ... compute the entire subgait
            if nargin < 4
                [tau, ~, fullPath] = ...
                            Path2_Mobility.computeSubgaitInCoordinates...
                                                (ref, startEndTimes, thisPath2);
            elseif nargin == 4
                [tau, ~, fullPath] = ...
                    Path2_Mobility.computeSubgaitInCoordinates...
                        (ref, startEndTimes, thisPath2, shapeStanceTraj);
            elseif nargin == 5
                [tau, ~, fullPath] = ...
                    Path2_Mobility.computeSubgaitInCoordinates...
                                        (ref, startEndTimes, thisPath2, ...
                                                shapeStanceTraj, status);
            else
                error(['ERROR! This function only accepts 5 args. Refer to ' ...
                    'usage in "se2_toyproblems_case_1_mobility.mlx" for ' ...
                    'more details.'])
            end
            tauStanceQ = ... % phase during stance
                linspace(0, pi, floor(size(tau, 1)/2)); 
            stancePath = interp1(tau, fullPath, tauStanceQ, "pchip");
            % .............................................................
            % plotting related
            pInfo = thisPath2.p_info; 
            lW = pInfo.lW; lWc = pInfo.lW_contour;
            colLimits = [thisPath2.F_inf, thisPath2.F_sup];
            stanceColor = pInfo.gc_col;
            scatSingSize = pInfo.circS;
            fAc = 0.6; fAcSp = 0.8;
            if ~isfield(thisPath2.p_info, 'fAc')
                thisPath2.p_info.fAc = fAc;
            end
            domainPercentage = 3; arrowAngle = deg2rad(18);
            arrowSize = domainPercentage/100*...
                            mean(diff(thisPath2.aLimits, 1, 2), 1)*...
                                                sum(thisPath2.intTime)/2;
            % .............................................................
            % ... 1) basic setup
            % ... 2) plot the singularity at the sup(F) point if within the
            % ... accessible shape space
            % ... 3) do the same for the inf(F) point
            % ... 4) iterate and plot each level-set in the slip-nonslip
            % ... coordinate system
            figure('Visible', 'on', 'Units', 'pixels', ...
                'Position', [0 0 600 600]); 
            ax = gca; box(ax, "on");
            ax.XColor = stanceColor; ax.YColor = stanceColor;
            axis(ax, "equal", "tight"); hold(ax, "on"); view(2);
            set(ax, 'Color', pInfo.col_backg);
            if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aSup)
                scatter(ax, thisPath2.aSup(1), thisPath2.aSup(2), 0.5*scatSingSize, ...
                    'MarkerEdgeColor', pInfo.jetDark(end, :), 'MarkerFaceColor', 'none', ...
                    'LineWidth', lW);
            end
            if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aInf)
                scatter(ax, thisPath2.aInf(1), thisPath2.aInf(2), 0.5*scatSingSize, ...
                    'MarkerEdgeColor', pInfo.jetDark(1, :), 'MarkerFaceColor', 'none', ...
                    'LineWidth', lW);
            end
            switch thisPath2.slipMode
                case 'legacy'
                    startIdx = find(thisPath2.aParallF.isValid, 1, 'first');
                    endIdx = find(thisPath2.aParallF.isValid, 1, 'last');
                    for i = startIdx:endIdx
                        plot(ax, ...
                            thisPath2.aParallF.y{i}(:, 1), ...
                            thisPath2.aParallF.y{i}(:, 2), 'LineWidth', lW,...
                            'Color', [thisPath2.aPerpF.color(i, :), fAc]);
                    end
                case 'branched'
                    for bItr = 1:numel(thisPath2.aParallF.y)
                        startIdx = find(thisPath2.aParallF.isValid{bItr},...
                            1, 'first');
                        endIdx = find(thisPath2.aParallF.isValid{bItr},...
                            1, 'last');
                        for i = startIdx:endIdx
                            plot(ax, ...
                                thisPath2.aParallF.y{bItr}{i}(:, 1), ...
                                thisPath2.aParallF.y{bItr}{i}(:, 2), ...
                                'LineWidth', lW,...
                                'Color', ...
                                [thisPath2.aPerpF.color{bItr}(i, :), fAc]);
                        end
                    end
            end
            % ... 1) the current F level-set the stance path belongs to
            % ... 2) the current stance path before plotting scatters
            % ... 3) the reference point scatter
            % ... 4) the stance path scatters: (a) a 'o' scatter at the 
            % ... starting point, (b) an 'x' at the ending point, and 
            % ... (c) an arrow in the middle of the path to denote its 
            % ... direction.
            plot(ax, aParaRefNow.y(:, 1), aParaRefNow.y(:, 2), 'LineWidth', lW,... % 1
                'Color', [aParaRefNow.color, fAcSp]);
            plot(ax, stancePath(:, 1), stancePath(:, 2), 'LineWidth', lW,... % stance path 2
                'Color', stanceColor);
            scatter(ax, refPtNow(1), refPtNow(2), scatSingSize, ... % reference point scatter 3
                'Marker', 'pentagram', ...
                'MarkerEdgeColor', 'k', 'MarkerFaceColor', stanceColor, ...
                'LineWidth', lWc);
            scatter(ax, stancePath(1, 1), stancePath(1, 2), 0.25*scatSingSize, ... % starting point scatter 4(a)
                'MarkerEdgeColor', 'k', 'MarkerFaceColor', stanceColor, ...
                'LineWidth', lWc);
            scatter(ax, stancePath(end, 1), stancePath(end, 2), 0.25*scatSingSize, ... % ending point scatter 4(b)
                'Marker', 'x', ...
                'MarkerEdgeColor', 'k', 'MarkerFaceColor', stanceColor, ...
                'LineWidth', lW);
            plotPathArrowV2(ax, stancePath(:, 1), stancePath(:, 2),... % arrow for direction 4(c)
                arrowSize, arrowAngle, lW-1, 'k', 'front_end'); % , stanceColor, 'mid'
            % .............................................................
            colormap(ax, pInfo.jetDark); clim(ax, colLimits); 
            colorbar(ax, 'TickLabelInterpreter', 'latex',...
                'FontSize', pInfo.cbarFS);
            set(get(ax, 'YLabel'),'rotation',0,'VerticalAlignment','middle');
            xticks(ax, pInfo.xtickval); yticks(ax, pInfo.ytickval);
            xticklabels(ax, pInfo.xticklab); yticklabels(ax, pInfo.yticklab);
            xlabel(ax, pInfo.x_label_txt,FontSize=pInfo.labelFS); 
                        ylabel(ax, pInfo.y_label_txt,FontSize=pInfo.labelFS);
            ax.XAxis.FontSize = pInfo.tickFS; 
                        ax.YAxis.FontSize = pInfo.tickFS;
            xlim(pInfo.xlimits); ylim(pInfo.ylimits);
            % .............................................................
        end

        % this function plots the requested stance phase path parameterized
        % by the starting and stopping times relative to a reference point 
        % in the limb angle subspace with panel heatmaps plotted as filled
        % contours (similar to previous visualization functions).
        % for specific details on how things are plotted, refer to the
        % comments in the previous function: "plotStanceOnNonslipLevelSets"
        function plotStanceOnPanels...
            (ref, startEndTimes, thisPath2, shapeStanceTraj, status, mode)
            % .............................................................
            % create a local instanace to highlight the current level-set
            refPtNow = ref.P;
            thisPath2Now = ... 
                Path2_Mobility.computeSpecificParallelCoordinates...
                                            ( thisPath2, refPtNow );
            aParaRefNow = thisPath2Now.aParaRef;
            % .............................................................
            % create the stance path that needs to be plotted
            % ... 1) we dont have the stance path
            % ... 2) we do have the stance path, and we pass it along to 
            % ... compute the entire subgait
            if nargin < 4
                [tau, ~, fullPath] = ...
                            Path2_Mobility.computeSubgaitInCoordinates...
                                                (ref, startEndTimes, thisPath2);
            elseif nargin == 4
                [tau, ~, fullPath] = ...
                    Path2_Mobility.computeSubgaitInCoordinates...
                        (ref, startEndTimes, thisPath2, shapeStanceTraj);
            elseif nargin == 5 || nargin == 6
                [tau, ~, fullPath] = ...
                    Path2_Mobility.computeSubgaitInCoordinates...
                                        (ref, startEndTimes, thisPath2, ...
                                                shapeStanceTraj, status);
            else
                error(['ERROR! This function only accepts 5 args. Refer to ' ...
                    'usage in "se2_toyproblems_case_1_mobility.mlx" for ' ...
                    'more details.'])
            end
            tauStanceQ = ... % phase during stance
                linspace(0, pi, floor(size(tau, 1)/2)); 
            stancePath = interp1(tau, fullPath, tauStanceQ, "pchip");
            % .............................................................
            % plotting related
            pInfo = thisPath2.p_info; 
            lW = pInfo.lW; lWc = pInfo.lW_contour;
            stanceColor = pInfo.gc_col;
            scatSingSize = pInfo.circS;
            % startIdx = find(thisPath2.aParallF.isValid, 1, 'first');
            % endIdx = find(thisPath2.aParallF.isValid, 1, 'last');
            fAc = 0.6; fAcSp = 0.8; cfLvl = pInfo.cfLvl; fA = pInfo.fA;
            if ~isfield(thisPath2.p_info, 'fAc')
                thisPath2.p_info.fAc = fAc;
            end
            domainPercentage = 3; arrowAngle = deg2rad(18);
            arrowSize = domainPercentage/100*...
                            mean(diff(thisPath2.aLimits, 1, 2), 1)*...
                                                sum(thisPath2.intTime)/2;
            % ... panel sweeps related
            ai = thisPath2.ai; aj = thisPath2.aj;
            componentString = {'x', 'y', 'theta'};
            panelSweeps = cell(1, numel(componentString));
            panelSweepLimits = cell(1, numel(componentString));
            minNow = []; maxNow = [];
            % ... ... iterate over each component
            for i = 1:numel(componentString)
                % ... ... ... extract component
                panelSweeps{i} = ...
                    thisPath2.p_kin.(['dz__' componentString{i} '_sweep']);
                % ... ... ... extract limits
                switch i
                    case numel(componentString)
                        minNow = min(panelSweeps{i}, [], "all");
                        maxNow = max(panelSweeps{i}, [], "all");
                    otherwise
                        minNow = [minNow, min(panelSweeps{i}, [], "all")];
                        maxNow = [maxNow, max(panelSweeps{i}, [], "all")];
                end
                % ... ... ... assign limits
                panelSweepLimits{i} = [min(minNow), max(maxNow)];
                if i == 2
                    panelSweepLimits{i-1} = panelSweepLimits{i};
                end
            end
            panelSweepAlpha = 0.8;
            % .............................................................
            % iterate over each panel and plot things accordingly
            switch mode
                case 'all'
                    f = figure('Visible', 'on', 'Units', 'pixels', ...
                        'Position', [0 0 1800 600]); 
                    tl = tiledlayout(f, 1, 3, ...
                        "TileSpacing", "compact", "Padding", "compact");
                    tlXY = tiledlayout(tl, 1, 2, ...
                        "TileSpacing", "compact", "Padding", "compact");
                    tlXY.Layout.Tile = 1; tlXY.Layout.TileSpan = [1, 2];
                    tlTh = tiledlayout(tl, 1, 1, ...
                        "TileSpacing", "compact", "Padding", "compact");
                    tlTh.Layout.Tile = 3; tlTh.Layout.TileSpan = [1, 1];
                    startCompIdx = 1; 
                    endCompIdx = numel(panelSweeps);
                case 'theta'
                    figure('Visible', 'on', 'Units', 'pixels', ...
                        'Position', [0 0 600 600]); 
                    startCompIdx = numel(panelSweeps); 
                    endCompIdx = startCompIdx;
            end
            for iComps = startCompIdx:endCompIdx
                switch startCompIdx == endCompIdx
                    case 1
                        ax = gca;
                    otherwise
                        switch iComps
                            case {1, 2}
                                ax = nexttile(tlXY);
                            otherwise
                                ax = nexttile(tlTh);
                        end
                end
                box(ax, "on"); 
                ax.XColor =stanceColor; ax.YColor = stanceColor;
                axis(ax, "equal", "tight"); hold(ax, "on"); view(2);
                set(ax, 'Color', pInfo.col_backg);
                % .........................................................
                % plot the panels as filled contours
                contourf(ax, ...
                         ai, aj, ... % shape subspace discretization
                         panelSweeps{iComps}, cfLvl, ... % panel
                         'LineStyle', 'none', ... % no contour lines
                         'FaceAlpha', panelSweepAlpha... % transparency
                         ); 
                % .........................................................
                if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aSup)
                    scatter(ax, thisPath2.aSup(1), thisPath2.aSup(2), ...
                        0.5*scatSingSize, ...
                        'MarkerEdgeColor', pInfo.jetDark(end, :), ...
                        'MarkerFaceColor', 'none', ...
                        'LineWidth', lW);
                end
                if Path2_Mobility.checkInsideAccessibleShapeSpace...
                                                (thisPath2, thisPath2.aInf)
                    scatter(ax, thisPath2.aInf(1), thisPath2.aInf(2), ...
                        0.5*scatSingSize, ...
                        'MarkerEdgeColor', pInfo.jetDark(1, :), ...
                        'MarkerFaceColor', 'none', ...
                        'LineWidth', lW);
                end
                plot(ax, aParaRefNow.y(:, 1), aParaRefNow.y(:, 2), ...
                    ':', ...
                    'LineWidth', lW,... % 1
                    'Color', [aParaRefNow.color, fAcSp]);
                plot(ax, stancePath(:, 1), stancePath(:, 2), ...
                    'LineWidth', lW,... % stance path 2
                    'Color', stanceColor);
                scatter(ax, refPtNow(1), refPtNow(2), ...
                    scatSingSize, ... % reference point scatter 3
                    'Marker', 'pentagram', ...
                    'MarkerEdgeColor', 'k', 'MarkerFaceColor', stanceColor, ...
                    'LineWidth', lWc);
                scatter(ax, stancePath(1, 1), stancePath(1, 2), ...
                    0.25*scatSingSize, ... % starting point scatter 4(a)
                    'MarkerEdgeColor', 'k', 'MarkerFaceColor', stanceColor, ...
                    'LineWidth', lWc);
                scatter(ax, stancePath(end, 1), stancePath(end, 2), ...
                    0.25*scatSingSize, ... % ending point scatter 4(b)
                    'Marker', 'x', ...
                    'MarkerEdgeColor', 'k', 'MarkerFaceColor', stanceColor, ...
                    'LineWidth', lW);
                plotPathArrowV2(ax, ... % arrow for direction 4(c)
                    stancePath(:, 1), stancePath(:, 2),... 
                    arrowSize, arrowAngle, lW-1, 'k', 'front_end');
                % .............................................................
                colormap(ax, pInfo.CUB); clim(ax, panelSweepLimits{iComps});
                switch startCompIdx == endCompIdx
                    case 1
                        colorbar(ax, ...
                            'TickLabelInterpreter', 'latex',...
                            'FontSize', pInfo.cbarFS);
                    otherwise
                        if iComps ~= 1
                            cb = colorbar(ax, ...
                                'TickLabelInterpreter', 'latex',...
                                'FontSize', pInfo.cbarFS);
                            cb.Layout.Tile = 'east';
                        end
                end
                set(get(ax, 'YLabel'),'rotation',0,...
                                            'VerticalAlignment','middle');
                xticks(ax, pInfo.xtickval); yticks(ax, pInfo.ytickval);
                xticklabels(ax, pInfo.xticklab); 
                yticklabels(ax, pInfo.yticklab);
                xlabel(ax, pInfo.x_label_txt,FontSize=pInfo.labelFS); 
                ylabel(ax, pInfo.y_label_txt,FontSize=pInfo.labelFS);
                ax.XAxis.FontSize = pInfo.tickFS; 
                ax.YAxis.FontSize = pInfo.tickFS;
                xlim(pInfo.xlimits); ylim(pInfo.ylimits);
                % .............................................................
            end
        end

        % this function computes the displacement of the body when provided
        % with a reference point, forward and backward intehgration times,
        % and scaling and sliding inputs.
        % OUTPUT: "displacement"
        % ... A more expanded version of this method is used in the 
        % ... "simulateConfigurationTrajectory" method.
        function z = simulateFinalBodyPosition...
                                (refPt, startEndTimes, thisPath2)
            aa = thisPath2.a; ll = thisPath2.l;
            dalpha = thisPath2.paraFdirn;
            dQ = thisPath2.dQ;
            if startEndTimes(1) ~= startEndTimes(2)
                if startEndTimes(1) ~= 0
                    [~, a0] = ode89( @(t, y)...
                        dalpha(aa, ll, y(1), y(2)), ...
                        [0, startEndTimes(1)], refPt ); % get shape ic
                else
                    a0 = refPt; % same ic for final path if tIC == 0
                end
                [~, solnY] = ode89( @(t, y) dQ(aa, ll,... % body fc
                                            y(1), y(2), y(3),...
                                            y(4), y(5)),...
                            [0, diff(startEndTimes)], ...
                            [zeros(1, 3), a0(end, :)] );
                z = solnY(end, 1:3);
            else % no path, just point => no body trajectory/displacement
                z = zeros(1, 3);
            end
        end
        
        % this function computes the body trajectory when provided with a
        % reference point, forward and backward integration times, and the
        % scaling and sliding inputs.
        function configTraj = simulateConfigurationTrajectory...
                    (ref, startEndTimes, thisPath2, plotFlags, panelsMode)
            % unpack reference
            % ... 1. a reference point
            % ... 2. integration times (taken togther with 1. provide the
            % ... starting and ending points for the stance path)
            % ... 3. the offset time to offset the time vectors for the
            % ... stance trajectory, this helps relate them back to the 
            % ... local parallel coordinates
            refPt = ref.P; refToff = ref.tOff;
            % unpack instance parameters
            aa = thisPath2.a; ll = thisPath2.l; 
            dnum = size(thisPath2.ai, 1); intTime = thisPath2.intTime;
            % unpack instance functions
            dalpha = thisPath2.paraFdirn;
            dQ = thisPath2.dQ;
            dz = thisPath2.dz;
            % unpack the stance path start and end times
            tIC = startEndTimes(1); tFC = startEndTimes(2);
            % check which case we are in and handle accordingly
            switch all(startEndTimes == 0)
                case 1 % stance path a point at the shape subspace origin
                    configTraj.complete.t = refToff;
                    configTraj.complete.g = zeros(1, 3); 
                    configTraj.complete.r = refPt;
                    configTraj.complete.dz = ...
                        dz(aa, ll, refPt(1), refPt(2));
                    configTraj.complete.gCirc = zeros(1, 3);
                    configTraj.parameters.Length = 0;
                    configTraj.parameters.disc = 1;
                    configTraj.discretized.t = refToff;
                    configTraj.discretized.g = zeros(1, 3); 
                    configTraj.discretized.r = refPt;
                    configTraj.discretized.dz = ...
                        dz(aa, ll, refPt(1), refPt(2));
                    configTraj.discretized.gCirc = zeros(1, 3);
                    configTraj.status = 'point';
                case 0 % stance path NOT a point at the origin
                    switch tIC == tFC % stance path a point elsewhere
                        case 1
                            % integrate and obtain the solution
                            [~, a0] = ode89( @(t, y)...
                                dalpha(aa, ll, y(1), y(2)), ...
                                [0, tIC], refPt ); % compute just shape IC
                            % pack this up similar to last case
                            configTraj.complete.t = refToff+tIC;
                            configTraj.complete.g = zeros(1, 3); 
                            configTraj.complete.r = a0(end, :);
                            configTraj.complete.dz = ...
                                dz(aa, ll, a0(end, 1), a0(end, 2));
                            configTraj.complete.gCirc = zeros(1, 3);
                            configTraj.parameters.Length = 0;
                            configTraj.parameters.disc = 1;
                            configTraj.discretized.t = refToff+tIC;
                            configTraj.discretized.g = zeros(1, 3); 
                            configTraj.discretized.r = a0(end, :);
                            configTraj.discretized.dz = ...
                                dz(aa, ll, a0(end, 1), a0(end, 2));
                            configTraj.discretized.gCirc = zeros(1, 3);
                            configTraj.status = 'point';
                        case 0 % stance path has actual length
                            % set the status to a path
                            configTraj.status = 'path';
                            % time array for the compute solution
                            solnT = linspace(0, tFC-tIC, dnum);
                            % integrate to obtain the solution
                            [~, a0] = ode89( @(t, y)...
                                dalpha(aa, ll, y(1), y(2)), ...
                                [0, tIC], refPt );
                            [solnT, solnY] = ode89( @(t, y) dQ(aa, ll,...
                                                        y(1), y(2), y(3),...
                                                        y(4), y(5)),...
                                        solnT, [zeros(1, 3), a0(end, :)] );
                            % offset the time vector to accommodate the
                            % reference point and initial condition
                            solnT = solnT + refToff + tIC;
                            % package and return this solution
                            % ... store and compute the full solution here
                            configTraj.complete.t = solnT;
                            configTraj.complete.g = solnY(:, 1:3); 
                            configTraj.complete.r = solnY(:, 4:end);
                            configTraj.complete.dz = ...
                                dz(aa, ll, solnY(:, 4), solnY(:, 5));
                            configTraj.complete.gCirc = cumtrapz(solnT, ...
                                                configTraj.complete.dz);
                            % ... compute the length of the shape path and 
                            % ... the number of discretizations required to 
                            % ... define the shape trajectory based on the 
                            % ... overall discretization of the continuous 
                            % ... shape subspace
                            configTraj.parameters.Length = ...
                                abs(solnT(end) - solnT(1)); 
                            configTraj.parameters.disc = ...
                                ceil(configTraj.parameters.Length...
                                                    /sum(intTime)*dnum);
                            if configTraj.parameters.disc == 0
                                configTraj.parameters.disc = 1;
                                % ... the discretization should not be zero
                            end
                            % ... using that information, condition the 
                            % ... trajectory and compute the stratified 
                            % ... panels before returning
                            solnHatT = linspace(solnT(1), solnT(end),...
                                            configTraj.parameters.disc)';
                            solnHatY = interp1(solnT, solnY, solnHatT, ...
                                                    "pchip");
                            configTraj.discretized.t = solnHatT;
                            configTraj.discretized.g = solnHatY(:, 1:3); 
                            configTraj.discretized.r = solnHatY(:, 4:end);
                            configTraj.discretized.dz = ...
                                dz(aa, ll, solnHatY(:, 4), solnHatY(:, 5));
                            configTraj.discretized.gCirc = ...
                                        cumtrapz(solnHatT, ...
                                                configTraj.discretized.dz);
                    end
            end
            % compute the commutative trajectory and net displacements
            % estimates for each case:
            % ... for the complete trajectory
            configTraj.complete.z = configTraj.complete.g(end, :);
            configTraj.complete.gHat = exponentiateLieAlgebraElement...
                                (configTraj.complete.gCirc);
            configTraj.complete.zHat = configTraj.complete.gHat(end, :);
            % ... for the discretized trajectory
            configTraj.discretized.z = configTraj.discretized.g(end, :);
            configTraj.discretized.gHat = exponentiateLieAlgebraElement...
                                (configTraj.discretized.gCirc);
            configTraj.discretized.zHat = ...
                                    configTraj.discretized.gHat(end, :);
            % ... if the 'plotFlag' doesn't exist, do not plot anything,
            % ... else based on the case plot
            if nargin < 4 || isempty(plotFlags)
                plotFlags = false(1, 2);
            end
            for i = 1:numel(plotFlags)
                if plotFlags(i)
                    % ... ... shape trajectory with an additional point
                    if ~exist('tMod', 'var') % compute mods only once!!
                        tMod = configTraj.complete.t;
                        rMod = configTraj.complete.r;
                        rMod = interp1...
                                (tMod, rMod, ...
                                linspace(tMod(1), tMod(end), dnum+1), ...
                                "pchip");
                    end
                    switch i
                        case 1
                            Path2_Mobility.plotStanceOnNonslipLevelSets...
                                    (ref, startEndTimes, thisPath2, ...
                                    rMod, configTraj.status);
                        case 2
                            if nargin < 5 || isempty(panelsMode)
                                panelsMode = 'all';
                            end
                            Path2_Mobility.plotStanceOnPanels...
                                    (ref, startEndTimes, thisPath2, ...
                                    rMod, configTraj.status, panelsMode);
                    end
                end
            end
        end

        % this function computes the shape trajectory in the gait phase 
        % when provided with a reference point, forward and backward 
        % integration times, and the scaling and sliding inputs.
        function [tau, beta, subgaitTraj] = computeSubgaitInCoordinates...
                (ref, startEndTimes, thisPath2, shapeStanceTraj, status)
            dnum = size(thisPath2.ai, 1); % discretization of shapes
            if nargin < 4 % if only inputs and references are provided
                % unpack reference
                % ... 1. a reference point
                % ... 2. integration times (taken togther with 1. provide the
                % ... starting and ending points for the stance path)
                refPt = ref.P;
                % unpack instance parameters
                aa = thisPath2.a; ll = thisPath2.l; 
                % unpack instance functions
                dalpha = thisPath2.paraFdirn;
                % unpack the
                tIC = startEndTimes(1); tFC = startEndTimes(2);
                % check which case we are in and handle accordingly
                switch all(startEndTimes == 0)
                    case 1 % path just point at origin
                        shapeStanceTraj = refPt;
                        status = 'point';
                    otherwise % path NOT a point at origin
                        switch tIC == tFC
                            case 1 % path is a point elsewhere
                                % integrate and obtain the solution
                                [~, a0] = ode89( @(t, y)...
                                    dalpha(aa, ll, y(1), y(2)), ...
                                    [0, tIC], refPt ); % compute just shape IC
                                % shape trajectory and status string
                                shapeStanceTraj = a0(end, :);
                                status = 'point';
                            case 0 % path is actually 1D
                                % set the status to a path
                                status = 'path';
                                % time array for the compute solution
                                solnT = linspace(0, tFC-tIC, dnum+1);
                                % integrate to obtain the solution
                                [~, a0] = ode89( @(t, y)...
                                    dalpha(aa, ll, y(1), y(2)), ...
                                    [0, tIC], refPt );
                                [~, solnY] = ode89( @(t, y) ...
                                                dalpha(aa, ll, y(1), y(2)), ...
                                                        solnT, a0(end, :) );
                                % shape trajectory
                                shapeStanceTraj = solnY;
                        end
                end
            end
            
            % Finally, we return the 
            % 1) shape trajectory as a function of the
            % ... gait phase: [0, 2*pi(-)] --> [r; flipud(r(2:end-1))]
            % 2) contact vector during stance phase
            % ... "beta" variable name for the discrete part of the shape 
            % ... space
            % ... the last poin the computed shape trajectory belongs to
            % ... the swing phase btw because we have already included the
            % ... first point
            switch status
                case 'point'
                    subgaitTraj = repmat(shapeStanceTraj, 2*dnum, 1);
                case 'path'
                    subgaitTraj = [shapeStanceTraj; % stance (one point at the beginning of swing)
                            flipud(shapeStanceTraj(2:end-1, :))]; % just swing
                    % the last point is right before the new cycle starts
            end
            beta = [ones(dnum, 1); zeros(dnum, 1)];
            % provide the normalized gait phase array
            tau = linspace(0, 2*pi, 2*dnum+1); tau = tau(1:end-1)';
        end

        % method to extract a specific parallel coordinate given the index
        % and parallel coordinate sweep
        function aParaOut = fetchParallCoordsFromIndex(aParaIn, idx, bItr)
            fieldsPara = fieldnames(aParaIn);
            aParaOut = [];
            switch nargin < 3
                case 1 % 'legacy' case
                    for i = 1:numel(fieldsPara)
                        switch fieldsPara{i}
                            case 'isValid'
                                aParaOut.(fieldsPara{i}) = ...
                                    aParaIn.(fieldsPara{i})(idx);
                            otherwise
                                aParaOut.(fieldsPara{i}) = ...
                                    aParaIn.(fieldsPara{i}){idx};
                        end
                    end
                case 0 % 'branched' case
                    for i = 1:numel(fieldsPara)
                        switch fieldsPara{i}
                            case 'isValid'
                                aParaOut.(fieldsPara{i}) = ...
                                    aParaIn.(fieldsPara{i}){bItr}(idx);
                            otherwise
                                aParaOut.(fieldsPara{i}) = ...
                                    aParaIn.(fieldsPara{i}){bItr}{idx};
                        end
                    end
            end
        end

        % method to compute the reference point along the perpendicular
        % coordinate given another reference point and the time offset,
        % this also returns the offset maximum path lengths accordingly
        function [tOff, tMax, thisPath2] = ...
                                computeToffFromPerpCoord(refPt, thisPath2)
            % unpack
            a = thisPath2.kinfunc.aa;
            l = thisPath2.kinfunc.ll;
            F_fxn = thisPath2.kin.ksq_ij{thisPath2.sIdx};
            perpCoords = thisPath2.aPerpF; paraCoords = thisPath2.aParallF;
            % compute the reference along the perp coordinates with the
            % same F-value
            % ... modified to work with new slipping or perpendicular axis 
            % ... generation mode: "branched"
            % ... this is a two step process: 1) if the reference is very
            % ... close to an already existing point, then we replace the
            % ... requested reference with that pre-computed point
            % ... 2) if the reference doesn't exist, then we interpolate to
            % ... find corresponding reference in the perp coords
            switch thisPath2.slipMode
                case 'legacy'
                    switch abs(min(vecnorm(perpCoords.y - refPt, 2, 2))) < 1e-6
                        case 1
                            % find the index with the smallest value below the
                            % threshold and obtain the corresponding parallel
                            % coordinates
                            [~, idxRef] = min(vecnorm(perpCoords.y - refPt, 2, 2));
                            aParaNow = Path2_Mobility.fetchParallCoordsFromIndex...
                                                        (paraCoords, idxRef);
                        case 0
                            % if you're here, none of the points are sufficiently
                            % close, so we explicity compute the parallel
                            % coordinates
                            % ... compute the F-lvl requested and the corresponding reference
                            % ... along the perp coords
                            refF = F_fxn(a, l, refPt(1), refPt(2));
                            perpRef = interp1(perpCoords.F, perpCoords.y, refF,...
                                                                    "spline", nan);
                            % ... Now, we compute the local parallel coordinates if 
                            % ... the reference at the interpolated perpendicular 
                            % ... Ref is not nan-- if it is, we return an error to 
                            % ... signify that a reference outside the allowable 
                            % ... region of F is chosen
                            if any(isnan(perpRef))
                                error(['ERROR! Please choose a reference within ' ...
                                    'the allowable region of F.']);
                            end
                            thisPath2 = ...
                                Path2_Mobility.computeSpecificParallelCoordinates...
                                ( thisPath2, perpRef ); 
                            aParaNow = thisPath2.aParaRef;
                    end
                    % Now, we compute the the time offset based on the difference
                    % between the requested reference and the computed parallel
                    % coordinates. Again, we also offset the maximum times
                    % according to the tOff value.
                    % ... again, if the time offset is smaller than 1e-6, just set
                    % ... it to 0.
                    refDiff = aParaNow.y - refPt;
                    tOff = fmincon(@(x) ...
                                norm(interp1(aParaNow.t, refDiff, x, "pchip")),...
                                mean(aParaNow.t), [], [], [], [],...
                                min(aParaNow.t), max(aParaNow.t), [],...
                                optimoptions("fmincon", "Display", "none") );
                    if isnan(tOff)
                        error(['ERROR! If the offset time is "nan", then the ' ...
                            'problem is ill-conditioned and lies outside ' ...
                            'slip-nonslip coordinates. Please reframe the problem ' ...
                            'to these computed coordinates. Use the ' ...
                            '"highlightNonslipLevelSet" to help visualize if the ' ...
                            'selected reference point is within this space.']);
                    end
                    if abs(tOff) < 1e-6
                        tOff = 0;
                    end
                    tMax = aParaNow.tMax + tOff*[1, -1];
                case 'branched'
                    for bItr = 1:numel(perpCoords.y) % iterate over each branch
                        switch abs(min(vecnorm(perpCoords.y{bItr} - refPt, 2, 2))) < 1e-6
                            case 1
                                [~, idxRef] = min(vecnorm(perpCoords.y{bItr} - refPt, 2, 2));
                                aParaNow = Path2_Mobility.fetchParallCoordsFromIndex...
                                                (paraCoords, idxRef, bItr);
                            case 0
                                refF = F_fxn(a, l, refPt(1), refPt(2));
                                perpRef = interp1(perpCoords.F{bItr}, perpCoords.y{bItr}, refF,...
                                                                        "spline", nan);
                                if any(isnan(perpRef))
                                    error(['ERROR! Please choose a reference within ' ...
                                        'the allowable region of F.']);
                                end
                                thisPath2 = ...
                                    Path2_Mobility.computeSpecificParallelCoordinates...
                                    ( thisPath2, perpRef ); 
                                aParaNow = thisPath2.aParaRef;
                        end
                        refDiff = aParaNow.y - refPt;
                        tOff = fmincon(@(x) ...
                                    norm(interp1(aParaNow.t, refDiff, x, "pchip")),...
                                    mean(aParaNow.t), [], [], [], [],...
                                    min(aParaNow.t), max(aParaNow.t), [],...
                                    optimoptions("fmincon", "Display", "none") );
                        if isnan(tOff)
                            if bItr > 1
                                error(['ERROR! If the offset time is "nan", then the ' ...
                                'problem is ill-conditioned and lies outside ' ...
                                'slip-nonslip coordinates. Please reframe the problem ' ...
                                'to these computed coordinates. Use the ' ...
                                '"highlightNonslipLevelSet" to help visualize if the ' ...
                                'selected reference point is within this space.']);
                            end
                        else
                            if abs(tOff) < 1e-6
                                tOff = 0;
                            end
                            tMax = aParaNow.tMax + tOff*[1, -1];
                            return;
                        end
                    end
            end
        end

        % For subgaits defined using the "computeSubgaitInCoordinates", we
        % offset the periodic waveform by the requested phase offset
        function [betaOut, subgaitOut] = phaseOffsetSubgait(tau, beta, ...
                                                    subgait, phaseOffset)
            % modulo the 'phaseOffset' to be between +pi and -pi; 
            % simplest (braindead) way to do this is to go into the S^1 
            % (circle) coordinates and then come back with the atan2 fxn
            sinPhase = sin(phaseOffset); cosPhase = cos(phaseOffset);
            phaseOffset = atan2(sinPhase, cosPhase);
            % augment the args with two more cycles before and after to
            % convert this phaseoffset problem into an interpolation
            % problem which is well defined
            tauA = [tau-2*pi; tau; tau+2*pi];
            subgaitA = repmat(subgait, 3, 1);
            % get the phase array to interpolate the solutions to
            tauQ = tau + phaseOffset;
            % interpolate the contact and limb angle trajectories
            % ... 1) do "pchip" (structure preserving) interpolation for 
            % ... the subgait 
            subgaitOut = interp1(tauA, subgaitA, tauQ, "pchip");
            % ... 2) an exact(ish) binary switch based on the location of 
            % ... ("phaseOffset") + pi
            betaOut = nan(size(beta));
            tauQ(tauQ >= 2*pi) = tauQ(tauQ >= 2*pi) - 2*pi; 
            tauQ(tauQ < 0) = tauQ(tauQ < 0) + 2*pi; % condition tauQ
            stanceIdx = (  tauQ >= 0 & tauQ < pi  ); % get stance indices
            % ... ... set the stance first and then the swing phase
            betaOut(stanceIdx) = ones(size(betaOut(stanceIdx)));
            betaOut(~stanceIdx) = zeros(size(betaOut(~stanceIdx)));
        end

        % compute the theta-panel value by interpolating between given an
        % integration time using the specific parallel coordinates
        function dzThetaHat = interpThetaPanelFromIntTime(t, dzTheta, tQ)
            % because the panels are typically quite smooth, but some times
            % have sharp features, we use 'pchip' interpolation method to
            % preserve the overall structure
            dzThetaHat = interp1(t, dzTheta, tQ, "pchip");
        end

        % compute all panel values through interpolation; similar to the
        % setup in the previous function
        function dzHat = interpPanelsFromIntTime(t, dz, tQ)
            dzHat = interp1(t, dz, tQ, "pchip");
        end
        
    end % END OF STATIC METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

%% AUXILIARY FUNCTIONS

% compute the F-value at a point along a line from 'refPt' constructed
% using a 'unitPhasor' with a multiplier 'aValue'
function fVal = FalongLine(aa, ll, F_fxn, refPt, unitPhasor, aValue)
    pt = returnPtAlongLineFromIC(refPt, unitPhasor, aValue)';
    fVal = F_fxn(aa, ll, pt(1), pt(2));
end

% compute the location at which the bounds of F are barely violated
function aViolation = ptViolatingFbounds(aa, ll, ...
    F_fxn, F_bounds, F_bounds_full, ...
    refPt, delX, delY, ...
    optIC, optLims)

    aViolation = [];

    [aLowViolation, fValViolation] = ...
    fmincon( @(a) abs(F_bounds(1) - ...
        FalongLine(aa, ll, F_fxn, refPt, [delX, delY], a)), ...
        optIC, ...
        [], [], [], [], ...
        optLims(1), optLims(2), ...
        [], ...
        optimoptions("fmincon", "Display", "off") ...
            );
    if fValViolation < 1e-5*diff(F_bounds_full)
        aViolation = aLowViolation;
    else
        [aHighViolation, fValViolation] = ...
        fmincon( @(a) abs(F_bounds(2) - ...
            FalongLine(aa, ll, F_fxn, refPt, [delX, delY], a)), ...
            optIC, ...
            [], [], [], [], ...
            optLims(1), optLims(2), ...
            [], ...
            optimoptions("fmincon", "Display", "off") ...
                );
        if fValViolation < 1e-5*diff(F_bounds_full)
            aViolation = aHighViolation;
        end
    end
end