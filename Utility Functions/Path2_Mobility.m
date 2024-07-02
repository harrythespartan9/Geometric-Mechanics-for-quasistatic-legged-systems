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

        kinfunc   % kineamtics of level-2 contact submanifolds

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

        aInf      % limb coordinates at the infimum of F

        aSup      % limb coordinates at the supremum of F

        F_inf     % infimum of F

        F_sup     % supremum of F

        F_bounds  % bounds on the accessible values of F and is a 
                  % ... subinterval of [F_inf, F_sup]

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

    end
    
    % just constructor %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        
        % Constructor
        function thisPath2 = Path2_Mobility(sIdx, ...
                                        kin, kinfunc, p_kin, p_info, ...
                                            intTime, refPt)
            % assign the props
            thisPath2.sIdx = sIdx;
            thisPath2.kin = kin;
            thisPath2.kinfunc = kinfunc{sIdx};
            thisPath2.p_kin = p_kin{sIdx};
            thisPath2.p_info = p_info{sIdx};
            thisPath2.intTime = intTime;
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
                                                            ( thisPath2 );
                thisPath2.aPerpF.color = interpColorAndCondition...
                ( ...
                linspace(F_inf, F_sup, size(thisPath2.p_info.jetDark, 1))', ...
                thisPath2.p_info.jetDark, thisPath2.aPerpF.F ...
                );
            thisPath2 = Path2_Mobility.computeParallelCoordinates...
                                                            ( thisPath2 );
            thisPath2 = Path2_Mobility.computeSpecificParallelCoordinates...
                                            ( thisPath2, refPt, intTime );
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
                                                            ( thisPath2 )
            % unpack properties and pack 
            % ... first required parameters followed by the bounds needed
            % ... and finally the functions needed
            % ... this argument structure is used to stop the ODE
            % ... integration process when shape space constraints are
            % ... violated
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
            [~, ~, teNow, yeNow, ~] = ...
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
        end

        % function to compute the level-set colors at each 

        % function to compute parallel limb shape coordinate
        % ... this is the path along the level-sets of F, obtained by
        % ... rotating the gradient 90degs clockwise (ensures an 
        % ... anticlockwise generator vector field around the singularity) 
        function thisPath2 = computeParallelCoordinates( thisPath2 )
            % iterate and find centroid of level-set
            % ... we iterate over each solution point and try to identify
            % ... the centroid about which they connect on themselves
            % ... for our case, the centroid is either the infimum or
            % ... supremum point in the reduced shape space
            % ... this can be sufficiently coarse, so we are using 'ode23'
            % ... because we only want to identify which direction (towards
            % ... infimum or supremum) the centroid is moving towards
            % ... to do this, we shall initialize two quantities and store
            % ... that information:
            yExtremal = nan(size(thisPath2.aPerpF.y));
            yRefPhase = nan(size(thisPath2.aPerpF.t));
            tInt = 100*[1 1]; % choose large integration time
            for i = 1:size(thisPath2.aPerpF.y, 1)
                [~, yNow] = ode23(@(t, y) -thisPath2.paraFdirn(...
                    thisPath2.a, thisPath2.l, y(1), y(2)),...
                    [0 tInt(1)], thisPath2.aPerpF.y(i, :)); % backward pass
                [~, yNow] = ode23(@(t, y) +thisPath2.paraFdirn(...
                    thisPath2.a, thisPath2.l, y(1), y(2)),...
                    [0 sum(tInt)], yNow(end, :)); % forward pass
                errInf = repmat(thisPath2.aInf, size(yNow, 1), 1) - yNow; 
                errInf = min(vecnorm(errInf, 2, 2)); % distance from Inf
                errSup = repmat(thisPath2.aSup, size(yNow, 1), 1) - yNow; 
                errSup = min(vecnorm(errSup, 2, 2)); % distance from Sup
                switch errInf < errSup
                    case 1
                        yExtremal(i, :) = thisPath2.aInf; 
                    case 0
                        yExtremal(i, :) = thisPath2.aSup;
                end
                yPhasorNow = (thisPath2.aPerpF.y(i, :)-yExtremal(i, :))/...
                        norm(thisPath2.aPerpF.y(i, :) - yExtremal(i, :)); 
                yRefPhase(i) = atan2(yPhasorNow(2), yPhasorNow(1));
            end
            % the main solution
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
            thisPath2.aParallF = [];
            thisPath2.aParallF.isValid = true(size(thisPath2.aPerpF.t));
            thisPath2.aParallF.eventInfo = cell(size(thisPath2.aPerpF.t));
            thisPath2.aParallF.y = cell(size(thisPath2.aPerpF.t)); 
                            thisPath2.aParallF.t = thisPath2.aParallF.y;
            thisPath2.aParallF.dz = thisPath2.aParallF.y;
            eventList = {'shape_bounds', 'phase_bounds'};
            argStruct = []; argStruct.functions = []; % struct init
            argStruct.bounds.alphaLimits = thisPath2.aLimits; % bounds
                        argStruct.bounds.angleThreshold = deg2rad(5);
            for i = 1:numel(thisPath2.aParallF.y)
                % ... initialize current event info cell
                thisPath2.aParallF.eventInfo{i} = cell(1, 2);
                % ... select initial conditions
                y0now = thisPath2.aPerpF.y(i, :);
                % ... select the current EVENT options
                argStruct.parameters.yExtremal = yExtremal(i, :); % params
                    argStruct.parameters.yRefPhase = yRefPhase(i);
                optionsNow = odeset(  ...
                    'Events', ...
                    @(t, y) nonslipShapeCoordsEvents(t, y, argStruct, ...
                    eventList) ...
                    );
                % ... integrate backward path
                [tNow, yNow, teNow, ~, ieNow] = ...
                ode89( @(t,y) -thisPath2.paraFdirn(...
                            thisPath2.a, thisPath2.l, y(1), y(2)), ...
                [0, thisPath2.intTime(1)], y0now, optionsNow );
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
                tMain = tBwd + thisPath2.intTime(2);
                yNow = yNow(tNow <= tBwd, :);
                [tNow, yNow, teNow, ~, ieNow] = ...
                    ode89( @(t,y) thisPath2.paraFdirn(...
                                thisPath2.a, thisPath2.l, y(1), y(2)), ...
                    linspace(0, tMain, size(thisPath2.ai, 1)),...
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
                thisPath2.aParallF.y{i} = yNow(tNow <= tFull, :);
                thisPath2.aParallF.t{i} = tNow(tNow <= tFull); % final solution
                % ... compute the stratified panel along this solution
                thisPath2.aParallF.dz{i} = thisPath2.dz(...
                                        thisPath2.a, thisPath2.l,...
                                        thisPath2.aParallF.y{i}(:, 1), ...
                                        thisPath2.aParallF.y{i}(:, 2));
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
                                                        intTime, refPt)
            % compute the complimentary submanifold index
            temp = zeros(1, 2); temp(thisPath2.sCol) = 1;
            thatCol = find(~temp);
            sThatIdx = kin.cs_IdxFor2bGaits(thisPath2.sRow, thatCol);
            % construct the complimentary submanifold "Path2_Mobility" obj
            thatPath2 = Path2_Mobility(sThatIdx,...
                                       kin, kinfunc, p_kin, p_info,...
                                       intTime, refPt);
        end

        % find the parallel coordinates specifically at a specific point
        % ... this function is setup to compute the nonslip directions for
        % ... the specified integration time about the refernce point and
        % ... all of this information is stored in the 'aParaRef' property
        % ... Everytime this function is called with new arguments, the
        % ... corresponding property fields get updated and replaced
        % ... for more information on the exact computations performed,
        % ... refer to "computeParallelCoordinates" method defined above.
        function thisPath2 = computeSpecificParallelCoordinates...
                                            ( thisPath2, refPt, intTime )
            if nargin < 3
                intTime = thisPath2.intTime;
            elseif nargin < 2
                refPt = thisPath2.refPt;
            elseif nargin > 3 || nargin == 0
                error('ERROR! Incorrect number of inputs.');
            end
            tInt = 100*[1 1];
            [~, yNow] = ode23(@(t, y) -thisPath2.paraFdirn(...
                thisPath2.a, thisPath2.l, y(1), y(2)),...
                [0 tInt(1)], refPt);
            [~, yNow] = ode23(@(t, y) +thisPath2.paraFdirn(...
                thisPath2.a, thisPath2.l, y(1), y(2)),...
                [0 sum(tInt)], yNow(end, :));
            errInf = repmat(thisPath2.aInf, size(yNow, 1), 1) - yNow; 
            errInf = min(vecnorm(errInf, 2, 2));
            errSup = repmat(thisPath2.aSup, size(yNow, 1), 1) - yNow; 
            errSup = min(vecnorm(errSup, 2, 2));
            switch errInf < errSup
                case 1
                    yExtremal = thisPath2.aInf; 
                case 0
                    yExtremal = thisPath2.aSup;
            end
            yPhasorNow = (refPt - yExtremal)/norm(refPt - yExtremal); 
            yRefPhase = atan2(yPhasorNow(2), yPhasorNow(1));
            thisPath2.aParaRef = [];
            thisPath2.aParaRef.isValid = true;
            thisPath2.aParaRef.eventInfo = cell(1, 2);
            eventList = {'shape_bounds', 'phase_bounds'};
            argStruct = []; argStruct.functions = []; 
            argStruct.bounds.alphaLimits = thisPath2.aLimits; 
                        argStruct.bounds.angleThreshold = deg2rad(5);
            argStruct.parameters.yExtremal = yExtremal; 
                argStruct.parameters.yRefPhase = yRefPhase;
            options = odeset(  ...
                'Events', ...
                @(t, y) nonslipShapeCoordsEvents(t, y, argStruct, ...
                eventList) ...
                );
            [tNow, yNow, teNow, ~, ieNow] = ...
            ode89( @(t,y) -thisPath2.paraFdirn(...
                        thisPath2.a, thisPath2.l, y(1), y(2)), ...
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
                ode89( @(t,y) thisPath2.paraFdirn(...
                            thisPath2.a, thisPath2.l, y(1), y(2)), ...
                linspace(0, tMain, size(thisPath2.ai, 1)),...
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
            thisPath2.aParaRef.y = yNow(tNow <= tFull, :);
            thisPath2.aParaRef.t = tNow(tNow <= tFull); % final solution
            % ... compute the stratified panel along this solution
            thisPath2.aParaRef.dz = thisPath2.dz(...
                                    thisPath2.a, thisPath2.l,...
                                    thisPath2.aParaRef.y(:, 1), ...
                                    thisPath2.aParaRef.y(:, 2));
        end

        function visualizeAccessibleShapeSpace(thePath2)
        end

        function visualizePerpendicularDirections(thePath2)
        end

        function visualizeNonslipDirections(thePath2)
        end

        function visualizeSlipNonslipShapeCoordinates(thePath2)
        end

        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end