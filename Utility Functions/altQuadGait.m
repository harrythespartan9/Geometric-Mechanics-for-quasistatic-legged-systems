classdef altQuadGait
    %ALTQUADGAIT this class is used for defining alternating
    %gaits for quadrupedal robots
    %   Given two instances of the "Path2_Mobility" class that are
    %   complementary subspaces of the shape space, this class defines an
    %   alternating gait such as a trot, bound, or a pace gait using
    %   subgatis in each shape subspace. The net displacements are
    %   approximated using the Baker-Campbell-Hausdorff formula.
    
    properties

        ithStance           % ith stance phase in the shape subspace Si

        jthStance           % jth stance phase in the shape subspace Sj which 
                            % is complementary to Si.

        leafExplorationMode % "origin-only" supported for now

        inputMode           % "std" and "path_limit_compliant" modes are 
                            % supported. 
                            % "std": this is the input description from
                            % geometrically modulable gait design paper and 
                            % "path_limit_compliant": is a linear scaling of 
                            % affine time shift provided by the sliding 
                            % input is now a function of the scaling input.
                            % in each method, there are two inputs per
                            % stance phase called the scaling and sliding
                            % inputs \in \[-1,1\].

        bodyLimbTransform   % function to compute the transforms from the 
                            % body frame to each limb
        
        integrationLimits   % forward and backward integration limits-- 
                            % this is only used when in "path_limit_compliant"

        stanceSpace         % the stance space is an orthogonal 
                            % representation of the stance phase paths 
                            % during each subgait cycles; the panels in the
                            % subgait cycles form the connection vector
                            % fields in each direction and their
                            % lie-bracket is the curvature; taken together
                            % using this information, we can check the gait
                            % controllability for this two-beat gait

        inputSpace          % the system's body displacement and velocity 
                            % information as a function of the input space
                            % in two categories: 'simulation' and
                            % 'estimation'. The inputs in this space are
                            % obtained 
        
    end
    
    methods

        function thisAltGait = altQuadGait(stance1, stance2, ...
                                            leafModeArg, inputModeArg, ...
                                            integrationLimitsArg, ...
                                            se2Transforms)
            %ALTERNATINGQUADRUPEDALGAIT Construct an instance of this class
            %   mostly we setup the properties here and make some back
            %   checks to see if tha "Path2_Mobility" instances are
            %   complementary, etc.
            
            % check if the stances 1 and 2 are complementary, if not return
            % and error
            if ~Path2_Mobility.areComplementaryStances(stance1, stance2)
                error(['ERROR! The provided stance phases are not ' ...
                    'complementary. For more details, refer to ' ...
                    '"se2_toyproblems_case_1_mobility.mlx" livescript for ' ...
                    'more details.']);
            else
                % order the stance phases so that we obtain the same
                % results irrespective of the ordering of the stance phases
                % here we because it is a quadruped, whichever stance phase
                % contains limb 1 gets to be the "ithStance" property
                if any(stance1.cs == 1)
                    thisAltGait.ithStance = stance1;
                    thisAltGait.jthStance = stance2;
                else
                    thisAltGait.ithStance = stance2;
                    thisAltGait.jthStance = stance1;
                end
            end

            % only the "origin-only" leaf mode is supported for now, the
            % other leafs are not too useful at the moment
            if ~strcmp(leafModeArg, "origin-only")
                error(['ERROR! We only support the "origin-mode" for ' ...
                    'mobility exploration at the moment.']);
            else
                thisAltGait.leafExplorationMode = leafModeArg;
            end

            % check the input mode: "std" or "path_limit_compliant"
            % ... if the mode requested is adaptive scaling method, we
            % ... additionally require the maximum forward and backward
            % ... integration times; we require this to be the last
            % ... argument and if that's not the case, then return an error
            % ... furthermore, if the integration limits is either infinity
            % ... or NaN, then also return an error; the limits have to be
            % ... numeric and finite
            switch inputModeArg
                case 'std'
                    thisAltGait.inputMode = 'std';
                    thisAltGait.integrationLimits = inf*[1, 1];
                case 'path_limit_compliant'
                    if nargin < 5
                        error(['ERROR! For the "path_limit_compliant" input ' ...
                            'method, we need the maximum forward and ' ...
                            'backward integration times to appropriately ' ...
                            'bound the sliding input contribution.']);
                    else
                        if any(~isfinite(integrationLimitsArg))
                            error(['ERROR! The integration limits have to ' ...
                                'finite numerical values, so they cant be ' ...
                                'NaN or Inf.']);
                        else
                            thisAltGait.inputMode = 'path_limit_compliant';
                            thisAltGait.integrationLimits = ...
                                                    integrationLimitsArg;
                        end
                    end
                otherwise
                    error(['ERROR! Other input descriptions are not ' ...
                        'supported.']);
            end

            % assign the SE(2) transforms from world frame to the body and
            % then from the body to each foot location
            thisAltGait.bodyLimbTransform = se2Transforms;

            % obtain the stance space description for this alternating gait
            % cycle
            thisAltGait = altQuadGait.computeStanceSpace(thisAltGait);

        end

    end  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Static)
        
        % this function helps in changing the current input mode to a new
        % input mode
        function thisAltGait = switchInputMode(thisAltGait, ...
                                        inputModeArg, integrationLimitsArg)
            % Check the current input mode
            switch thisAltGait.inputMode
                case inputModeArg % new input mode same as current mode
                    % if it is the adaptive mode which requires an
                    % integration limit, handle accordingly, else ignore if
                    % its "std" which requires no change
                    if strcmp(inputModeArg, 'path_limit_compliant')
                        if nargin < 3
                            error(['ERROR! For the ' ...
                                    '"path_limit_compliant" input ' ...
                                    'method, we need the maximum forward ' ...
                                    'and backward integration times to ' ...
                                    'appropriately bound the sliding input ' ...
                                    'contribution.']);
                        else
                            thisAltGait.inputMode = ...
                                                    'path_limit_compliant';
                            thisAltGait.integrationLimits = ...
                                                    integrationLimitsArg;
                        end
                    end
                otherwise
                    switch inputModeArg
                        case 'std'
                            thisAltGait.inputMode = 'std';
                            thisAltGait.integrationLimits = inf*[-1, 1];
                        case 'path_limit_compliant'
                            if nargin < 3
                                error(['ERROR! For the ' ...
                                    '"path_limit_compliant" input ' ...
                                    'method, we need the maximum forward ' ...
                                    'and backward integration times to ' ...
                                    'appropriately bound the sliding input ' ...
                                    'contribution.']);
                            else
                                if any(~isfinite(integrationLimitsArg))
                                    error(['ERROR! The integration limits ' ...
                                        'have to finite numerical values, ' ...
                                        'so they cant be NaN or Inf.']);
                                else
                                    thisAltGait.inputMode = ...
                                                    'path_limit_compliant';
                                    thisAltGait.integrationLimits = ...
                                                    integrationLimitsArg;
                                end
                            end
                        otherwise
                            error(['ERROR! Other input descriptions are not ' ...
                                'supported.']);
                    end
            end
        end

        % construct the stance space panels and lie-brackets for estimating
        % the net displacement and checking gait controllability
        function thisAltGait = computeStanceSpace(thisAltGait)
            % make sure that you're in the origin-only leaf exploration
            % mode, else error out as other modes aren't supported right
            % now
            if ~strcmp(thisAltGait.leafExplorationMode, 'origin-only')
                error(['ERROR! Only origin exploration mode is supported ' ...
                    'for now']);
            end
            % unpack the subgait instances
            stanceI = thisAltGait.ithStance;
            stanceJ = thisAltGait.jthStance;
            % obtain the parallel or nonslip coordinates at the origin for
            % the full level-set
            stanceI = ...
                Path2_Mobility.computeSpecificParallelCoordinates...
                (stanceI, zeros(1, 2), [], true);
            stanceJ = ...
                Path2_Mobility.computeSpecificParallelCoordinates...
                (stanceJ, zeros(1, 2), [], true);
            % stance space properties
            % ... get the stance-decoupled properties and store
            % ... for the limits, if the input-mode is 
            % ... "path_limit_compliant", we want the limits store in the
            % ... gait instance as "integrationLimits"
            aI  = stanceI.aParaRef.t;   aJ = stanceJ.aParaRef.t;
            dzI = stanceI.aParaRef.dz; dzJ = stanceJ.aParaRef.dz;
            thisAltGait.stanceSpace.a{1}  = aI;
            thisAltGait.stanceSpace.a{2}  = aJ;
            switch strcmp(thisAltGait.inputMode, 'path_limit_compliant')
                case 1
                    thisAltGait.stanceSpace.aLimits{1} = ...
                                            thisAltGait.integrationLimits;
                    thisAltGait.stanceSpace.aLimits{2} = ...
                                            thisAltGait.integrationLimits;
                case 0
                    thisAltGait.stanceSpace.aLimits{1} = ...
                        [min(aI, [], "all"), max(aI, [], "all")];
                    thisAltGait.stanceSpace.aLimits{2} = ...
                        [min(aJ, [], "all"), max(aJ, [], "all")];
            end
            thisAltGait.stanceSpace.dnum{1} = size(aI, 1);
            thisAltGait.stanceSpace.dnum{2} = size(aJ, 1);
            thisAltGait.stanceSpace.dz{1} = dzI;
            thisAltGait.stanceSpace.dz{2} = dzJ;
            % ... get the meshed properties as well
            % ... ... first obtain the indices running along each
            % ... ... direction, then map the properties accordingly
            [AI, AJ] = meshgrid(aI, aJ);
            % ... ... iterate over each column corresponding to the panel
            % ... ... directions and compute the grid
            [DZI, DZJ, DZIJ] = stratifiedGridAndLieBracket(dzI, dzJ);
            % ... compute the information to discuss the span of the
            % ... involutive closure in the stance subspace
            dzImagn = vecnorm(DZI, 2, 3); dzJmagn = vecnorm(DZJ, 2, 3);
            dzIdotJ = dot(DZI, DZJ, 3);
            idxNonzero = (dzIdotJ ~= 0); % nonzero indices
            dzIdotJ =   reshape(...
                dzIdotJ(idxNonzero)./... % normalize
                (dzImagn(idxNonzero).*dzJmagn(idxNonzero)),...
                        size(dzIdotJ)); 
            thisAltGait.stanceSpace.A{1} = AI;
            thisAltGait.stanceSpace.A{2} = AJ;
            thisAltGait.stanceSpace.DZ{1} = DZI;
            thisAltGait.stanceSpace.DZ{2} = DZJ;
            thisAltGait.stanceSpace.LBDZ = DZIJ;
            thisAltGait.stanceSpace.dzIdotJ = dzIdotJ;
            % ... compute the exact location where the theta component of 
            % ... both panels are zero where the span would be 1
            thetaSelect = 3;
            thisAltGait.stanceSpace.a_dzThNull = ...
                fmincon(   @(a) ...
                abs(Path2_Mobility.interpThetaPanelFromIntTime...
                                        (aI, dzI(:, thetaSelect), a(1))) ...
                + abs(Path2_Mobility.interpThetaPanelFromIntTime...
                                        (aJ, dzJ(:, thetaSelect), a(2))), ...
                        [0, 0], [], [], [], [], ... % ic and no constraints
                        [min(aI, [], "all"), min(aJ, [], "all")], ... % lb
                        [max(aI, [], "all"), max(aJ, [], "all")], ... % ub
                        [], ... % no nonlinear constraints and switch off display
                        optimoptions("fmincon", "Display", "none"));
        end

        % generate the input-space neede to conduct the movility analysis
        % ... the second argument is the number of discretization points to
        % ... divide each input dimension into. So, for "origin-only" mode,
        % ... each dimension is sampled at ('dNumU') which makes the entire
        % ... sweep have ('dNumU')^4
        function thisAltGait = generateInputSpace( thisAltGait, dNumU, T )
            % make sure we are in the 'origin-only' mode
            if ~strcmp(thisAltGait.leafExplorationMode, 'origin-only')
                error(['ERROR! Only origin exploration mode is supported ' ...
                    'for now']);
            end
            % make sure the discretization number is atleast 3; and convert
            % it to an odd number if possible
            if dNumU < 3
                error(['ERROR! The input discretization number should be ' ...
                    'atleast 3 and preferably odd.']);
            else
                % ... if even, convert to odd by removing a dimension a
                % discretization point
                if rem(dNumU, 2) == 0
                    dNumU = dNumU - 1;
                end
            end
            % initialize and declare input space properties
            thisAltGait.inputSpace = [];
            thisAltGait.inputSpace.dNumU = dNumU;
            % ... input sweep in each direction: this is defined
            % ... irrespective of 'inputMode' as that only changes the
            % ... mapping of the inputs to the path integration times
            % ... u1, u2 are the scaling & sliding inputs for "ithStance"
            % ... u3, u4 are the same for "jthStance"
            u = cell(1, 4); uGrid = cell(1, 4);
            for k = 1:4 % input discretizations in each dimension
                if k == 1
                    u{k} = linspace(-1, 1, dNumU);
                else
                    u{k} = u{k-1};
                end
            end
            % ... data in gridded (subgaitwise) input subspaces
            % ... ... obtain the gridded inputs
            % ... ... obtain integration times for the estimation procedure
            % ... ... obtain the number of discretizations for different
            tIC = cell(1, 2); tFC = tIC; disc = tFC;
            for k = 1:2
                kStart = 2*(k-1)+1; kEnd = 2*k; % mesh indices
                [uGrid{kStart}, uGrid{kEnd}] = ...
                        meshgrid(u{kStart}, u{kEnd}); % meshed inputs
                % ... unpack subgait integration times and max times
                Tnow = T{k};
                tMaxNow = thisAltGait.stanceSpace.aLimits{k};
                % ... compute the initial and final times for the paths
                [tIC{k}, tFC{k}] = ...
                    altQuadGait.computeSubgaitIntegrationTimes...
                    (thisAltGait, ...
                    [uGrid{kStart}(:), uGrid{kEnd}(:)], ... % col inputs
                    Tnow, [], tMaxNow);
                tIC{k} = reshape(tIC{k}, size(uGrid{kStart}));
                tFC{k} = reshape(tFC{k}, size(uGrid{kEnd})); % grid of times
                % ... unpack
                dnumNow = thisAltGait.stanceSpace.dnum{k};
                % ... compute the discretizations for each path
                % ... ... the max length path gets 'dNumNow' points
                % ... ... if the path length is zero => one point
                % ... ... everything else atleast two points
                pathLengthNow = abs(tFC{k} - tIC{k});
                disc{k} = round(pathLengthNow/diff(tMaxNow)*dnumNow);
                zeroPathLocs = pathLengthNow == 0; % zero path length locs
                disc{k}(zeroPathLocs) = 1; % set those locs to 1 disc
                illPathlocs = (~zeroPathLocs & (disc{k} < 2));
                disc{k}(illPathlocs) = 2; % finite paths with < 2 disc to 2
            end
            % obtain the discretization for the estimation paths
            discNum(:, :, 1) = disc{1}; discNum(:, :, 2) = disc{2};
            discNum = max(discNum, [], 3); % get the max disc for z
            % ... store the inputs in their own fields
            thisAltGait.inputSpace.u = u;
            thisAltGait.inputSpace.uGrid = uGrid;
            % ... init and store in 'intParam' field and return
            % ... ... integration times for IC and FC poitns
            % ... ... overall discretization number for the estimation path
            thisAltGait.inputSpace.intParam.tIC  = tIC;
            thisAltGait.inputSpace.intParam.tFC  = tFC;
            thisAltGait.inputSpace.intParam.disc = discNum;
        end

        % simulate the input space to obtain ground-truth reachable sets
        function thisAltGait = simulateInputSpace( thisAltGait )
            % unpack
            % ... subgaits from 'ithStance' and 'jthStance' props
            stances = {thisAltGait.ithStance, thisAltGait.jthStance};
            % ... integration params from 'inputSpace' prop
            dNumU = thisAltGait.inputSpace.dNumU;
            tIC = thisAltGait.inputSpace.intParam.tIC;
            tFC = thisAltGait.inputSpace.intParam.tFC;
            % ... simulate the body displacement from both subgaits
            zSub = cell(1, 2); % cells for each subgait
            for k = 1:numel(stances) % iterate over each subgait
                zSub{k} = cell(1, 3); % subcells for each component
                for i = 1:dNumU % iterate over the scaling input
                    for j = 1:dNumU % over sliding input
                        % displacement for current input pair in current
                        % stance
                        % ... obtain the displacement
                        % ... split it componentwise and store
                        zSubNow = ...
                            Path2_Mobility.simulateFinalBodyPosition...
                            (zeros(1, 2), ... % origin-only
                            [tIC{k}(i, j) tFC{k}(i, j)], ... % bwd and fwd times
                            stances{k}); % current "Path2_Mobility" instance
                        for iComp = 1:3 % iterate and assign components
                            zSub{k}{iComp}(i, j) = zSubNow(iComp);
                        end
                    end
                end
            end
            % ... stitch the displacements together
            % ... ... 1) iterate over the 4-dimensional input space and 
            % ... ... obtain each displacement component as column vectors
            % ... ... 2) compute the pagewise product
            % ... ... 3) again iterate over the input space and assign it
            % ... ... element by element
            zSubI = zSub{1}; zSubJ = zSub{2};
            zSubIcols = nan(0, 3); zSubJcols = zSubIcols;
            % ... ... iterate and compile
            for i = 1:dNumU % scaling for Ith subgait
                for j = 1:dNumU % sliding for Ith subgait
                    for k = 1:dNumU % scaling for Jth subgait
                        for l = 1:dNumU % sliding for Jth subgait
                            % current subgait displacements
                            zSubIcols(end+1, :) = ...
                                [zSubI{1}(i, j), ...
                                zSubI{2}(i, j), zSubI{3}(i, j)];
                            zSubJcols(end+1, :) = ...
                                [zSubJ{1}(k, l), ...
                                zSubJ{2}(k, l), zSubJ{3}(k, l)];
                        end
                    end
                end
            end
            % ... ... stitch the displacements sequentially
            % ... ...  also compute the average body velocity
            zCols = stitchTwoSE2displacements( zSubIcols, zSubJcols );%%%%%
            % gCircCols = logMapOfALieGroupElement( zCols );%%%%%%%%%%%%%%%%%
            zCols = mat2cell(zCols, size(zCols, 1), ones(size(zCols,2),1));
            % gCircCols = mat2cell(gCircCols, ...
                            % size(gCircCols, 1), ones(size(gCircCols, 2)));
            % ... ... iterate again and assign
            z = cell(size(zCols)); 
            % gCirc = cell(size(gCircCols));
            for iComp = 1:numel(z)
                z{iComp} = nan( dNumU*ones(1, 4) ); % init cells
                % gCirc{iComp} = nan( dNumU*ones(1, 4) );
                loopCount = 1; % init loop count
                for i = 1:dNumU
                    for j = 1:dNumU
                        for k = 1:dNumU
                            for l = 1:dNumU
                                z{iComp}(i, j, k, l) = ...
                                    zCols{iComp}(loopCount); % assign
                                % gCirc{iComp}(i, j, k, l) = ...
                                %     zCols{iComp}(loopCount);
                                loopCount = loopCount + 1;
                            end
                        end
                    end
                end
            end
            % ... initialize the "sim"ulation field within 'inputSpace'
            % ... property and assign the results
            thisAltGait.inputSpace.sim.z = z;
            % thisAltGait.inputSpace.sim.gCirc = gCirc;
        end

        % compute the cost associate with executing the alternating gait
        % ... there are two costs related to the shape velocity and
        % ... acceleration, we want to compute both as a function of the
        % ... input space
        % ... NOTE: SETUP VERY SIMILAR TO SIMULATE INPUT SPACE
        function thisAltGait = computeGaitCost( thisAltGait )
            % general unpacking
            stances = {thisAltGait.ithStance, thisAltGait.jthStance};
            dNumU = thisAltGait.inputSpace.dNumU;
            tIC = thisAltGait.inputSpace.intParam.tIC;
            tFC = thisAltGait.inputSpace.intParam.tFC;
            % recompute the specific level-set at the origin
            % ... assumes that the only leaf-exploration mode is
            % ... "origin-only"
            switch strcmp(thisAltGait.leafExplorationMode, 'origin-only')
                case 1
                    for k = 1:numel(stances)
                        stances{k} = ...
                            Path2_Mobility...
                                .computeSpecificParallelCoordinates...
                                    ( stances{k}, zeros(1,2), [], true );
                    end
                otherwise
                    error(['ERROR! Only "origin-only" exploration mode is ' ...
                        'selected at the moment']);
            end
            % get the shape velocity and acceleration costs for each 
            % subgait
            cost = []; cost.aVel = cell(1, 2); cost.aAccln = cell(1, 2);
            for k = 1:numel(stances)
                tVelNow = stances{k}.aParaRef.t(1:end-1); 
                aVelNow = stances{k}.aParaRef.aVel;
                tAcclnNow = stances{k}.aParaRef.t(1:end-2); 
                aAcclnNow = stances{k}.aParaRef.aAccln;
                for i = 1:dNumU
                    for j = 1:dNumU
                        tQueryNow = linspace(tIC{k}(i, j), tFC{k}(i, j));
                        cost.aVel{k}(i, j) = ...
                            abs(trapz( tQueryNow, ...
                                interp1(...
                                    tVelNow, aVelNow, tQueryNow, "pchip"...
                                         )...
                                    ));
                        cost.aAccln{k}(i, j) = ...
                            abs(trapz( tQueryNow, ...
                                interp1(...
                                tAcclnNow, aAcclnNow, tQueryNow, "pchip"...
                                         )...
                                    ));
                    end
                end
            end
            % get the stitched cost
            J = []; J.vel = nan(dNumU*ones(1, 4)); J.accln = J.vel;
            for i = 1:dNumU
                for j = 1:dNumU
                    for k = 1:dNumU
                        for l = 1:dNumU
                            J.vel(i, j, k, l) = ... % shape velocity
                                cost.aVel{1}(i, j) + cost.aVel{2}(k, l);
                            J.accln(i, j, k, l) = ... % shape accleration
                            cost.aAccln{1}(i, j) + cost.aAccln{2}(k, l);
                        end
                    end
                end
            end
            % pack and return
            thisAltGait.inputSpace.J = J;
        end

        % compute the cost associate with executing the alternating gait
        % ... right now we are just considering an offset cost of 1 for the
        % ... lift amplitudes
        function thisAltGait = computeMobility(thisAltGait, type)
            switch type
                case 'sim'
                    % unpack everything needed
                    z = thisAltGait.inputSpace.(type).z;
                    cost = thisAltGait.inputSpace.J;
                    % define the "cost" types to iterate over
                    costStr = {'vel', 'accln'};
                    % iterate and obtain the efficiency formulation
                    for iCost = 1:numel(costStr)
                        E.(costStr{iCost}) = cell(size(z));
                        for iComponent = 1:numel(z)
                            E.(costStr{iCost}){iComponent} = ...
                                z{iComponent}./(cost.(costStr{iCost}) + 1);
                        end
                    end
                    % pack the efficiency formulation
                    thisAltGait.inputSpace.(type).E = E;
                otherwise
                    error(['ERROR! Other methods apart from "sim" are not ' ...
                        'supported right now.']);
            end
        end

        % compute the boundaries for the set ('reachable' or 'mobility') 
        % provided
        function computeSetBoundaries(thisAltGait, type)
            switch type
                case 'sim'
                    % unpack the reachable set
                    z = thisAltGait.inputSpace.(type).z;
                    zX = z{1}(:); zY = z{2}(:); zYaw = z{3}(:);
                    % assign boundaries
                    % ... first to the 3D set
                    % ... next to the 2D projections
                    % ... these things make the set easier to visualize
                    fullyShrunkBoundary3D = boundary(zX, zY, zYaw, 1);
                    fullyShrunkBoundary2D = cell(1, 3); idxCyclic = 1:3;
                    for i = 1:3
                        fullyShrunkBoundary2D{i} = boundary(z{idxCyclic(1)}(:), z{idxCyclic(2)}(:), 1);
                        idxCyclic = circshift(idxCyclic, -1);
                    end
                    idxCyclic = circshift(idxCyclic, -1); % back to original ordering
                    % define some strings for plotting
                    dirnLabelTxt = {'Lateral (x)', ...
                                    'Longitudinal (y)', ... 
                                    'Rotational ($\theta$)'};
                    % pack everything
                    thisAltGait.inputSpace.(type).plot.idxCyclic = idxCyclic;
                    thisAltGait.inputSpace.(type).plot.dirnLabelTxt = dirnLabelTxt;
                    thisAltGait.inputSpace.(type).plot.zBounds3D = ...
                                                    fullyShrunkBoundary3D;
                    thisAltGait.inputSpace.(type).plot.zBounds2D = ...
                                                    fullyShrunkBoundary2D;
                otherwise
                    error(['ERROR! Other methods apart from "sim" are not ' ...
                        'supported right now.']);
            end
        end
        
        % this function computes the integration times for both subgaits
        % that form the "thisAltGait" instance
        function [tIC, tFC] = computeGaitIntegrationTimes...
                                    (thisAltGait, inputs, T, Toff, Tmax)
            % ensure that the variables are the correct sizes
            if size(T, 1) ~= 1
                error(['ERROR! Only one set of integration times are ' ...
                    'required.']);
            end
            if size(Toff, 1) ~= 1
                error(['ERROR! There should only be one pair of values for ' ...
                    'the offset time']);
            end
            % unpack inputs, path integration times, and offset times
            % ... the inputs are setup as column vectors ordered from left 
            % ... to right as scaling, sliding for Si, and same for Sj 
            ithInput = inputs(:, 1:2); jthInput = inputs(:, 3:4);
            ithIntgTimes = T(1:2); jthIntgTimes = T(3:4);
            ithOffTime = Toff(1); jthOffTime = Toff(2);
            % obtain the integration times individually for each subgait
            % ... if the max times are provided use that
            % ... else just pass the rest and limits are computed inside
            switch nargin == 5
                case 1
                    ithIntgMAXTimes = Tmax(1:2); 
                    jthIntgMAXTimes = Tmax(3:4);
                    [tICi, tFCi] = ...
                        altQuadGait.computeSubgaitIntegrationTimes...
                                (thisAltGait, ...
                                ithInput, ithIntgTimes, ithOffTime, ...
                                ithIntgMAXTimes);
                    [tICj, tFCj] = ...
                        altQuadGait.computeSubgaitIntegrationTimes...
                                (thisAltGait, ...
                                jthInput, jthIntgTimes, jthOffTime, ...
                                jthIntgMAXTimes);
                case 0
                    [tICi, tFCi] = ...
                        altQuadGait.computeSubgaitIntegrationTimes...
                                (thisAltGait, ...
                                ithInput, ithIntgTimes, ithOffTime);
                    [tICj, tFCj] = ...
                        altQuadGait.computeSubgaitIntegrationTimes...
                                (thisAltGait, ...
                                jthInput, jthIntgTimes, jthOffTime);
            end
            tIC = [tICi, tICj]; 
            tFC = [tFCi, tFCj]; % return columnwise concatenated arrays
        end

        % this function computes the integration time for a requested
        % subgait
        function [tIC, tFC] = ...
                            computeSubgaitIntegrationTimes...
                                    (thisAltGait, inputs, T, ~, Tmax)
            % Compute initial and final condition times based on the
            % current input mode
            u1 = inputs(1); u2  = inputs(2);
            T0 =      T(1); Tpi =      T(2);
            % assign the initial and final times as they remain unchanged
            initTime = -u1*T0 ;
            finTime  = +u1*Tpi;
            switch thisAltGait.inputMode
                case 'std'
                    tIC = initTime + u2;
                    tFC = finTime  + u2;
                case 'path_limit_compliant'
                    % first, if the Tmax is not provided, obtain it from
                    % the "altQuadGait" instance: "thisAltGait"
                    % if limits are provided, use that
                    if nargin < 5
                        Tmax = thisAltGait.integrationLimits;
                    end
                    % check the quadrant of the input to compute the
                    % capping term associated with the sliding input
                    % ... we negate Tminus because we are moving backwards
                    Tminus = Tmax(1); Tplus = Tmax(2); 
                    switch u2 < 0 
                        case 1 % MOVING LEFT
                            switch initTime <= finTime
                                case 1 % initTime closer to Tminus
                                    capTerm = initTime - Tminus;
                                case 0 % finTime  closer to Tminus
                                    capTerm = finTime - Tminus;
                            end
                        case 0 % MOVING RIGHT
                            switch finTime >= initTime
                                case 1 % finTime  closer to Tplus
                                    capTerm = Tplus - finTime;
                                case 0 % initTime closer to Tplus
                                    capTerm = Tplus - initTime;
                            end
                    end
                    % compute the initial and final condition integration
                    % times
                    tIC = initTime + capTerm*u2;
                    tFC = finTime  + capTerm*u2;
            end
        end

        % compute the full configuration trajectory when provided with 
        % individual configuration trajectories during each stance phase
        function cT = fullCycleConfigTrajectory(thisAltGait, cTi, cTj, ...
                                                    nAppxOrder, zoomFlag)
            % ... unpack the subgait instances
            stance_i = thisAltGait.ithStance;
            stance_j = thisAltGait.jthStance;
            % ... length of the shape trajection in the stance space
            cT = []; % init output
            cT.parameters.length = ... % shape traj length in stance space
                mean([size(stance_i.ai, 1) size(stance_j.ai, 1)]);
            % ... iterate over each field we want to obtain the
            % ... configuration trajectory over
            for fieldName = {'complete', 'discretized'}
                % extract the fieldName out of the cell array
                fieldNow = fieldName{1};
                % based on the complete or discretized case, setup
                % accordingly
                switch fieldNow
                    case 'complete'
                        dnum = ceil(norm([size(stance_i.ai, 1), ...
                                          size(stance_j.ai, 1)]));
                        cT.(fieldNow).T = linspace(0, cT.parameters.length, ...
                                                                    dnum)';
                    case 'discretized'
                        cT.parameters.disc = ceil(...
                                                mean( ...
                            [ cT.parameters.length/sum(stance_i.intTime)...
                                                *size(stance_i.ai, 1), ...
                              cT.parameters.length/sum(stance_j.intTime)...
                                                *size(stance_j.ai, 1) ] ...
                                                    )...
                                                 );
                        if cT.parameters.disc == 0
                            cT.parameters.disc = cT.parameters.disc + 1;
                        end
                        dnum = cT.parameters.disc;
                        cT.(fieldNow).T = linspace(0, ...
                                                    cT.parameters.length, ...
                                                    dnum)';
                end
                % compute everything needed -------------------------------
                % ... setup body velocity for the full trajectory
                cT = dzAndGcircForConfigTrjectory(cT, cTi, fieldNow,dnum);
                cT = dzAndGcircForConfigTrjectory(cT, cTj, fieldNow,dnum);
                % ... obtain the simulated trajectories in each stance phase
                % ... and stitch it together
                g_i = cTi.(fieldNow).g; z_i = cTi.(fieldNow).z';
                g_j = cTj.(fieldNow).g; z_j = cTj.(fieldNow).z';
                [cT.(fieldNow).g, cT.(fieldNow).z] = ...
                            stitchTwoSE2trajectories(g_i, z_i, g_j, z_j);
                % ... obtain the lie-algebra terms for flowing along the 
                % ... body velocity generated by each subgait commutatively
                gC_i = cT.(fieldNow).gCirc{1}(:, 1:3); 
                gC_j = cT.(fieldNow).gCirc{1}(:, 4:6);
                % ... obtain the piecewise commutative trajectory
                switch isempty(nAppxOrder)
                    case 1
                        cT.(fieldNow).gHat{1} = [];
                        cT.(fieldNow).zHat{1} = [];
                    case 0
                        [cT.(fieldNow).gHat{1}, cT.(fieldNow).zHat{1}] = ...
                            altQuadGait.piecewiseBodyVelFlow(gC_i, gC_j);
                end
                % ... obtain approximations of piecewise trajectory to the
                % ... first "nAppxOrder" orders of the BCH expansion get 
                % ... the estimations of body velocities, trajectory, and 
                % ... displacements.
                [cT.(fieldNow).gCirc((1:nAppxOrder)+1), ... vel
                    cT.(fieldNow).gHat((1:nAppxOrder)+1), ... traj
                        cT.(fieldNow).zHat((1:nAppxOrder)+1)] = ... disp
                      altQuadGait.BCHexpansion(gC_i, gC_j, 1:nAppxOrder); 
            end
            % plot the estimate body trajectories for visualization
            % purposes if needed-- only happens if a "zoomFlag" option is
            % provided
            if nargin == 5
                % init figure
                figure('Visible', 'on', 'Units', 'pixels', ...
                    'Position', [0 0 900 900]);
                ax = gca; hold(ax, "on"); box(ax, "on");
                % get the body trajectory
                b = cT.discretized.g;
                % get the transforms to the legs
                bodyLimbTransform
                % iterate and plot the body bounding box and leg locations
                for ctr = 1:size(b, 1)
                    % current body locations
                    currBodyPos = b(ctr, :)';
                    % compute and plot the locations of the body corners
                    % ... the body bounding box is plotted at the end of 
                    % ... the trajectory to provide "visually" an estimate 
                    % ... of the displacement in body-lengths (BLs)
                    xCorners = l*[1, -1, -1, 1];
                    yCorners = 2*circshift(xCorners, 1);
                    hbCorners = [xCorners; 
                                 yCorners; 
                                 zeros(size(xCorners))];
                    heCorners = nan(size(hbCorners));
                    for i = 1:size(hbCorners, 2)
                        heCorners(:, i) = seqSE2transformation(...
                                        [currBodyPos, hbCorners(:, i)]...
                                                                );
                    end
                    xCorners = heCorners(1, :); yCorners = heCorners(2, :);
                    uCorners = circshift(xCorners, -1) - xCorners; 
                    vCorners = circshift(yCorners, -1) - yCorners;
                    p = quiver(ax, xCorners, yCorners, uCorners, vCorners, ...
                    "AutoScale", "off", 'ShowArrowHead', 'off',...
                    'LineWidth', 1.2, 'LineStyle', '-', 'Color', gbCol);
                    set(get(get(p,'Annotation'),'LegendInformation'),...
                            'IconDisplayStyle','off');
                    % compute and plot the leg locations in the respective
                    % colors
                    for i = 1:4 % iterate over each limb and plot locs
                        currLimbPos = seqSE2transformation(...
                            [currBodyPos]...
                                                            );
                    end
                end
                altQuadGait.plotBodyTrajectoryEstimates...
                                (ax, cT.discretized, zoomFlag, stance_i.l);
            end
        end

        % compute the piecewise commutative approximation for flowing along
        % two sequential body velocities ("gCi" and "gCj") for unit time 
        % each.
        function [gHat, zHat] = piecewiseBodyVelFlow(gC_i, gC_j)
            % first, flow along "gC_i" for unit time
            [gHat_i, zHat_i] = exponentiateBodyVelocities(gC_i);
            % second, flow along "gC_j" for unit time to obtain a
            % piecewise, commutative trajectory approximation of the
            % alternating quadrupedal gait cycle
            [gHat_j, zHat_j] = exponentiateBodyVelocities(gC_j);
            [gHat, zHat] = stitchTwoSE2trajectories(gHat_i, zHat_i, ...
                                                        gHat_j, zHat_j);
        end

        % compute the Baker-Campbell-Hausdorff (BCH) expansion of the two
        % body velicities to the requested order. This only supports two
        % sequential body velocities and a maximum estimation order of upto
        % 4 for the BCH expansion.
        % ... Need to improve this to arbitrary order with arbitrary number
        % ... of vectors using the formula in the BCH Wikipedia page: 
        % ... https://en.wikipedia.org/wiki/Baker%E2%80%93Campbell%E2%80%93Hausdorff_formula
        function [gCHat, gHat, zHat] = BCHexpansion(gC_i, gC_j, n)
            % if n is an array, we output cell arrays of each listed output
            % for the requested orders, else we just output one timeseries
            % for each
            switch numel(n)
                case 0 % return empty cells
                    gCHat = cell(1, 1); gHat = gCHat; zHat = gHat;
                case 1
                    for i = 1:n
                        switch i
                            case 1
                                gC = gC_i + gC_j;
                            case 2
                                gC_i_j = ...
                                    computeLieBracketOfse2VectorFields...
                                                        (gC_i, gC_j);
                                gC = gC + gC_i_j/2;
                            case 3
                                gC_iMj_i_j = ...
                                    computeLieBracketOfse2VectorFields...
                                                    (gC_i-gC_j, gC_i_j);
                                gC = gC + gC_iMj_i_j/12;
                            case 4
                                gC_i_i_j = ...
                                    computeLieBracketOfse2VectorFields...
                                                        (gC_i, gC_i_j);
                                gC_j_i_i_j = ...
                                    computeLieBracketOfse2VectorFields...
                                                        (gC_j, gC_i_i_j);
                                gC = gC + gC_j_i_i_j/24;
                        end
                    end
                    gCHat = {gC};
                    [gHat, zHat] = exponentiateBodyVelocities(gC);
                    gHat = {gHat}; zHat = {zHat}; % cellularize
                otherwise
                    gCHat = cell(1, n(end)); 
                    gHat = gCHat; zHat = gCHat; % init
                    for i = n
                        switch i
                            case 1
                                gC = gC_i + gC_j;
                            case 2
                                gC_i_j = ...
                                    computeLieBracketOfse2VectorFields...
                                                        (gC_i, gC_j);
                                gC = gC + gC_i_j/2;
                            case 3
                                gC_iMj_i_j = ...
                                    computeLieBracketOfse2VectorFields...
                                                    (gC_i-gC_j, gC_i_j);
                                gC = gC + gC_iMj_i_j/12;
                            case 4
                                gC_i_i_j = ...
                                    computeLieBracketOfse2VectorFields...
                                                        (gC_i, gC_i_j);
                                gC_j_i_i_j = ...
                                    computeLieBracketOfse2VectorFields...
                                                        (gC_j, gC_i_i_j);
                                gC = gC + gC_j_i_i_j/24;
                        end
                        gCHat{i} = gC; % compute element
                        [gHat{i}, zHat{i}] = ...
                                        exponentiateBodyVelocities(gC);
                    end
            end
        end
        
        function [pFS, subGT, subGTFS] = ...
                    obtainGaitFSapproximation...
                        (thisAltGait, refi, refj, inputs, ...
                         nOrderFS, pltStruct, packCSVflag)
        %OBTAINGAITFSAPPROXIMATION approximate the limb fore-aft angles
        %defined by the continuous shape space using Fourier series
        %approximation
            % because this function only supports one set of inputs, check to
            % makre sure this is the case
            if size(inputs, 1) ~= 1
                error(['ERROR! Only one set of inputs are needed and the ' ...
                '"inputs" variable should be a row vector of 4 elements']);
            end
            % if the flags and mode is not provided, define default values
            if nargin < 7
                packCSVflag = false; % do not generate data if not required
            elseif nargin < 6
                pltStruct.flag = true;
                pltStruct.mode = 'colored'; % modes- 'stance_colored', 'k'
            elseif nargin > 7
                error(['ERROR! The number of input arguments can not be ' ...
                    'more than 7.']);
            end
            % unpack the subgait instances
            stance_i = thisAltGait.ithStance;
            stance_j = thisAltGait.jthStance;
            % obtain offset times and max time references
            [refi.tOff, refi.tMax, stance_i] = ...
                Path2_Mobility.computeToffFromPerpCoord(refi.P, stance_i);
            [refj.tOff, refj.tMax, stance_j] = ...
                Path2_Mobility.computeToffFromPerpCoord(refj.P, stance_j);
            % obtain the integration times to the stance path initial and 
            % final conditions
            [tIC, tFC] = ...
                altQuadGait.computeGaitIntegrationTimes...
                (    thisAltGait, ... % gait instance
                     inputs, ... % scaling and sliding inputs concatenated
                     [refi.T,    refj.T], ... % integration times
                     [refi.tOff, refj.tOff], ... % offset times
                     [refi.tMax, refj.tMax]   ); % max integration times
            % ... unpack integration times for each subgait
            tICi = tIC(1); tICj = tIC(2);
            tFCi = tFC(1); tFCj = tFC(2);
            % obtain the subgait trajectories
            subGTi = []; 
            [subGTi.tau, subGTi.beta, subGTi.subgait] = ...
                        Path2_Mobility.computeSubgaitInCoordinates...
                                        (refi, [tICi, tFCi], stance_i);
            subGTj = [];
            [subGTj.tau, subGTj.beta, subGTj.subgait] = ...
                        Path2_Mobility.computeSubgaitInCoordinates...
                                        (refj, [tICj, tFCj], stance_j);
            subGT = {subGTi, subGTj};
            % ... offset the jth subgait by +pi to make the gait 
            % ... alternating in nature-- else the whole gait construction 
            % ... would be wrong; '+pi' is the last argument
            [subGTj.beta, subGTj.subgait] = ...
                    Path2_Mobility.phaseOffsetSubgait...
                        (subGTj.tau, subGTj.beta, subGTj.subgait, +pi);
            % approximate the trajectories until the requested order
            pFS = cell(1, 2); pFS{1} = []; pFS{2} = [];
            [pFS{1}.a0, pFS{1}.a, pFS{1}.b, subGTFSi] = ...
            generateFourierSeriesAppx...
                                (subGTi.tau, subGTi.subgait, nOrderFS);
            [pFS{2}.a0, pFS{2}.a, pFS{2}.b, subGTFSj] = ...
            generateFourierSeriesAppx...
                                (subGTj.tau, subGTj.subgait, nOrderFS);
            subGTFS = {subGTFSi, subGTFSj};
            % plot them together for comparison if neeeded
            % ... first put each subgait and its Fourier series estimate
            % ... into the correct plotting format
            if pltStruct.flag
                plt_i = formatLimbSignalsForPlotting...
                    (subGTi.tau, subGTi.beta, subGTi.subgait, stance_i);
                plt_j = formatLimbSignalsForPlotting...
                    (subGTj.tau, subGTj.beta, subGTj.subgait, stance_j);
                pltFSi = formatLimbSignalsForPlotting...
                    (subGTi.tau, subGTi.beta, subGTFSi, stance_i, ...
                                                            false, ':');
                pltFSj = formatLimbSignalsForPlotting...
                    (subGTj.tau, subGTj.beta, subGTFSj, stance_j, ...
                                                            false, ':');
                plotLimbAngleSignals(...
                                        {   plt_i, pltFSi; ...
                                            plt_j, pltFSj      }, ...
                                            pltStruct.mode...
                                    );
            end
            % package the fore-aft angle into CSV if neeeded
            % ... coefficients along rows with order a0, a, and then b
            % ... which correspond to the DC offset term, Cosine
            % ... components, and the Sine components.
            if packCSVflag
                iStance = [];
                    iStance.cs = stance_i.cs;
                        iStance.tau = subGTi.tau; 
                            iStance.subgait = subGTi.subgait;
                                iStance.nAppxOrder = nOrderFS;
                jStance = [];
                    jStance.cs = stance_j.cs;
                        jStance.tau = subGTj.tau; 
                            jStance.subgait = subGTj.subgait;
                                jStance.nAppxOrder = nOrderFS;
                compileWaveformsFromFSappx(... % generate .csv
                    iStance, jStance, ...
                    ['Data\rigidTrotGait_' num2str(inputs) '_' ...
                    thisAltGait.inputMode '_FSparams' '.csv']);
            end
        end

        function plotBodyTrajectoryEstimates...
                (ax, discJointStancePath, zoomFlag, l)
        %PLOTALTGAITBODYTRAJECTORYESTIMATES plot the body trajectory of the 
        %system when performing an alternating gait and its estimates using 
        %the BCH formula
            % check if the jointStancePath information is provided
            if ~isfield(discJointStancePath, 'g') ||...
               ~isfield(discJointStancePath, 'z') ||...
               ~isfield(discJointStancePath, 'gHat') ||...
               ~isfield(discJointStancePath, 'zHat')
                error(['The struct "jointStancePath.discretized" does not contain ' ...
                    'the fields required to plot the body trajectory and its ' ...
                    'estimates.']);
            end
            % check if zoomFlag is provided
            if nargin < 2
                zoomFlag = false;
            end
            % unpack
            b = discJointStancePath.g;
            zb = discJointStancePath.z;
            % compute the locations of the body corners
            % ... the body bounding box is plotted at the end of the
            % ... trajectory to provide "visually" an estimate of the
            % ... displacement in body-lengths (BLs)
            xCorners = l*[1, -1, -1, 1];
            yCorners = 2*circshift(xCorners, 1);
            hbCorners = [xCorners; yCorners; zeros(size(xCorners))];
            heCorners = nan(size(hbCorners));
            for i = 1:size(hbCorners, 2)
                heCorners(:, i) = seqSE2transformation(...
                                            [zb(:), hbCorners(:, i)]...
                                                        );
            end
            xCorners = heCorners(1, :); yCorners = heCorners(2, :);
            uCorners = circshift(xCorners, -1) - xCorners; 
            vCorners = circshift(yCorners, -1) - yCorners;
            % setups
            fS = 25; 
            appxPicewCol = [51,160,44]/255;
            appxCol = [31,120,180]/255; appxfA = 0.2*(1:4);
            gbCol = [0, 0, 0]/255;
            
            % plot the body trajectory, represent the net displacement, and the body
            % orientation
            % ... also, plot body bounding box for body-length reference
            plot(ax, b(:, 1), b(:, 2), '-', 'LineWidth', 2.0, 'Color', gbCol, ...
                'DisplayName', 'simulated trajectory');
            grid(ax, "on"); hold(ax, "on"); set(ax,'TickLabelInterpreter','latex');
            p = scatter(ax, zb(1), zb(2), 100, 'o', "filled", "MarkerFaceColor", gbCol, 'MarkerFaceAlpha', 1.00);
            set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            p = quiver(ax, zb(1),  zb(2), -sin(zb(3)), cos(zb(3)),...
                'LineWidth', 3.0, 'Color', gbCol, 'LineStyle', '-',...
                'AutoScaleFactor', 0.1, 'MaxHeadSize', 1);
            set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            p = quiver(ax, xCorners, yCorners, uCorners, vCorners, ...
                "AutoScale", "off", 'ShowArrowHead', 'off',...
                'LineWidth', 1.2, 'LineStyle', '--', 'Color', gbCol);
            set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            % plot the stancewise commutative estimate
            gHatb = discJointStancePath.gHat{1}; 
            zHatb = discJointStancePath.zHat{1}; % locally unpack
            if ~isempty(gHatb)
                plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0,...
                'Color', [ appxPicewCol, 1.0 ], ...
                    'DisplayName', 'stancewise commutative');
                p = scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
                        "MarkerFaceColor", appxPicewCol, ...
                        'MarkerFaceAlpha', 1.0, ...
                        'MarkerEdgeColor', appxPicewCol, ...
                        'MarkerEdgeAlpha', 1.0);
                set(get(get(p,'Annotation'),'LegendInformation'),...
                    'IconDisplayStyle','off');
            end
            % plot the single flow estimates
            for i = 2:numel(discJointStancePath.gHat)
                gHatb = discJointStancePath.gHat{i}; 
                zHatb = discJointStancePath.zHat{i}; % unpack for each case
                plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0, 'Color', [ appxCol, appxfA(i-1) ], ...
                    'DisplayName', ['$$' num2str(i-1) '^{\circ}$$ approximation']);
                p = scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
                        "MarkerFaceColor", appxCol, 'MarkerFaceAlpha', appxfA(i-1), ...
                        'MarkerEdgeColor', appxCol, 'MarkerEdgeAlpha', appxfA(i-1));
                set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            end
            % remaining setup
            p = xline(0, 'k:', 'LineWidth', 0.1); 
            set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            p = yline(0, 'k:', 'LineWidth', 0.1);
            set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            legend(ax, 'location', 'bestoutside', 'Interpreter', 'latex');
            axis(ax, "equal", "padded"); 
            ax.FontSize = fS; ax.XTick = ''; ax.YTick = '';
            xlabel('$$x$$', 'FontSize', fS, 'Interpreter', 'latex');
            ylabel('$$y$$', 'FontSize', fS, 'Interpreter', 'latex');
            % set(get(ax,'YLabel'), 'rotation', 0, ...
            %                                 'VerticalAlignment', 'middle');
            %%% ADDITIONAL PLOT WITHOUT LEGEND FOR ZOOMING IN, ETC.
            % ... same shit as the last plot, except without the labels, legends,
            % ... and any other text
            if zoomFlag
                figure('Visible', 'on', 'Units', 'pixels', 'Position', [0 0 600 600]); ax = gca; box(ax, "on");
                plot(ax, b(:, 1), b(:, 2), '-', 'LineWidth', 2.0, 'Color', gbCol);
                grid(ax, "on"); hold(ax, "on"); set(ax,'TickLabelInterpreter','latex');
                scatter(ax, zb(1), zb(2), 100, 'o', "filled", "MarkerFaceColor", gbCol, 'MarkerFaceAlpha', 1.00);
                gHatb = discJointStancePath.gHat{1}; zHatb = discJointStancePath.zHat{1};
                plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0, 'Color', [ appxPicewCol, 1.0 ]);
                scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
                        "MarkerFaceColor", appxPicewCol, 'MarkerFaceAlpha', 1.0, ...
                        'MarkerEdgeColor', appxPicewCol, 'MarkerEdgeAlpha', 1.0);
                for i = 2:numel(discJointStancePath.gHat)-1
                    gHatb = discJointStancePath.gHat{i}; zHatb = discJointStancePath.zHat{i};
                    plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0, 'Color', [ appxCol, appxfA(i) ]);
                    scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
                            "MarkerFaceColor", appxCol, 'MarkerFaceAlpha', appxfA(i), ...
                            'MarkerEdgeColor', appxCol, 'MarkerEdgeAlpha', appxfA(i));
                end
                xline(0, 'k:', 'LineWidth', 0.1); yline(0, 'k:', 'LineWidth', 0.1);
                axis(ax, "equal", "padded"); 
                ax.FontSize = fS; ax.XTick = ''; ax.YTick = '';
            end
        
        end
        
        % plot the body trajectory timeseries if the configuration
        % trajectories are already available
        function plotBodyTimeseries(thisAltGait, cTi, cTj, ...
                                                    pltMode, scatterFlag)
            % ... unpack the subgait instances
            stance_i = thisAltGait.ithStance;
            stance_j = thisAltGait.jthStance;
            % ... obtain the complete trajectories from each subgait
            g_i = cTi.complete.g; z_i = cTi.complete.z';
            g_j = cTj.complete.g; z_j = cTj.complete.z';
            [g, ~] = stitchTwoSE2trajectories(g_i, z_i, g_j, z_j);
            % ... contact trajectory
            % ... ... because it is a alternating gait, there are only two
            % ... ... stance phases dictated by the ordering of the
            % ... ... subgaits ("Path2_Mobility" instances)
            numPts = size(g, 1);
            c = [ones([floor(numPts/2), 1]); 2*ones([ceil(numPts/2), 1])];
            % ... obtain the plotting format and then plot the body
            % ... timeseries in separate layouts
            pltBody = formatBodyTimeseriesForPlotting...
                (g, c, ...
                pltMode, '-', scatterFlag, {stance_i, stance_j});
            plotBodyTimeseriesGeneral( pltBody );
        end

        % plot the body trajectory timeseries from inputs and references
        % ... this performs computations similar to the "noncommutative
        % ... approximation of net displacement" section in the
        % ... "se2_toyproblems_case_1_mobility.mlx" livescript;
        % ... once these computations are performed, then the function
        % ... above "altQuadGait.plotBodyTimeseries" is called in a loop
        % ... for each input value
        % ... also, have the option to choose between the body trajectory
        % ... or average velocity trajectory
        function plotBodyTimeseriesFromInputs(thisAltGait, ...
                            inputs, ref, ...
                            trajType, ... % for body avg vel or pos traj
                            traceColors, traceLabels) % trace related
            % ... unpack the references
            refI = ref{1}; refJ = ref{2};
            % ... unpack the stance phase instances
            stanceI = thisAltGait.ithStance; 
            stanceJ = thisAltGait.jthStance;
            % ... init "pltBody" cell array to store structs at each input
            % ... value
            pltBodyStructs = cell(size(inputs, 1), 1);
            % ... iterate over each input and compute the the configuration
            % ... trajectory
            for i = 1:size(inputs, 1)
                % ... ... inputs
                uNow = inputs(i, :); 
                % ... ... compute the gait integration times when provided 
                % ... ... with inputs and references
                switch thisAltGait.inputMode
                    case 'std'
                        [tICnow, tFCnow] = ...
                            altQuadGait.computeGaitIntegrationTimes...
                            (    thisAltGait, ... % gait instance
                                 uNow, ... % [scale, slide] inputs
                                [refI.T,    refJ.T], ... % int times
                                [refI.tOff, refJ.tOff], ... % offset times
                                [refI.tMax, refJ.tMax]   ); % int lim
                    case 'path_limit_compliant'
                        [tICnow, tFCnow] = ...
                            altQuadGait.computeGaitIntegrationTimes...
                            (    thisAltGait, ...
                                 uNow, ...
                                [refI.T,    refJ.T], ...
                                [refI.tOff, refJ.tOff]);
                end
                % ... ... compute the configuration trajectories for each
                % ... ... stance phase
                cTi = ...
                    Path2_Mobility.simulateConfigurationTrajectory...
                            (refI, ...
                            [tICnow(1), tFCnow(1)], ...
                             stanceI);
                cTj = ...
                    Path2_Mobility.simulateConfigurationTrajectory...
                            (refJ, ...
                            [tICnow(2), tFCnow(2)], ...
                             stanceJ);
                % ... ... extract the overall trajectory
                g_i = cTi.complete.g; z_i = cTi.complete.z';
                g_j = cTj.complete.g; z_j = cTj.complete.z';
                [gNow, ~] = stitchTwoSE2trajectories(g_i, z_i, g_j, z_j);
                % ... ... convert the body trajectory to average velocity
                % ... ... if needed
                if strcmp(trajType, 'gCirc') % 'g'
                    gNow = logMapOfALieGroupElement( gNow );
                end
                % ... ... extract the current contact trajectory
                numPts = size(gNow, 1);
                cNow = [    ones([floor(numPts/2), 1]); 
                          2*ones([ ceil(numPts/2), 1])   ];
                % ... obtain the plotting format
                pltBodyStructs{i} = ...
                    formatBodyTimeseriesForPlotting...
                    (...
                        gNow, cNow, ... % trajectories for current input
                        traceColors(i, :), ... % traj color
                        '-', ... % linestyle
                        false, ... % no scatters at contact change
                        {stanceI, stanceJ}... % stance instances
                    );
            end
            % ... call the plotting function for each struct
            plotBodyTimeseriesGeneral(pltBodyStructs,trajType,traceLabels);
        end
        
        % plot the shape trajectory from a gait cycle as a timeseries
        % ...  this function is very similar to the method 
        % ... "obtainGaitFSapproximation" (refer to this for more details)
        function plotShapeTimeseries(thisAltGait, ...
                                        refi, refj, inputs, pltStruct)
            if size(inputs, 1) ~= 1
                error(['ERROR! Only one set of inputs are needed and the ' ...
                '"inputs" variable should be a row vector of 4 elements']);
            end
            stance_i = thisAltGait.ithStance;
            stance_j = thisAltGait.jthStance;
            [refi.tOff, refi.tMax, stance_i] = ...
                Path2_Mobility.computeToffFromPerpCoord(refi.P, stance_i);
            [refj.tOff, refj.tMax, stance_j] = ...
                Path2_Mobility.computeToffFromPerpCoord(refj.P, stance_j);
            [tIC, tFC] = ...
                altQuadGait.computeGaitIntegrationTimes...
                (    thisAltGait, ...
                     inputs, ...
                     [refi.T,    refj.T], ...
                     [refi.tOff, refj.tOff], ...
                     [refi.tMax, refj.tMax]   );
            tICi = tIC(1); tICj = tIC(2);
            tFCi = tFC(1); tFCj = tFC(2);
            subGTi = []; 
            [subGTi.tau, subGTi.beta, subGTi.subgait] = ...
                        Path2_Mobility.computeSubgaitInCoordinates...
                                        (refi, [tICi, tFCi], stance_i);
            subGTj = [];
            [subGTj.tau, subGTj.beta, subGTj.subgait] = ...
                        Path2_Mobility.computeSubgaitInCoordinates...
                                        (refj, [tICj, tFCj], stance_j);
            if pltStruct.flag
                plt_i = formatLimbSignalsForPlotting...
                    (subGTi.tau, subGTi.beta, subGTi.subgait, stance_i);
                plt_j = formatLimbSignalsForPlotting...
                    (subGTj.tau, subGTj.beta, subGTj.subgait, stance_j);
                plotLimbAngleSignals({plt_i; plt_j}, pltStruct.mode);
            end
        end

        % plot the trajectory on the stance space panels
        function plotGaitInStanceSpace(thisAltGait, tIC, tFC, ...
                jointPathFlag, numReq, limReq)
            % unpack
            stanceI = thisAltGait.ithStance; colI = stanceI.p_info.gc_col;
            stanceJ = thisAltGait.jthStance; colJ = stanceJ.p_info.gc_col;
            fS = 25;
            switch numReq
                case 'cs'
                    xTxt = ['$$\alpha_{' 
                        num2str(thisAltGait.ithStance.cs) '}$$'];
                    yTxt = ['$$\alpha_{' 
                        num2str(thisAltGait.jthStance.cs) '}$$'];
                case '12'
                    xTxt = '$$t_{1}$$';
                    yTxt = '$$t_{2}$$';
            end
            % plot
            % ... init setup
            figure('Visible', 'on', 'Units', 'pixels',...
                'Position', [0 0 600 600]); 
            ax = gca; box(ax, "on");
            ax.XColor = colI; ax.YColor = colJ; ax.FontSize = fS;
            axis(ax, "equal", "tight"); hold(ax, "on");
            % ... plot thin dotted lines to identify origin
            xline(0, 'k:', 'LineWidth', 0.1);
            yline(0, 'k:', 'LineWidth', 0.1);
            % ... paths and ic scatters
            plot(ax, [tIC(1), tFC(1)], tIC(2)*ones(1, 2),...
                'LineWidth', 2.0, 'Color', colI);
            scatter(ax, tIC(1), tIC(2), 100, colI, "filled", "o",...
                "MarkerEdgeColor", 'k', 'LineWidth', 1.0);
            plot(ax, tFC(1)*ones(1, 2), [tIC(2), tFC(2)],...
                'LineWidth', 2.0, 'Color', colJ);
            scatter(ax, tFC(1), tIC(2), 100, colJ, "filled", "o",...
                "MarkerEdgeColor", 'k', 'LineWidth', 1.0);
            % ... ... plot the holonomic joint stance path if needed
            scatter(ax, tFC(1), tFC(2), 100, ...
                'Marker', 'x', 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
            % ... ... plot the joint path
            if jointPathFlag
                quiver(ax, tIC(1), tIC(2), tFC(1)-tIC(1), tFC(2)-tIC(2), ...
                'LineWidth', 2.0, 'AutoScale', 'off', 'Color', 'k', ...
                'LineStyle', '-','MaxHeadSize',0.3);
            end
            % ... final setup
            switch limReq
                case 'full'
                    ax.XLim = thisAltGait.ithStance.aParaRef.tMax.*[-1, 1];
                    ax.YLim = thisAltGait.jthStance.aParaRef.tMax.*[-1, 1];
                case 'limits'
                    ax.XLim = thisAltGait.stanceSpace.aLimits{1}; 
                    ax.YLim = thisAltGait.stanceSpace.aLimits{2};
            end
            xlabel(ax, xTxt, "FontSize", fS);
            ylabel(ax, yTxt, "FontSize", fS);
        end

        % plot the stance space panels
        function plotStanceSpacePanels(thisAltGait, titleReq, numReq)
            % ... unpack plotting data
            aI = thisAltGait.stanceSpace.A{1};
            aJ = thisAltGait.stanceSpace.A{2}; dnum = size(aI, 1);
            aIlimits = [ min(aI, [], "all"), max(aI, [], "all") ]; 
            aJlimits = [ min(aJ, [], "all"), max(aJ, [], "all") ];
            dzI= thisAltGait.stanceSpace.DZ{1};
            dzJ= thisAltGait.stanceSpace.DZ{2};
            % ... unpack plotting params
            p_info = thisAltGait.ithStance.p_info;
            lW_Vector = p_info.lW_Vector;
            skipV = 7.5; % in percentage
            skipV = round((skipV/100)*2*floor(dnum/2)); 
            idxQ = 1:skipV:dnum;
            fS = 25;
            A_title_txt = cell(1, 3);
            switch titleReq
                case 'panel'
                    A_txt = '\vec{dz}';
                    parallTxt = '';
                case 'connProj'
                    A_txt = '(\vec{A}';
                    parallTxt = '^{\parallel})';
            end
            switch numReq
                case 'cs'
                    A_title_txt{1} = ['$$', A_txt, ...
                        '_{', ...
                        num2str(thisAltGait.ithStance.cs), ...
                        '\leftrightarrow ', ...
                        num2str(thisAltGait.jthStance.cs), ...
                        '}', parallTxt, '^{x}$$']; 
                    A_title_txt{2} = ['$$', A_txt, ...
                        '_{', ...
                        num2str(thisAltGait.ithStance.cs), ...
                        '\leftrightarrow ', ...
                        num2str(thisAltGait.jthStance.cs), ...
                        '}', parallTxt, '^{y}$$'];
                    A_title_txt{3} = ['$$', A_txt, ...
                        '_{', ...
                        num2str(thisAltGait.ithStance.cs), ...
                        '\leftrightarrow ', ...
                        num2str(thisAltGait.jthStance.cs), ...
                        '}', parallTxt, '^{\theta}$$'];
                    xTxt = ['$$\alpha_{' 
                        num2str(thisAltGait.ithStance.cs) '}$$'];
                    yTxt = ['$$\alpha_{' 
                        num2str(thisAltGait.jthStance.cs) '}$$'];
                case '12'
                    A_title_txt{1} = ['$$', A_txt, ...
                        '_{1 \leftrightarrow 2}', parallTxt, ...
                        '^{x}$$']; 
                    A_title_txt{2} = ['$$', A_txt, ...
                        '_{1 \leftrightarrow 2}', parallTxt, ...
                        '^{y}$$'];
                    A_title_txt{3} = ['$$', A_txt, ...
                        '_{1 \leftrightarrow 2}', parallTxt, ...
                        '^{\theta}$$'];
                    xTxt = '$$t_{1}$$';
                    yTxt = '$$t_{2}$$';
            end
            % ... plot
            f = figure('units', 'pixels', 'position', [0 0 450 1200], ...
                'Color','w'); set(f,'Visible','on');
            tiledlayout(3, 1, 'TileSpacing', 'tight', 'Padding', 'tight');
            for i = 1:3
                % ... ... unpack further
                dzInow=reshape(dzI(:, :, i), [size(dzI, 1), size(dzI, 2)]);
                dzJnow=reshape(dzJ(:, :, i), [size(dzJ, 1), size(dzJ, 2)]);
                % ... plot
                ax = nexttile();
                quiver(ax, ...
                    aI(idxQ,idxQ), aJ(idxQ,idxQ), ...
                    dzInow(idxQ,idxQ), dzJnow(idxQ,idxQ),...
                'LineWidth', lW_Vector, 'Color', 'k');
                axis equal tight; hold on;
                if i == 1
                    xlabel(ax, xTxt, "FontSize", fS);
                    ylabel(ax, yTxt, "FontSize", fS);
                end
                ax.XColor = thisAltGait.ithStance.p_info.gc_col; 
                ax.YColor = thisAltGait.jthStance.p_info.gc_col;
                % set(get(ax,'YLabel'), 'rotation', 0, ...
                    % 'VerticalAlignment', 'middle');
                title(ax, A_title_txt{i}, 'Color', 'k', FontSize=fS);
                ax.XAxis.FontSize = fS; ax.YAxis.FontSize = fS; 
                xlim(aIlimits); ylim(aJlimits);
            end
        end

        % plot the stance space curvature/lie-bracket
        function plotStanceSpaceCurvature(thisAltGait, titleReq, numReq)
            aI = thisAltGait.stanceSpace.A{1}; 
            aJ = thisAltGait.stanceSpace.A{2}; dnum = size(aI, 1);
            aIlimits = [ min(aI, [], "all"), max(aI, [], "all") ]; 
            aJlimits = [ min(aJ, [], "all"), max(aJ, [], "all") ];
            lbdz = thisAltGait.stanceSpace.LBDZ;
            colLimits = [min(lbdz(:, :, 1:2), [], 'all'), ...
                                        max(lbdz(:, :, 1:2), [], 'all')];
            cfLvl = dnum; % number of color lvls
            CUB = thisAltGait.ithStance.p_info.CUB;
            fS = 25;
            lb_title_txt = cell(1, 2);
            % ... 
            % ... legacy stuff, plots 13 and 24 for the panel
            % ... components; commented out for now.
            % ... 
            % lb_title_txt{1} = ...
            %     ['$$[\vec{dz}_{' ...
            %     num2str(thisAltGait.ithStance.cs) ...
            %     '}, \vec{dz}_{' ...
            %     num2str(thisAltGait.jthStance.cs) '}]^{x}$$']; 
            % lb_title_txt{2} = ...
            %     ['$$[\vec{dz}_{' ...
            %     num2str(thisAltGait.ithStance.cs) ...
            %     '}, \vec{dz}_{' ...
            %     num2str(thisAltGait.jthStance.cs) '}]^{y}$$'];
            switch numReq
                case 'cs'
                    numStrI = num2str(thisAltGait.ithStance.cs);
                    numStrJ = num2str(thisAltGait.jthStance.cs);
                    xTxt = ['$$\alpha_{' 
                        num2str(thisAltGait.ithStance.cs) '}$$'];
                    yTxt = ['$$\alpha_{' 
                        num2str(thisAltGait.jthStance.cs) '}$$'];
                case '12'
                    numStrI = num2str(1); numStrJ = num2str(2);
                    xTxt = '$$t_{1}$$';
                    yTxt = '$$t_{2}$$';
            end
            switch titleReq
                case 'panel'
                    lb_txt = ['[\vec{dz}_{' numStrI '}, ' ...
                               '\vec{dz}_{' numStrJ '}]'];
                case 'connProj'
                    lb_txt = ['[\vec{A}_{' numStrI '}^{\parallel}, ' ...
                               '\vec{A}_{' numStrJ '}^{\parallel}]'];
            end
            lb_title_txt{1} = ['$$', lb_txt, '^{x}$$']; 
            lb_title_txt{2} = ['$$', lb_txt, '^{y}$$']; 
            % ... plot
            f = figure('units', 'pixels', 'position', [0 0 600 900], ...
                'Color','w'); set(f,'Visible','on');
            tiledlayout(2, 1, 'TileSpacing', 'tight', 'Padding', 'tight');
            for i = 1:2
                lbdzNow=reshape(lbdz(:,:,i),[size(lbdz,1),size(lbdz,2)]);
                ax = nexttile();
                contourf(ax, aI, aJ, lbdzNow, cfLvl, 'LineStyle', 'none');
                axis equal tight; hold on; colormap(CUB);
                if i == 1
                    xlabel(ax, xTxt, "FontSize", fS);
                    ylabel(ax, yTxt, "FontSize", fS);
                    ax.XColor = thisAltGait.ithStance.p_info.gc_col; 
                    ax.YColor = thisAltGait.jthStance.p_info.gc_col;
                end
                % set(get(ax,'YLabel'), 'rotation', 0, ...
                %                             'VerticalAlignment', 'middle');
                title(ax, lb_title_txt{i}, 'Color', 'k', FontSize=fS);
                ax.XAxis.FontSize = fS; ax.YAxis.FontSize = fS; 
                xlim(aIlimits); ylim(aJlimits); clim(colLimits);
                if i == 2
                    cb = colorbar(ax, 'FontSize', fS, ...
                        'TickLabelInterpreter', 'latex');
                    cb.Layout.Tile = 'east';
                end
            end
        end

        % plot the stance space curvature/lie-bracket
        function plotStanceSpaceInterpanelDotProduct(thisAltGait, titleReq, ...
                numReq)
            aI = thisAltGait.stanceSpace.A{1}; 
            aJ = thisAltGait.stanceSpace.A{2}; dnum = size(aI, 1);
            aIlimits = [ min(aI, [], "all"), max(aI, [], "all") ]; 
            aJlimits = [ min(aJ, [], "all"), max(aJ, [], "all") ];
            dzIdotJ = thisAltGait.stanceSpace.dzIdotJ;
            scatterLoc = thisAltGait.stanceSpace.a_dzThNull;
            cfLvl = dnum; 
            lW_contour = thisAltGait.ithStance.p_info.lW_contour;
            CUB = thisAltGait.ithStance.p_info.CUB;
            fS = 25;
            switch titleReq
                case 'panel'
                    dotTxt = '\hat{dz}';
                    parallTxt = '';
                case 'connProj'
                    dotTxt = '\vec{A}';
                    parallTxt = '^{\parallel}';
            end
            switch numReq
                case 'cs'
                    dot_title_txt = ['$$', dotTxt, parallTxt, '_{' ...
                        num2str(thisAltGait.ithStance.cs), ...
                        '} \cdot ', dotTxt, parallTxt, '_{', ...
                        num2str(thisAltGait.jthStance.cs), ...
                        '}$$']; 
                    xTxt = ['$$\alpha_{', ... 
                        num2str(thisAltGait.ithStance.cs), '}$$'];
                    yTxt = ['$$\alpha_{', ... 
                        num2str(thisAltGait.jthStance.cs), '}$$'];
                case '12'
                    dot_title_txt = ['$$', dotTxt, parallTxt, ...
                        '_{1} \cdot ', ...
                        dotTxt, parallTxt, '_{2}$$']; 
                    xTxt = '$$t_{1}$$';
                    yTxt = '$$t_{2}$$';
            end
            f = figure('units', 'pixels', 'position', [0 0 600 600], ...
                'Color','w'); set(f,'Visible','on'); ax = gca;
            contourf(ax, aI, aJ, dzIdotJ, cfLvl, 'LineStyle', 'none');
            axis equal tight; hold on; colormap(CUB);
            thresh_value = 0.99; % 0.95 % 0.99 % 0.999
            contour(ax, aI, aJ, dzIdotJ, thresh_value*ones(1, 2),...
                'k--','LineWidth',lW_contour+1); % , 'ShowText', 'on'
            contour(ax, aI, aJ, dzIdotJ, -thresh_value*ones(1, 2),...
                'k--','LineWidth',lW_contour+1); % , 'ShowText', 'on'
            scatter(ax, scatterLoc(1), scatterLoc(2), 100, "+",...
                'MarkerEdgeColor','k', 'LineWidth', lW_contour+1);
            xlabel(ax, xTxt, "FontSize", fS);
            ylabel(ax, yTxt, "FontSize", fS);
            ax.XColor = thisAltGait.ithStance.p_info.gc_col; 
            ax.YColor = thisAltGait.jthStance.p_info.gc_col;
            title(ax, dot_title_txt, 'Color', 'k', FontSize=fS);
            ax.XAxis.FontSize = fS; ax.YAxis.FontSize = fS; 
            xlim(aIlimits); ylim(aJlimits); clim([-1, 1]);
            colorbar(ax, 'FontSize', fS, 'TickLabelInterpreter', 'latex', ...
                                                    'Ticks', [-1, 0, 1]);
        end

    end  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

%% AUXILIARY FUNCTIONS-- can't be accessed outside this class

% this functions extracts the properties of a subgait's configuration
% trajectory for use by the static methods defined above
% ... "CT" : full configuration trajectory
% ... "cts": configuration trajectory for one of the subgaits
function CT = dzAndGcircForConfigTrjectory(CT, cts, fieldName, dnum)
    % check if the fields already exist, and append data accordingly
    tIdx = 1; rIdx = 1:2; vIdx = 1:3; % init column indices
    iIdx = 1;
    if isfield(CT.(fieldName), 't') % if already exist, append
        tIdx=tIdx+tIdx(end); rIdx=rIdx+rIdx(end); vIdx=vIdx+vIdx(end);
        iIdx = iIdx+1;
    end
    % iterate the extract the data
    switch cts.parameters.disc
        case 1
            CT.(fieldName).t(:, tIdx) = repmat(cts.(fieldName).t, dnum, 1); 
            CT.(fieldName).r(:, rIdx) = repmat(cts.(fieldName).r, dnum, 1);
            CT.(fieldName).dz(:, vIdx)=repmat(cts.(fieldName).dz, dnum, 1);
            CT.(fieldName).gCirc{1}(:, vIdx) =repmat(cts.(fieldName).gCirc, ...
                                                                dnum, 1);
        otherwise
            CT.(fieldName).t(:,tIdx)=interp1(...
                                        (1:numel(cts.(fieldName).t))', ...
                                            cts.(fieldName).t,... main
                                            linspace(1, ... query
                                        numel(cts.(fieldName).t), dnum)', ...
                                                    "linear");
            switch cts.parameters.disc
                case {2, 3}
                    interpMethod = 'linear';
                otherwise
                    interpMethod = 'pchip';
            end 
            CT.(fieldName).r(:, rIdx) = interp1(cts.complete.t, ...
                                            cts.complete.r,...
                                            CT.(fieldName).t(:, iIdx), ...
                                                    interpMethod); 
            CT.(fieldName).dz(:, vIdx) = interp1(cts.complete.t, ...
                                            cts.complete.dz, ...
                                            CT.(fieldName).t(:, iIdx), ...
                                                    interpMethod);
            CT.(fieldName).gCirc{1}(:, vIdx) = interp1(cts.complete.t, ...
                                                cts.complete.gCirc,...
                                                CT.(fieldName).t(:, iIdx), ...
                                                            interpMethod); 
    end
end

% exponentiate SE(2) body velocity (in se(2)) and obtain
% body displacement and trajectory
function [g, z] = exponentiateBodyVelocities(gCirc)
    g = exponentiateLieAlgebraElement(gCirc); z = g(end, :);
end

% convert the startified panels into meshgrids and obtain their lie-bracket
function [dzI, dzJ, dzIJ] = stratifiedGridAndLieBracket(dzI, dzJ)

    % divide into components
    dzIx = dzI(:, 1); dzIy = dzI(:, 2); dzIth = dzI(:, 3);
    dzJx = dzJ(:, 1); dzJy = dzJ(:, 2); dzJth = dzJ(:, 3);
    [dzIx,  dzJx]  = meshgrid(dzIx,  dzJx);
    [dzIy,  dzJy]  = meshgrid(dzIy,  dzJy);
    [dzIth, dzJth] = meshgrid(dzIth, dzJth);
    numRows = size(dzIx, 1); numCols = size(dzIx, 2); % get sizes
    dzI = nan([numRows, numCols, 3]); dzJ = dzI; % reinit dzI and dzJ
    dzI(:, :, 1) = dzIx; dzI(:, :, 2) = dzIy; dzI(:, :, 3) = dzIth;
    dzJ(:, :, 1) = dzJx; dzJ(:, :, 2) = dzJy; dzJ(:, :, 3) = dzJth;

    % obtain the lie-bracket manually
    dzIJ = nan(size(dzI));
    dzIJ(:, :, 1) = dzI(:, :, 2).*dzJ(:, :, 3) -dzJ(:, :, 2).*dzI(:, :, 3);
    dzIJ(:, :, 2) = dzJ(:, :, 1).*dzI(:, :, 3) -dzI(:, :, 1).*dzJ(:, :, 3);
    dzIJ(:, :, 3) = zeros([numRows, numCols, 1]); % always zero
    
end