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
        
        integrationLimits   % forward and backward integration limits-- 
                            % this is only used when in "path_limit_compliant"

    end
    
    methods

        function thisAltGait = altQuadGait(stance1, stance2, ...
                                                leafModeArg, inputModeArg, ...
                                                integrationLimitsArg)
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
                    thisAltGait.integrationLimits = inf*[-1, 1];
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
            if strcmp(thisAltGait.inputMode, 'path_limit_compliant') ...
                                                && (nargin < 5)
                error(['ERROR! For the "path_limit_compliant" input ' ...
                       'method, we need the maximum forward and ' ...
                       'backward integration times to appropriately ' ...
                       'bound the sliding input contribution.']);
            end
            % unpack inputs, path integration times, and offset times
            % ... the inputs are setup as column vectors ordered from left 
            % ... to right as scaling, sliding for Si, and same for Sj 
            ithInput = inputs(:, 1:2); jthInput = inputs(:, 3:4);
            ithIntgTimes = T(1:2); jthIntgTimes = T(3:4);
            ithOffTime = Toff(1); jthOffTime = Toff(2);
            ithIntgMAXTimes = Tmax(1:2); jthIntgMAXTimes = Tmax(3:4);
            % obtain the integration times individually for each subgait
            [tICi, tFCi] = altQuadGait.computeSubgaitIntegrationTimes...
                                    (thisAltGait, ...
                                    ithInput, ithIntgTimes, ithOffTime, ...
                                    ithIntgMAXTimes);
            [tICj, tFCj] = altQuadGait.computeSubgaitIntegrationTimes...
                                    (thisAltGait, ...
                                    jthInput, jthIntgTimes, jthOffTime, ...
                                    jthIntgMAXTimes);
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
            u1 = inputs(:, 1); u2 = inputs(:, 2);
            T0 = T(1); Tpi = T(2);
            switch thisAltGait.inputMode
                case 'std'
                    tIC = -u1*T0  + u2;
                    tFC = +u1*Tpi + u2;
                case 'path_limit_compliant'
                    % check the quadrant of the input to compute the
                    % capping term associated with the sliding input
                    Tminus = Tmax(1); Tplus = Tmax(2); 
                    switch +u1*Tpi >= -u1*T0
                        case 1
                            switch u2 >= 0
                                case 1
                                    capTerm = Tplus - u1*Tpi;
                                case 0
                                    capTerm = -u1*T0 - Tminus;
                            end
                        case 0
                            switch u2 >= 0
                                case 1
                                    capTerm = Tplus + u1*T0;
                                case 0
                                    capTerm = u1*Tpi - Tminus;
                            end
                    end
                    % compute the initial and final condition integration
                    % times
                    tIC = -u1*T0  + capTerm.*u2;
                    tFC = +u1*Tpi + capTerm.*u2;
            end
        end

        % this function computes the full configuration trajectory when
        % provided with individual configuration trajectories during each
        % stance phase
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
                [cT.(fieldNow).gHat{1}, cT.(fieldNow).zHat{1}] = ...
                            altQuadGait.piecewiseBodyVelFlow(gC_i, gC_j);
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
                altQuadGait.plotBodyTrajectoryEstimates...
                                            (cT.discretized, zoomFlag);
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
                    gCHat = gC;
                    [gHat, zHat] = exponentiateBodyVelocities(gC);
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

        function plotBodyTrajectoryEstimates(discJointStancePath, zoomFlag)
        %PLOTALTGAITBODYTRAJECTORYESTIMATES plot the body trajectory of the system
        %when performing an alternating gait and its estimates using the BCH
        %formula
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
            % setups
            fS = 25; 
            appxPicewCol = [51,160,44]/255;
            appxCol = [31,120,180]/255; appxfA = 0.2*(1:4);
            gbCol = [0, 0, 0]/255;
            figure('Visible', 'on', 'Units', 'pixels', 'Position', [0 0 900 900]); ax = gca; box(ax, "on");
            % plot the body trajectory, represent the net displacement, and the body
            % orientation
            plot(ax, b(:, 1), b(:, 2), '-', 'LineWidth', 2.0, 'Color', gbCol, ...
                'DisplayName', 'simulated trajectory');
            grid(ax, "on"); hold(ax, "on"); set(ax,'TickLabelInterpreter','latex');
            p = scatter(ax, zb(1), zb(2), 100, 'o', "filled", "MarkerFaceColor", gbCol, 'MarkerFaceAlpha', 1.00);
            set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            p = quiver(ax, zb(1),  zb(2), -sin(zb(3)), cos(zb(3)),...
                'LineWidth', 3.0, 'Color', gbCol, 'LineStyle', '-',...
                'AutoScaleFactor', 0.1, 'MaxHeadSize', 1);
            set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            % plot the stancewise commutative estimate
            gHatb = discJointStancePath.gHat{1}; 
            zHatb = discJointStancePath.zHat{1}; % locally unpack
            plot(ax, gHatb(:, 1), gHatb(:, 2), '-', 'LineWidth', 2.0,...
                'Color', [ appxPicewCol, 1.0 ], ...
                    'DisplayName', 'stancewise commutative');
            p = scatter(ax, zHatb(1), zHatb(2), 100, 'o', "filled",...
                    "MarkerFaceColor", appxPicewCol, 'MarkerFaceAlpha', 1.0, ...
                    'MarkerEdgeColor', appxPicewCol, 'MarkerEdgeAlpha', 1.0);
            set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
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