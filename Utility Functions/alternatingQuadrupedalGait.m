classdef alternatingQuadrupedalGait
    %ALTERNATINGQUADRUPEDALGAIT this class is used for defining alternating
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

        function thisAltGait = alternatingQuadrupedalGait(stance1, stance2, ...
                                                leafModeArg, inputModeArg, ...
                                                integrationLimitsArg)
            %ALTERNATINGQUADRUPEDALGAIT Construct an instance of this class
            %   mostly we setup the properties here and make some back
            %   checks to see if tha "Path2_Mobility" instances are
            %   complementary, etc.
            
            % check if the stances 1 and 2 are complementary, if not return
            % and error
            if ~areComplementaryStances(stance1, stance2)
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
                        thisAltGait.inputMode = 'path_limit_compliant';
                        thisAltGait.integrationLimits = ...
                                                    integrationLimitsArg;
                    end
                otherwise
                    error(['ERROR! Other input descriptions are not ' ...
                        'supported.']);
            end
        end

    end%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
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
                            if nargin < 5
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
                        otherwise
                            error(['ERROR! Other input descriptions are not ' ...
                                'supported.']);
                    end
            end
        end

        function thisAltGait = simulateAndEstimateConfigTrajectory(...
                thisAltGait, inputModeArg, integrationLimitsArg)
        end

        function thisAltGait = simulateBodyTrajectory(thisAltGait, ...
                                    inputModeArg, integrationLimitsArg)
        end
        
        % this function computes the integration times for both subgaits
        % that form the "thisAltGait" instance
        function [tIC, tFC] = computeGaitIntegrationTimes...
                                            (thisAltGait, inputs, T, Toff,...
                                                                    Tmax)
            % ensure that the variables are the correct sizes
            if size(T, 1)
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
            [tICi, tFCi] = computeSubgaitIntegrationTimes...
                                    (thisAltGait, ...
                                    ithInput, ithIntgTimes, ithOffTime, ...
                                    ithIntgMAXTimes);
            [tICj, tFCj] = computeSubgaitIntegrationTimes...
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
                                    (thisAltGait, inputs, T, Toff, Tmax)
            % Compute initial and final condition times based on the
            % current input mode
            u1 = inputs(:, 1); u2 = inputs(:, 2);
            T0 = T(1); Tpi = T(2);
            switch thisAltGait.inputMode
                case 'std'
                    tIC = -u1*T0  + u2 + Toff;
                    tFC = +u1*Tpi + u2 + Toff;
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
                    tIC = -u1*T0  + capTerm.*u2 + Toff;
                    tFC = +u1*Tpi + capTerm.*u2 + Toff;
            end
        end

    end%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

