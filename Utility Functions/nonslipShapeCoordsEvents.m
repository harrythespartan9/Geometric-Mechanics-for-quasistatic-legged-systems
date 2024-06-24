function [value, isTerminal, direction] = nonslipShapeCoordsEvents(~, y,...
                                                    argStruct, eventList)
%NONSLIPSHAPECOORDSEVENTS this function generates hybrid EVENTs based on
%nonslip coordinates in the two-beat contact state of a pin-constrained
%quadrupedal robot
%   This function checks if ode terminating EVENTs are true based on the
%   provided 'eventList'. If a new EVENT is needed, it should be first
%   defined within the switch case statement below.

    % static decimal precision below for checking if a point is on the
    % boundary
    % ... static because higher values don't work from trials and lower
    % ... values are too imprecise
    decimalPrecision = 5;
    
    % By default, just return nonterminal conditions if not EVENT is
    % requested
    if nargin < 4
        value = 1; isTerminal = 1; direction = 0; return
    end
    
    % unpack the argument structure
    % ... parameters include arguments to evaluate functions
    % ... bounds include 1D or 2D closed intervals  to define the allowable
    % ... range for the ode integration
    % ... functions we need to evaluate
    argParameters = argStruct.parameters;
    argBounds = argStruct.bounds;
    argFunctions = argStruct.functions;

    % initialize the return value based on the number of EVENTs requested
    % and each output argument as a cell array of the same size
    eventNum = numel(eventList);
    value = nan(eventNum, 1); 
    isTerminal = ones(eventNum, 1); % terminate when EVENT by default
    direction = nan(eventNum, 1);
    
    % EVENTs are requested and thus we iterate over the EVENT list cell 
    % array
    for i = 1:eventNum
        switch eventList{i}

            case 'shape_bounds'
                % initialize the direction
                % ... the direction for this EVENT is increasing as you
                % ... leave the accessible shape space bounds
                directionNow = +1;
                % unpack and init
                % ... get the shape space limits and check if each
                % ... component is within the bounds in the corresponding
                % ... direction
                alphaLimits = argBounds.alphaLimits;
                % check if the query point 'y' is inside the accessible
                % shape space
                % ... we do this by simply comparing each component of 'y'
                % ... with componentwise bounds
                % ... do approximate 'onBounds' check only if the inside
                % ... check returns false
                alphaInsideCheck = (y(1) > alphaLimits(1, 1) ...
                                            && y(1) < alphaLimits(1, 2))...
                                && (y(2) > alphaLimits(2, 1) ...
                                            && y(2) < alphaLimits(2, 2));
                if ~alphaInsideCheck
                    alphaBoundaryCheck = isOnBoundsWithPrecision(y,...
                                                        alphaLimits,...
                                                        decimalPrecision);
                end
                % get the multipler for EVENT value
                if alphaInsideCheck
                    mul = -1;
                elseif alphaBoundaryCheck
                    mul = 0;
                else
                    mul = +1;
                end
                % find the minimum distance between y and the boundary of 
                % the polygon-- EVENT value
                % ... this is akin to finding the minimum distance to
                % ... componentwise bounds
                valueNow = mul*min(abs(alphaLimits-repmat(y(:),1,2)),...
                                    [], "all");

            case 'F_bounds'
                % initialize increasing direction
                directionNow = +1;
                % unpack and init
                a = argParameters.a;
                l = argParameters.l;
                F_bounds = argBounds.F_bounds;
                F_fxn = argFunctions.F_fxn;
                % check the current value of F
                F_now = F_fxn(a, l, y(1), y(2));
                % obtain the EVENT value
                % ... this is defined as the minimum distance to the bounds
                % ... of the scalar field F
                % ... this distance is further multiplied by +1 if outside
                % ... the bounds, 0 for on the bounds, and -1 for inside
                % ... the bounds
                if F_now > F_bounds(1) && F_now < F_bounds(2)
                    mul = -1;
                elseif isOnBoundsWithPrecision(F_now, F_bounds,...
                                                        decimalPrecision)
                    mul = 0;
                else
                    mul = +1; % current multiplier
                end
                valueNow = mul*min(abs(F_bounds - F_now), [], "all");

            case 'phase_bounds'
                % initialize the direction
                % ... this will be decreasing because the angle bounds are
                % ... very small, and EVENT value will be defined from
                % ... within the centroid of the angle bounds
                directionNow = -1;
                % unpack and init
                angleThreshold = argBounds.angleThreshold;
                yRefPhase = argParameters.yRefPhase;
                yExtremal = argParameters.yExtremal';
                % compute the EVENT value
                % ... create SO(2) matrix for the self-connecting level-set
                % ... phase by adding pi to 'yRefPhase'
                % ... compute the SO(2) matrix to go from the previous
                % ... matrix to the current 'y' phase
                % ... computed as: ySO2*ySelfConnectedSO2'
                ySelfConnectedSO2 = rot_SE2(pi)*rot_SE2(yRefPhase);
                yNow = (y - yExtremal)/norm(y - yExtremal); 
                yPhaseNow = atan2(yNow(2), yNow(1));
                ySO2 = rot_SE2(yPhaseNow);
                yDiffSO2 = ySO2*ySelfConnectedSO2';
                diffNow = atan2(  (-yDiffSO2(1, 2)+yDiffSO2(2, 1))*0.5, ...
                                  ( yDiffSO2(1, 1)+yDiffSO2(2, 2))*0.5   );
                if abs(diffNow) < angleThreshold % inside self-connection 
                                                 % threshold
                    mul = -1;
                elseif isOnBoundsWithPrecision...% is on the threshold
                        (diffNow, angleThreshold*[-1, 1], decimalPrecision)
                    mul = 0;
                else                             % allowable space
                    mul = +1;
                end
                valueNow = mul*min(abs(angleThreshold*[-1, 1] - diffNow));

        end

        % Assign the current EVENT details
        value(i)      = valueNow;
        direction(i)  = directionNow;
    end


end

%% AUXILIARY FUNCTIONS

% compute if the position 'y' is close enough to the boudary
% ... specifically useful when we can't achieve perfect equality with the
% ... bounds especially working with floating-point precision,
% ... trigonometric functions, and other special constants
function flag = isOnBoundsWithPrecision(y, yBounds, decimalPrecision)
    % unset the flag by default
    flag = false;
    % iterate over each component and compute if they are within the
    % reasonable bounds
    for i = 1:numel(y)
        yComponentNow = y(i);
        for j = 1:size(yBounds, 2)
            yBoundNow = yBounds(i, j);
            yBoundThesholdNow = sort( yBoundNow*(ones(1, 2) + ...
                                    (10*[-1 +1]).^-decimalPrecision) );
            % check if the component is within a precision ball of the 
            % current boundary
            yBoundaryCheck = ...
                yComponentNow >= yBoundThesholdNow(1) &&...
                yComponentNow <= yBoundThesholdNow(2);
            if yBoundaryCheck
                flag = true; return
            end
        end
    end
end

% compute the bounds from points of interest provide as an array by taking
% the minimum and maximum value
function boundsArr = computeBounds(inArr)
    boundsArr = [min(inArr, [], "all"), max(inArr, [], "all")];
end