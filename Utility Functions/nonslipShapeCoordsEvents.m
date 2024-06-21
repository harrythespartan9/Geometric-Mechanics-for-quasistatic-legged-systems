function [value, isTerminal, direction] = nonslipShapeCoordsEvents(~, y,...
                                                    argStruct, eventList)
%NONSLIPSHAPECOORDSEVENTS this function generates hybrid EVENTs based on
%nonslip coordinates in the two-beat contact state of a pin-constrained
%quadrupedal robot
%   This function checks if ode terminating EVENTs are true based on the
%   provided 'eventList'. If a new EVENT is needed, it should be first
%   defined within the switch case statement below.
    
    % By default, just return nonterminal conditions if not EVENT is
    % requested
    if nargin < 4
        value = 1; isTerminal = 0; direction = 0; return
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
    value = nan(eventNum, 1); isTerminal = nan(eventNum, 1);
    direction = nan(eventNum, 1);
    
    % EVENTs are requested and thus we iterate over the EVENT list cell 
    % array
    for i = 1:eventNum
        switch eventList{i}

            case 'shape_bounds'
                % initialize the direction
                % ... the direction for this EVENT is increasing because
                % ... when the point is inside the shape-bounds, the
                % ... minimum distance takes a negative value, zero on the
                % ... boundary, and positive outside.
                directionNow = +1;
                % unpack and init
                % ... the number of points around a 4-gon is 5 because we
                % ... double count the first point for interpolation
                % ... we also
                numPtsClosedLoop = numel(argBounds.alphaBox.X) + 1;
                X = [argBounds.alphaBox.X(:)',...
                     argBounds.alphaBox.X(1)];
                Y = [argBounds.alphaBox.Y(:)',...
                     argBounds.alphaBox.Y(1)];
                % check if the query point 'y' is inside, on the boundary,
                % or outside the polygon
                % ... based on this value, setup an increasing multiplier
                % ... scale the 'minDist'
                [inFlag, onFlag] = inpolygon(y(1), y(2), X, Y);
                switch inFlag % if inside or on the boundary
                    case 1
                        if onFlag % if on the boundary
                            mul = 0;
                        else % if completely inside
                            mul = -1;
                        end
                    case 0
                        mul = 1;
                end
                % find the minimum distance between y and the boundary of 
                % the polygon
                % ... we set this up as a constrained minimization problem
                % ... and solve it using 'fmincon' function
                % ... the 'computeDistFromYtoPolyBounds' will evalute the
                % ... distance to 'y' for a chosen point
                % ... finally, we scale the point using the bounds
                % ... multiplier ('mul') we computed earlier
                minDistPolyLoc = ...
                    fmincon(@(idx) computeDistToPolyBounds(y, X, Y, idx), ...
                    ceil(numPtsClosedLoop/2),... % start in the middle
                    [], [], [], [],... % no equality or inequality constr.
                    0, numPtsClosedLoop); % upper and lower bounds
                valueNow = ... % EVENT value
                    mul*computeDistToPolyBounds(X, Y, minDistPolyLoc);

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
                elseif F_now == F_bounds(1) || F_now == F_bounds(2)
                    mul = 0;
                else
                    mul = +1; % current multiplier
                end
                valueNow = mul*min(norm(F_bounds - F_now)); % EVENT value

            case 'phase_bounds'
                % initialize the direction
                % ... this will be decreasing because the angle bounds are
                % ... very small, and EVENT value will be defined from
                % ... within the centroid of the angle bounds
                directionNow = -1;
                % unpack and init
                angleThreshold = argParameters.angleThreshold;
                yRefPhase = argParameters.yRefPhase;
                yExtremal = argParameters.yExtremal;
                % create the phase bounds
                % ... we first obtain the self-connection points for the
                % ... current level-set of F
                % ... obtain the phase bounds about this point 
                selfConnectingPhase = yRefPhase + pi;
                phaseBounds = selfConnectingPhase + angleThreshold*[-1, 1];
                cosBounds = [cos(phaseBounds),... % pts of interest
                                    cos(selfConnectingPhase)];
                cosBounds = computeBounds(cosBounds); % bounds of these pts
                sinBounds = [sin(phaseBounds),... 
                                    sin(selfConnectingPhase)];
                sinBounds = computeBounds(sinBounds);
                % obtain the current phase
                % ... we first obtain the phase of the current position 'y'
                % ... from the centroid 'yExtremal' and check if the angle
                % ... of this phasor is within our bounds for self
                % ... connected sets of F
                phasorNow = y(:) - yExtremal(:)/norm(y(:) - yExtremal(:));
                phaseNow = atan2(phasorNow(2), phasorNow(1));
                cosPhase = cos(phaseNow); sinPhase = sin(phaseNow);
                % obtain the EVENT value
                % ... we first obtain the location of the current phase
                % ... to in turn obtain the value multiple
                % ... then we obtain the EVENT value by scaling the
                % ... difference to the closest boundary
                if (cosPhase > cosBounds(1) && cosPhase < cosBounds(2))... 
                && (sinPhase > sinBounds(1) && sinPhase < sinBounds(2))
                    % out-of-bounds/violation zone
                    mul = -1;
                elseif (cosPhase == cosBounds(1) && sinPhase == sinBounds(1))...
                    || (cosPhase == cosBounds(2) && sinPhase == sinBounds(2))
                    % on the bounds
                    mul = 0;
                else
                    % inside the bounds
                    mul = 1;
                end
                % distance of current phase to each bound
                dist2Bounds = min( ...
                              vecnorm([cosBounds-cosPhase;... x-distance
                                       sinBounds-sinPhase],... y-distnace
                                       2, 1)... % 2-norm, row-wise 
                                 );  % give the minimum distance
                valueNow = mul*min(dist2Bounds);

        end
        
        % terminate the integration if the EVENT value is zero
        if valueNow == 0, isTerminalNow = 1; else, isTerminalNow = 0; end

        % Assign the current EVENT details
        value(i)      = valueNow;
        isTerminal(i) = isTerminalNow;
        direction(i)  = directionNow;
    end


end

%% AUXILIARY FUNCTIONS

% interpolate along the corners of a polygon represented as a closed-loop
% set of coordinates in 'X' and 'Y'
function polyPt = obtainPolyBoundPt(X, Y, idx)
    polyPt = interp1(0:numel(X), [X; Y], idx, "linear"); % specified pt
end

% compute the distance from 'pt1' to another pt 'pt2'
function dist = computeDistBWtwoPoints(pt1, pt2)
    dist = norm(pt1(:) - pt2(:)); % 2-norm distance
end

% Compute the distance from 'pt' to a  point on the boundary of the polygon 
% specified by a set closed loop of corners in their cartesian coordinates.
function dist = computeDistToPolyBounds(pt, X, Y, idx)
    polyPt = obtainPolyBoundPt(X, Y, idx);
    dist = computeDistBWtwoPoints(pt, polyPt);
end

% compute the bounds from points of interest provide as an array by taking
% the minimum and maximum value
function boundsArr = computeBounds(inArr)
    boundsArr = [min(inArr, [], "all"), max(inArr, [], "all")];
end