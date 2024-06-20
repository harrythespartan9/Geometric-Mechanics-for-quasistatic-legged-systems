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
    
    % initialize the return value based on the number of EVENTs requested
    % and each output argument as a cell array of the same size
    eventNum = numel(eventList);
    value = cell(eventNum, 1);
    isTerminal = cell(eventNum, 1);
    direction = cell(eventNum, 1);
    
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
                numPtsClosedLoop = numel(argStruct.alphaBox.X) + 1;
                X = [argStruct.alphaBox.X(:)', argStruct.alphaBox.X(1)];
                Y = [argStruct.alphaBox.Y(:)', argStruct.alphaBox.Y(1)];
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
                valueNow = ... % obtain the value for EVENT
                    mul*computeDistToPolyBounds(X, Y, minDistPolyLoc);
                if valueNow >= 0, isTerminalNow = 1; end % terminate int.

            case 'F_bounds'

            case 'phase_bounds'

        end
        % Assign the current event details
        value{i}      = valueNow;
        isTerminal{i} = isTerminalNow;
        direction{i}  = directionNow;
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