function ...
    [alpha0, ... % initial condition
     t, alpha, tMax, ... % integrated results ('alpha0' is found at t == 0)
     eventInfo, isValid] = ... % more integration stopping event info
            nonslipIntegratorCartAndPolar...
                    (aa, ll, ... % quadrupedal morphological parameters
                     aInf, ... % the singularity producing inf({F})
                     aSup, ... % the singularity producing sup({F})
                     alphaB, ... % fixed body shape variable
                     alpha0, ... % initial condition of the path
                     nonslipUnitVecF, ... % unit vector field to flow along
                     intTime, ... % forward and backward integration times
                     eventArgStruct) % struct with details for termination events
    %NONSLIPINTEGRATORCARTANDPOLAR this function flows along the nonslip
    %direction to generate nonslip trajectories in both cartesian and
    %singularity relative polar coordinates

end

%% AUXILIARY FUNCTIONS

function [value, isTerminal, direction] = ...
    nonslipShapeCoordsEventsLocal(~, y, argStruct, eventList)
%NONSLIPSHAPECOORDSEVENTS this function generates terminal conditions for
%the ODE integrator when one of the following conditions are satisfied: 1.
%if the accessible shape bounds are violated. 2: if the trajectory connects
%on itself. 
%   Working procedure: Checks if ODE terminating EVENTs are true based on 
%   the provided 'eventList' (and a defined case in the execution below).
%   Refer to the comments below for more information on the algorithm for a
%   specific termination event.

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
                % ... get the shape space limits
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
                yCol = y(1:2); yCol = yCol(:);
                valueNow = mul*min(abs(alphaLimits-repmat(yCol, 1, 2)),...
                                    [], "all");

            case 'phase_bounds'
                % initialize the direction
                % ... the direction for this EVENT is decreasing as the set
                % ... starts to connect on itself
                directionNow = -1;
                % unpack and init
                % ... get the initial phase, current phase, and obtain the
                % ... absolute value of net change in phase
                % ... unpack the phase limit
                phaseLimit = argBounds.phaseLimits;
                phase0 = argParameters.initPhase;
                phaset = [y(4), y(6)]; delPhase = abs(phaset - phase0);
                % EVENT value
                % check if the net change in phase is smaller than the set
                % threshold by taking the difference
                valueNow = phaseLimit - delPhase;
        end

        % Assign the current EVENT details
        value(i)      = valueNow;
        direction(i)  = directionNow;
    end

end