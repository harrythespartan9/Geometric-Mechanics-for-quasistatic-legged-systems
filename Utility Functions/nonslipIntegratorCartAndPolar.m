function ...
    [a0, ... % adjusted initial condition
     tOut, alphaOut, tMax, ... % integrated results
     eventInfo, isValid, ... % more integration stopping event info
     dzOut, alphaVel, alphaAccln] = ... % stratified panels, vel, and accln 
            nonslipIntegratorCartAndPolar...
            (aa, ll, ... % quadrupedal morphological parameters
             nt, ... % number of timesteps to provide
             aInf, ... % the alpha value at inf({F})
             aSup, ... % the alpha value at sup({F})
             aB, ... % fixed body shape variable
             a0, ... % initial condition of the path
             dzF, ... % strafied panel fxn
             nonslipUnitVecF, ... % unit vector field fxn to flow along
             intTime, ... % minimum viable nonslip path length
             eventArgStruct) % struct with details for termination events
    %NONSLIPINTEGRATORCARTANDPOLAR this function flows along the nonslip
    %direction to generate nonslip trajectories in both cartesian and
    %singularity relative polar coordinates

    % obtain the initial conditions for our integration
    % ... ode states: [alphaI, alphaJ, rInf, gammaInf, rSup, gammaSup]
    % ... r and gamma are the phasors to current shape trajectory point
    % ... from the F-singular locations in the shape space
    infPhasor = a0 - aInf; 
    rInf = norm(infPhasor); gammaInf = atan2(infPhasor(2), infPhasor(1));
    supPhasor = a0 - aSup;
    rSup = norm(supPhasor); gammaSup = atan2(supPhasor(2), supPhasor(1));
    y0 = [a0, rInf, gammaInf, rSup, gammaSup]; % IC for ode integration
    eventArgStruct.parameters.initPhase = ...
                    [gammaInf, gammaSup]; % initial phase for ODE events

    % setup the options for the ODE integrator to stop upon hitting the
    % shape subspace limits or when a self-connection happens because the
    % level-sets are diffeomorphic to S1
    odeOptions = odeset(  ...
                    'Events', ...
                    @(t, y) nonslipShapeCoordsEventsLocal...
                                            (t, y, eventArgStruct) ...
                            );

    % integrate forwards and backwards until in each case we hit the limits
    % of the allowable shape space or we connect on ourselves
    intDirns = [-1, 1]; numDirns = numel(intDirns); % init directions
    t = cell(1, numDirns); y = t; % init outputs
    te = t; ye = t; ie = t; 
    tMax = nan(1, 2); % init max integration time in each direction
    for i = 1:numDirns
        [t{i}, y{i}, te{i}, ye{i}, ie{i}] = ...
                ode89( @(t,y) ...
                        computeNonslipVelocityAugmented...
                        (t, y, nonslipUnitVecF, intDirns(i), aa, ll, aB), ...
                                [0, 1e2], y0, odeOptions ...
                      ); % solve for outputs
        tMax(i) = te{i}; % extract tMax
    end
    
    % check if the trajectory is valid
    % ... based on comparing the max integration time "intTime" for a
    % ... single subgait/nonslip path and the max integration time before
    % ... an ill-conditioned path
    % ... even if a path is valid, both sides might not be valid as one
    % ... side could have a smaller subset of solution, so adjust the
    % ... initial condition in that case
    eventInfo = cell(1, numDirns); 
    tDiff = 0; % set the reference point adjustment time to zero as default
    switch sum(tMax) >= sum(intTime)
        case 1 % valid level-set
            isValid = true;
            for i = 1:numDirns
                eventInfo{i} = '';
            end
            if ~all(tMax >= intTime) 
                % ... one side is valid, but the other side is not
                % ... needs adjustment to the initial condition - done
                % ... through the time offset variable "tDiff"
                if tMax(1) < intTime(1)
                    tDiff = intTime(1) - tMax(1);
                elseif tMax(2) < intTime(2)
                    tDiff = -(intTime(2) - tMax(2));
                end
            end
        case 0 % invalid level-set
            isValid = false;
            for i = 1:numDirns
                switch ie{i}
                    case 1
                        eventInfo{i} = 'shape-bounds violation';
                    case 2
                        eventInfo{i} = 'self-connected sets violation';
                end
            end
    end
    
    % interpolate the solutions
    % ... concatenate the two solutions
    % ... offset the time accordingly and then get the new reference point
    % ... returned as well
    for i = 1:numDirns
        switch intDirns(i)
            case -1
                tFull = flipud(-t{i}); yFull = flipud(y{i});
                tFull = tFull(1:end-1); yFull = yFull(1:end-1, 1:2);
            case +1
                tFull = [tFull; t{i}]; yFull = [yFull; y{i}(:, 1:2)];
        end
    end
    tOut = linspace( tFull(1), tFull(end), nt )'; 
    alphaOut = interp1( tFull, yFull, tOut, "pchip" ); % nonslip trajectory
    tOut = tOut - tDiff; % offset time if set
    dzOut = dzF( aa, ll, aB, alphaOut(:, 1), alphaOut(:, 2) ); % panels
    alphaVel = diff(alphaOut)./...
                        repmat(diff(tOut), 1, 2); % vel (nt-1 x 1)
    alphaAccln = diff(alphaVel)./...
                    repmat(diff(tOut(1:end-1)), 1, 2); % accln (nt-1 x 1)
end

%% AUXILIARY FUNCTIONS

function [value, isTerminal, direction] = ...
    nonslipShapeCoordsEventsLocal(~, y, argStruct)
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
    
    % unpack the argument structure
    % ... parameters include arguments to evaluate functions
    % ... bounds include 1D or 2D closed intervals  to define the allowable
    % ... range for the ode integration
    % ... functions we need to evaluate
    argParameters = argStruct.parameters;
    argBounds = argStruct.bounds;
    eventList = argStruct.events;

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
                    alphaBoundaryCheck = isOnBoundsWithPrecision(y(1:2),...
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
                delPhase = abs([y(4), y(6)] - argParameters.initPhase);
                % EVENT value
                % check if the net change in phase is smaller than the set
                % threshold by taking the difference
                valueNow = min( phaseLimit - delPhase );
        end
        % Assign the current EVENT details
        value(i)      = valueNow;
        direction(i)  = directionNow;
    end
end

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

% function to compute the velocity along the flow
% ... corresponding to shape vars, we flow along the "nonslipUnitVecF"
% ... for the phasor dynamics, we compute the phasor velocity
% ... described in polar coordinates
% ... the first argument is current time which is ignored
% ... last three terms are parameters needed to compute the velocity
function yDot = computeNonslipVelocityAugmented(~, y, X, dirn, a, l, ab)
    % extract the different terms
    alpha_i = y(1); alpha_j = y(2);
    rInf = y(3); gammaInf = y(4);
    rSup = y(5); gammaSup = y(6);
    % compute the nonslip shape trajectory velocity
    alphaDot = dirn*X(a, l, ab, alpha_i, alpha_j);
    % compute the phasor velocity wrt to the inf(F)
    phasorInfDot = transpose( generate2x2RotMat(gammaInf) )*alphaDot;
    rInfDot = phasorInfDot(1);
    gammaInfDot = phasorInfDot(2)/rInf;
    % ... dividing by 'r' to convert the linear $\hat{\theta}$ velocity in 
    % ... the polar coordinates into the rotational velocity
    % compute the phasor velocity wrt to the sup(F)
    phasorSupDot = transpose( generate2x2RotMat(gammaSup) )*alphaDot;
    rSupDot = phasorSupDot(1);
    gammaSupDot = phasorSupDot(2)/rSup;
    % extract the state velocities and return in the correct format
    yDot = [alphaDot; rInfDot; gammaInfDot; rSupDot; gammaSupDot];
end

% function to compute the SO(2) matrix associated with the polar
% description of the nonslip shape trajectories
function M = generate2x2RotMat( x )
    M = [cos(x), -sin(x);
         sin(x), +cos(x)];
end