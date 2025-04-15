function [t, y, z] = simulateSineSwingGaits(fA, params, mPhase)
    %SIMULATESINESWINGGAITS simulate sinusoidal swing trajectories
    %   For more details, refer to the following livescript:
    %   "se2_fixedCLARImobility_slipEstimates.mlx"
    
    % unpack the params
    dnum = params.dnum;
    a = params.na; l = params.nl; ab = params.nalpha_b; sHat = params.sHat;

    % obtain the phase offset for the sinusoidal shape trajectories
    % based on the current maneuverability phase
    % ... sometimes the phase offsets are directly prescribed, so code
    % ... below is setup to handle that
    switch numel(mPhase)
        case 1
            sP = computeSineShapePhaseV1(mPhase);
            % ... mPhase == 0.00: right-displacing trot gait
            % ... mPhase == 0.25: fwd  -displacing trot gait
            % ... mPhase == 0.50: left -displacing trot gait
            % ... mPhase == 0.75: bwd  -displacing trot gait
            % ... then it wraps back around to right-displacing gait
        case 4
            sP = mPhase;
            % ... 'mPhase' already in the phase coordinates for each leg
        otherwise
            error(['ERROR! "mPhase" arg should be a double array with ' ...
                '1 or 4 elements.']);
    end

    % simulate the system
    tau = linspace(0, 1, dnum); % a gait phase array; 1 => cycle complete.
    [t, y] = ode45( ... % function tha that provides the body velocity
        @(t, y) computeBodyVelocityV1(t, y, fA, sHat, sP, a, l, ab), ...
                tau, ... % time vector to obtain the output
                zeros(1, 3)); % body starts at the origin
    z = y(end, :); % return the last body position as the net displacement
end