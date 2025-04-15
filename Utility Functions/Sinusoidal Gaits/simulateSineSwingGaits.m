function [t, y, z] = simulateSineSwingGaits(fA, params, mPhase)
    %SIMULATESINESWINGGAITS simulate sinusoidal swing trajectories
    %   For more details, refer to the following livescript:
    %   "se2_fixedCLARImobility_slipEstimates.mlx"
    
    % unpack the params
    dnum = params.dnum;
    a = params.na; l = params.nl; ab = params.nalpha_b; sHat = params.sHat;

    % obtain the swing phase offset based on the current maneuverability
    % phase
    sP = computeSwingPhase(mPhase);
    % ... mPhase == 0.00: right-displacing trot gait
    % ... mPhase == 0.25: fwd  -displacing trot gait
    % ... mPhase == 0.50: left -displacing trot gait
    % ... mPhase == 0.75: bwd  -displacing trot gait
    % ... then it wraps back around to right-displacing gait

    % simulate the system
    tau = linspace(0, 1, dnum); % a gait phase array; 1 => cycle complete.
    [t, y] = ode45( ... % function tha that provides the body velocity
        @(t, y) computeBodyVelocity(t, y, fA, sHat, sP, a, l, ab), ...
                tau, ... % time vector to obtain the output
                zeros(1, 3)); % body starts at the origin
    z = y(end, :); % return the last body position as the net displacement
end

%% AUXILIARY FUNCTIONS

% Here, we defined a couple of functions to generate the limb swing angle
% and angular rate trajectories to simulate the system. We also define the
% necessary phase modulations to sweep out the system trajectory.
% ... note that the current setup only supports Trot gaits.
function [alpha_t, alphaDot_t] = computeSwingShapeTrajectory(t, sHat, sP)
    alpha_t = sHat*[-cos(2*pi*t + sP(1));
                    +cos(2*pi*t + sP(2));
                    +cos(2*pi*t + sP(3));
                    -cos(2*pi*t + sP(4))];
    alphaDot_t = 2*pi*sHat*[+sin(2*pi*t + sP(1));
                            -sin(2*pi*t + sP(2));
                            -sin(2*pi*t + sP(3));
                            +sin(2*pi*t + sP(4))];
end
% 
function sPhase = computeSwingPhase(mPhase)
    if mPhase >= 0 && mPhase < 0.25
        intPhaseOff = 0;
        initVals = zeros(4, 1);
        rampVec = [1, 0, 1, 0]';
    elseif mPhase >= 0.25 && mPhase < 0.50
        intPhaseOff = 0.25; 
        initVals = [pi, 0, pi, 0]';
        rampVec = [0, 1, 0, 1]';
    elseif mPhase >= 0.50 && mPhase < 0.75
        intPhaseOff = 0.50;
        initVals = pi*ones(4, 1);
        rampVec = [-1, 0, -1, 0]';
    else
        intPhaseOff = 0.75; 
        initVals = [0, pi, 0, pi]';
        rampVec = [0, -1, 0, -1]';
    end
    rampVal = 4*pi*(mPhase-intPhaseOff);
    sPhase = initVals + rampVal*rampVec;
end

% compute body velocity at current gait phase, tau
function gDot = computeBodyVelocity...
                                        (t, y, fA, sHat, sP, a, l, ab)

    % obtain the current swing shape trajctory
    [a_t, aDot_t] = computeSwingShapeTrajectory(t, sHat, sP);

    % obtain the body velocity in the rest frame
    switch t < 0.5 % are we in first stance phase?
        case 1
            fAnow = fA{1}; % yes
        case 0
            fAnow = fA{2}; % no
    end
    gDot = ...
        rot_SE2(y(3))*... % orientation of body relative to rest frame
        -fAnow... % local connection (- sign for the motility map)
        (a, l, ab, a_t(1), a_t(2), a_t(3), a_t(4))*... 
        aDot_t; % swing angular velocity
    % ... the terms are ordered as xDot, yDot, and thetaDot

end