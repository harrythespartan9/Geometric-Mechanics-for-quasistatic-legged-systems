function gDot = computeBodyVelocityV1(t, y, fA, sHat, sP, a, l, ab)
    %COMPUTEBODYVELOCITYV1 compute the body velocity for sinusoidal shape
    %trajectories
    %   Provided with sinusoidal shape trajectoies, the current body
    %   velocity generated through the locomotion jacobian is returned, it
    %   is designed to work with V1 of the maneuverability gait parameters.

    % obtain the current swing shape trajctory
    [a_t, aDot_t] = computeSineShapeTrajectoriesV1(t, sHat, sP);

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