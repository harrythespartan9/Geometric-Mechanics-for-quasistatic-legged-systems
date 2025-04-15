% This function defines the phase offset in such a way to interpolate
% between longitudinal and lateral displacement gaits to sweep out the
% manueverability sets.
function sPhase = computeSineShapePhaseV1(mPhase)
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