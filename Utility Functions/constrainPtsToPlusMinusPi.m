function constrainedPt = constrainPtsToPlusMinusPi(pt)
%CONSTRAINPTSTOPLUSMINUSPI convert angular points to within the range of
%+-pi
    
    % condition pts not to have more than one rotation
    if pt > 2*pi
        pt = mod(pt, 2*pi);
    end
    if pt < -2*pi
        pt = mod(pt, -2*pi);
    end
    % final conditions
    if pt > pi && pt < 2*pi
        pt = mod(pt, -2*pi);
    end
    if pt < -pi && pt > -2*pi
        pt = mod(pt, 2*pi);
    end
    constrainedPt = pt;

end

