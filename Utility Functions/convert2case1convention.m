function rout = convert2case1convention(rin)
%CONVERT2CASE1CONVENTION The given shape-space trajectory (shape or shape-velocity) is converted to the SE(2) kinematics convention.
%   The experimental convention for HAMR 6's dataset assumes a different structure to what we are using in the SE(2) modeling frame for rigid planar systems.
%   This function takes this experimental kinematics and modulates it into the form we need for planar analysis.
    if iscell(rin)
        rout = cell(size(rin));
        for i = 1:numel(rin)
            if i == 2  || i == 3
                rout{i} = -rin{i}; % the hip frame is rotated by pi on the left side of the body-- flipping the swing angle convention
            else
                rout{i} = rin{i};
            end
        end
    else
        error('ERROR! The elements of the shape-space trajectory needs to be a cell array.')
    end

end

