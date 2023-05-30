function out = mul2case1(in)
%MUL2CASE1 compute the multipler to convert r and r_dot to case_1 format
%   Given the input leg module index for a quadrupedal robot, this function helps us find the multiplier to convert shape and shape velocities from HAMR format
%   to case 1 kinematics format.
    switch in == 2 || in == 3
        case 0
            out = +1;
        case 1
            out = -1;
    end
end

