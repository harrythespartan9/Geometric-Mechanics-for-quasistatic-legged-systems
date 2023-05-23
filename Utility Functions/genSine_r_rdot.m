function [r, r_dot] = genSine_r_rdot(in, t)
%GENSINE_R_RDOT generate pure sinusoidal shape and shape velocity trajectories from fit parameters
%   Given estimated pure sinusoid parameters for the shape, phase space trajectories, estimate them for each shape element when provided with a time-vector.
    switch iscell(in)
        case 1
            r = cell(size(in)); r_dot = cell(size(in));
            for i = 1:numel(in)
                r{i} = genswing_t(t, in{i}); r_dot{i} = genswingrate_t(t, in{i});
            end
        case 0
            r = genswing_t(t, in); r_dot = genswingrate_t(t, in);
    end
end

