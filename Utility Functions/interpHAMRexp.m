function [tc, tr, tr_dot] = interpHAMRexp(t, data)
%INTERPHAMREXP This function computes the current swing and swing velocity information for a quadrupedal system from its discretely sampled trajectory data.
%   Given the current time t, and a shape trajectory with time stamps, we use linear interpolation to obtain the current shape. In the case of the contact
%   trajectory, we use the 'previous' point interpolation method (like ZOH).
    
    % Unpack
    T     = data{1};
    c     = data{2};
    r     = data{3};
    r_dot = data{4};

    % Interpolate to current time-step for each shape element
    tc = nan(numel(c), 1); tr = nan(numel(r), 1); tr_dot = nan(numel(r_dot), 1);
    for i = 1:numel(c)
        tc(i) = logical(interp1(T, double(c{i}), t, 'previous')); % no extrapolation needed
        tr(i) = interp1(T, r{i}, t, 'pchip');
        tr_dot(i) = interp1(T, r_dot{i}, t, 'pchip');
    end

end