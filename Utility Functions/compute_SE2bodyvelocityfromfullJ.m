function bdotS = compute_SE2bodyvelocityfromfullJ(t, aa, ll, x, in, p)
%COMPUTE_BODYVELOCITYFROMFULLJ Computes the body velocity when provided with contact and shape kinematics of a quadrupedal system.
%   This function when provided with a case_1 system computes the body velocity given the shape and contact trajectory (normally HAMR/CLARI kinematics).

    % Unpack
    c = in{1}; J = in{2}; T = in{5};
    if nargin < 6
        flag = false;
    else
        flag = true;
    end
    if ~flag
        r = in{3}; r_dot = in{4};
        % Interpolate using the time vector to determine the current shape and shape velocity
        [tc, tr, tr_dot] = interpHAMRexp(t, {T, c, r, r_dot});
    else
        r = in{3}; r_dot = in{4};
        % jusy interpolate the contact trajectory
        [tc, ~, ~] = interpHAMRexp(t, {T, c, r, r_dot}); tr = nan(numel(r), 1); tr_dot = nan(numel(r_dot), 1);
        for i = 1:numel(r)
            % compute the pure sinusoidal swing and lift trajectories
            tr(i) = genswing_t(t, p{i});
            tr_dot(i) = genswingrate_t(t, p{i});
        end
    end

    % Initialize symbolic variables we need
    syms a l alpha_1 alpha_2 alpha_3 alpha_4 real

    % Construct the pfaffian constraint from the current active contact states and then compute the current local connection
    pfaff = [];
    for i = 1:numel(c)
        pfaff = [pfaff; tc(i)*J{i}]; % generate the current local connection
    end
    pfaff = double( subs( pfaff, [a, l, alpha_1, alpha_2, alpha_3, alpha_4], [aa, ll, tr(1), tr(2), tr(3), tr(4)] ) );
    bdotS = -TeLg(x(3)) * (pfaff(:, 1:3)\pfaff(:, 4:end)) * tr_dot;

end