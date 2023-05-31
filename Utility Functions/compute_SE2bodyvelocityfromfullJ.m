function bdotS = compute_SE2bodyvelocityfromfullJ(t, aa, ll, x, in)
%COMPUTE_BODYVELOCITYFROMFULLJ Computes the body velocity when provided with contact and shape kinematics of a quadrupedal system.
%   This function when provided with a case_1 system computes the body velocity given the shape and contact trajectory (normally HAMR/CLARI kinematics).

    % Unpack
    c = in{1}; pfaff = in{2}; T = in{5};
    r = in{3}; r_dot = in{4};
    
    % Interpolate using the time vector to determine the current shape and shape velocity
    [tc, tr, tr_dot] = interpHAMRexp(t, {T, c, r, r_dot});

    % % % % % % % % % % % LEGACY SYMBOLIC VERSION-- directly uses a function now
    % % % Initialize symbolic variables we need
    % % syms a l alpha_1 alpha_2 alpha_3 alpha_4 real

    % Construct the pfaffian constraint from the current active contact states and then compute the current local connection
    % % pfaff = [];
    % % for i = 1:numel(c)
    % %     pfaff = [pfaff; tc(i)*J{i}]; % generate the current local connection
    % % end
    % % pfaff = double( subs( pfaff, [a, l, alpha_1, alpha_2, alpha_3, alpha_4], [aa, ll, tr(1), tr(2), tr(3), tr(4)] ) );
    pfaff_t = pfaff(aa, ll, tr(1), tr(2), tr(3), tr(4));
    C_t = nan(size(pfaff_t));
    for i = 1:numel(c)
        C_t(2*(i-1)+1:2*i, :) = repmat(tc(i), 2, size(pfaff_t, 2));
    end
    pfaff_t = C_t.*pfaff_t;
    bdotS = -TeLg(x(3)) * (pfaff_t(:, 1:3)\pfaff_t(:, 4:end)) * tr_dot;

end