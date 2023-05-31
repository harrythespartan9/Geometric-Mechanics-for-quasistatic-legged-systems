function bdotS = compute_SE2bodyvelocityfromfullJ_timeseries(input, robot_params, robot_kinematics, tSpecific)
%COMPUTE_BODYVELOCITYFROMFULLJ Computes the body velocity when provided with contact and shape kinematics of a quadrupedal system.
%   This function when provided with a case_1 system computes the body velocity given the shape and contact trajectory (normally HAMR/CLARI kinematics). This
%   code is originally built for the ode integration environment, but is now expanded to other cases where timeseries can be provided instead of a single point.
    
    warning('off');

    % Unpack
    t = input{1}; c = input{2}; r = input{3}; r_dot = input{4}; b = input{5};
    if numel(input) < 6
        velReq_str = 'Rest'; % we want the rest frame velocities by default
    else
        velReq_str = input{6};
    end
    aa = robot_params{1}; ll = robot_params{1};
    pfaff = robot_kinematics;

    % Convert HAMR format to case 1 kinematics format -- swing, swing-vel, and body trajectory
    r = convert2case1convention(r); r_dot = convert2case1convention(r_dot);
    b = {-b{2}, b{1}, b{3}}';
    
    % Check if a specific time-step is required, or then proceed with estimating the whole timeseries
    switch nargin == 4

        case 1 % need the body velocity at a specific points

            % Interpolate using the time vector to determine the current shape and shape velocity
            [tc, tr, tr_dot] = interpHAMRexp(tSpecific, {t, c, r, r_dot});
            if strcmp(velReq_str, 'Rest')
                tb = interpTimeseriesData({tSpecific, t, b});
            end

            % Compute the pfaffian constraint with the
            pfaff_t = pfaff(aa, ll, tr(1), tr(2), tr(3), tr(4));
            C_t = nan(size(pfaff_t));
            for i = 1:numel(c)
                C_t(2*(i-1)+1:2*i, :) = repmat(tc(i), 2, size(pfaff_t, 2));
            end
            pfaff_t = C_t.*pfaff_t;
            switch velReq_str
                case 'Rest'
                    bdotS = -TeLg(tb(2)) * (pfaff_t(:, 1:3)\pfaff_t(:, 4:end)) * tr_dot;
                case 'Body'
                    bdotS = -(pfaff_t(:, 1:3)\pfaff_t(:, 4:end)) * tr_dot;
                otherwise
                    error('ERROR! Only "Rest" frame and "Body" frame are supported.');
                    % Need to add support for an SE(3) transform to the body frame from an arbitrary frame.
            end

            % Convert the body velocity back to HAMR format
            bdotS = [bdotS(2), -bdotS(1), bdotS(3)]';

        otherwise % need the body velocity for the whole trajectory
            
            bdotS = cell(3, 1); % output body velocity container

            for i = 1:numel(t)
                
                % Get the current time-step information
                tc = retrieve_ithTimeStepData(c, i); 
                tr = retrieve_ithTimeStepData(r, i); 
                tr_dot = retrieve_ithTimeStepData(r_dot, i);
                if strcmp(velReq_str, 'Rest')
                    tb = retrieve_ithTimeStepData(b, i);
                end
                
                % Compute
                pfaff_t = pfaff(aa, ll, tr(1), tr(2), tr(3), tr(4));
                C_t = nan(size(pfaff_t));
                for j = 1:numel(c)
                    C_t(2*(j-1)+1:2*j, :) = repmat(tc(j), 2, size(pfaff_t, 2));
                end
                pfaff_t = C_t.*pfaff_t;
                switch velReq_str
                    case 'Rest'
                        temp = -TeLg(tb(3)) * (pfaff_t(:, 1:3)\pfaff_t(:, 4:end)) * tr_dot;
                    case 'Body'
                        temp = -(pfaff_t(:, 1:3)\pfaff_t(:, 4:end)) * tr_dot;
                    otherwise
                        error('ERROR! Only "Rest" frame and "Body" frame are supported.');
                end

                % save (directly in HAMR format)
                bdotS{1}(i) = temp(2); bdotS{2}(i) = -temp(1); bdotS{3}(i) = temp(3);

            end

    end

    warning('on');

end