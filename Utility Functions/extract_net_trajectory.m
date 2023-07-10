function [T, traj_tau] = extract_net_trajectory(traj, t, tau, requirement_str)
%EXTRACT_NET_TRAJECTORY this function computes the body trajectory array with the net displacement after each gait cycle
%   Given a trajectory 'b' (if vector trajectory should be in a cell array), the associated time vector 't', and time period 'tau'. Compute the body trajectory 
%   array after each gait cycle.

    switch iscell(traj)
        
        case 1

            % Check if all cell are of equal lengths and then check the length with the time vector
            verifylength(traj);
            if numel(traj{1}) ~= numel(t)
                error('ERROR! The length of each trajectory component should match the length of the time vector.');
            end
            
            % Initialize the return container
            traj_tau = cell(size(traj));
        
            % Compute the time vector array after each gait cycle
            T = t(1):tau:t(end);
        
            % Since the body trajectory is in a cell array, let's unpack and then interpolate the body trajectory data
            switch requirement_str
                case 'interpolate'
                    for i = 1:numel(traj)
                        traj_tau{i} = interp1(t, traj{i}, T, "pchip");
                    end
                case 'average'
                    T_start = T(1:end-1); T_end = T(2:end); % intialize the upper and lower bounds of the gait-cycle
                    for i = 1:numel(traj)
                        traj_tau{i} = nan(1, numel(T_start)); % initialize the current component
                        for j = 1:numel(T_start)
                            traj_tau{i}(j) = mean(traj{i}(t >= T_start(j) & t < T_end(j)));
                        end
                    end
            end

        case 0

            if numel(traj) ~= numel(t)
                error('ERROR! The length of the trajectory should match the length of the time vector.');
            end
            
            T = t(1):tau:t(end);
            
            switch requirement_str
                case 'interp1'
                    traj_tau = interp1(t, traj, T, "pchip");
                case 'avg'
                    T_start = T(1:end-1); T_end = T(2:end); % intialize the upper and lower bounds of the gait-cycle
                    traj_tau = nan(1, numel(T_start)); % initialize the current component
                    for j = 1:numel(T_start)
                        traj_tau(j) = mean(traj(t >= T_start(j) & t < T_end(j)));
                    end
            end

    end

end