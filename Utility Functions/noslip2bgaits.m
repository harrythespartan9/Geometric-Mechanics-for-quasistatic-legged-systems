% This script computes the unit-square control inputs to combine two paths in complementary level-2 contact states of quadrupedal robot with a noslip
% constraint.
function dataij = noslip2bgaits(pi, pj)

    % Unpack everything you need
    dnum = 0.5*(pi.path_discretization + pj.path_discretization); % number of points for plotting the gait
    dc_i = pi.deadband_dutycycle; dc_j = pj.deadband_dutycycle; % deadband period
    csi_idx = pi.active_state; csj_idx = pj.active_state; % contact states numbers; ordering-- [12, 23, 34, 41, 13, 24]

    % the percentage scaling in both directions (as a proper decimal)
    u_i = [ fliplr(-10*(1:numel(pi.open_trajectory))), 10*(1:numel(pi.open_trajectory)) ]/100;
    u_j = [ fliplr(-10*(1:numel(pj.open_trajectory))), 10*(1:numel(pj.open_trajectory)) ]/100;
    ni = numel(u_i);
    nj = numel(u_j);
    [u_i_S, u_j_S] = meshgrid(u_i, u_j);
    
    % create an cell array to scale and hold both positive and negatively scaled paths
    gaits = cell( ni, nj );
    
    % Iterate over each path scaling in each contact state
    for im = 1:ni
        for jm = 1:nj
            
            % get the control input for positive scaling values (data exists here and we need to interpolate this to the current negative values)
            ip = ni - im + 1; jp = nj - jm + 1;
            i = ip - numel(pi.open_trajectory); j = jp - numel(pj.open_trajectory);
    
            % get the time periods
            tau_i = pi.pathlength{i}; tau_j = pj.pathlength{i};
            db_i = dc_i*tau_i; db_j = dc_j*tau_j;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % path category-- 1: both deadbands are smaller than their complementary stance times, 2: deadband of i is greater than the stance of j, 3: deadband
            % of j is greater than the stance of i, and 4: both deadbands are larger than their complementary stance times
            if db_i <= tau_j
                flag_i = true;
            else
                flag_i = false;
            end
            if db_j <= tau_i
                flag_j = true;
            else
                flag_j = false;
            end
            switch flag_i

                case true

                    switch flag_j

                        case true
                            
                            % category 1
                            h_i = tau_j - db_i; % inactive state at the end of ith shape-space slice
                            h_j = tau_i - db_j; % inactive state at the beginning of jth shape-space slice
                            phi_tau   = [ tau_i, tau_j ];
                            phi_tau_i = [ tau_i, db_i, h_i ];
                            phi_tau_j = [ h_j, db_j, tau_j ];
                            phi_state = [ csi_idx, csj_idx ];
                            phi_state_i = [1.0, 0.5, 0.0];
                            phi_state_j = [0.0, 0.5, 1.0];

                        case false

                            % category 3
                            h_i1 = db_j - tau_i; % inactive state in the beginning of gait cycle
                            h_i2 = tau_j - db_i; % inactive state at the end of the gait cycle
                            phi_tau   = [ h_i1, tau_i, tau_j ];
                            phi_tau_i = [ h_i1, tau_i, db_i, h_i2 ];
                            phi_tau_j = [ db_j, tau_j ];
                            phi_state = [ nan, csi_idx, csj_idx ];
                            phi_state_i = [0.0, 1.0, 0.5, 0.0];
                            phi_state_j = [0.5, 1.0];

                    end

                case false

                    switch flag_j

                        case true
                            
                            % category 2
                            h_j1 = tau_i - db_j; % inactive state at the beginning of jth shape-space slice
                            h_j2 = db_i - tau_j; % inactive state at the end of the gait cycle
                            phi_tau   = [ tau_i, tau_j, h_j2 ];
                            phi_tau_i = [ tau_i, db_i ];
                            phi_tau_j = [ h_j1, db_j, tau_j, h_j2 ];
                            phi_state = [ csi_idx, csj_idx, nan ];
                            phi_state_i = [1.0, 0.5];
                            phi_state_j = [0.0, 0.5, 1.0, 0.0];

                        case false
                            
                            % category 4
                            h_i = db_j - tau_i; % inactive state in the beginning of the gait cycle
                            h_j = db_i - tau_j; % inactive state at the end of the gait cycle
                            phi_tau   = [ h_i, tau_i, tau_j, h_j ];
                            phi_tau_i = [ h_i, tau_i, db_i ];
                            phi_tau_j = [ db_j, tau_j, h_j ];
                            phi_state = [ nan, csi_idx, csj_idx, nan ];
                            phi_state_i = [0.0, 1.0, 0.5];
                            phi_state_j = [0.5, 1.0, 0.0];

                    end
            end
            
            phi_start = tfromperiodarr(phi_tau);       % starting times for the periods
            phi_start_i = tfromperiodarr(phi_tau_i);
            phi_start_j = tfromperiodarr(phi_tau_j);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % gait time vector
            t = linspace(0, sum(phi_tau), dnum + 1); t = t(1:end-1);

            % Obtain the trajectory information for the positive path scaling
            xi     = pi.open_trajectory{i}{2};
            yi     = pi.open_trajectory{i}{3};
            thetai = pi.open_trajectory{i}{4};
            a1_i   = pi.open_trajectory{i}{5};
            a2_i   = pi.open_trajectory{i}{6};
            
            xj     = pi.open_trajectory{j}{2};
            yj     = pi.open_trajectory{j}{3};
            thetaj = pi.open_trajectory{j}{4};
            a1_j   = pi.open_trajectory{j}{5};
            a2_j   = pi.open_trajectory{j}{6};
            
            % get the positive scaling and negative scaling trajectories
            [ gaits{ip, jp}.trajectory, gaits{im, jm}.trajectory, ...
                gaits{ip, jp}.periods, gaits{im, jm}.periods ] = stitch_noslip2b_trajectories(csi_idx, csj_idx,...
                                                                     phi_start, phi_tau, phi_state,...
                                                                     phi_start_i, phi_start_j,...
                                                                     phi_tau_i, phi_tau_j,...
                                                                     phi_state_i, phi_state_j,...
                                                                     t,...
                                                                     xi, yi, thetai, a1_i, a2_i,...
                                                                     xj, yj, thetaj, a1_j, a2_j);
            
        end
    end

    % Return the gaits structure
    dataij.path_i = pi;   % noslip level-2 path objects
    dataij.path_j = pj;
    dataij.u_i = u_i;     % arrays
    dataij.u_j = u_j;
    dataij.u_i_S = u_i_S; % sweep
    dataij.u_j_S = u_j_S;
    dataij.gaits = gaits; % ui, uj swept gaits and associated periods
    dataij.tau = phi_tau; % time periods

end