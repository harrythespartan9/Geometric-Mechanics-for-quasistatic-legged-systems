% This script computes the unit-square control inputs to combine two paths in complementary level-2 contact states of quadrupedal robot with a noslip
% constraint.
function dataij = noslip2bgaits(pi, pj, dataij)

    % Unpack everything you need
    dnum = 0.5*(pi.path_discretization + pj.path_discretization); % number of points for plotting the gait
    dc_i = pi.deadband_dutycycle; dc_j = pj.deadband_dutycycle; % deadband period
    csi_idx = pi.active_state; csj_idx = pj.active_state; % contact states numbers; ordering-- [12, 23, 34, 41, 13, 24]

    % the percentage scaling in both directions (as a proper decimal)
    switch numel(pi.open_trajectory)/2 == floor(numel(pi.open_trajectory)/2) % checking if it is even or odd
        case 1
            u_i = [ fliplr(-10*(1:numel(pi.open_trajectory)/2)), 10*(1:numel(pi.open_trajectory)/2) ]/100;
        case 0
            u_i = [ fliplr(-10*(1:numel(pi.open_trajectory)/2)), 0, 10*(1:numel(pi.open_trajectory)/2) ]/100;
    end
    switch numel(pj.open_trajectory)/2 == floor(numel(pj.open_trajectory)/2)
        case 1
            u_j = [ fliplr(-10*(1:numel(pj.open_trajectory)/2)), 10*(1:numel(pj.open_trajectory)/2) ]/100;
        case 0
            u_j = [ fliplr(-10*(1:numel(pj.open_trajectory)/2)), 0, 10*(1:numel(pj.open_trajectory)/2) ]/100;
    end
    ni = numel(u_i);
    nj = numel(u_j);
    [u_i_S, u_j_S] = meshgrid(u_i, u_j);

    % % get the net path curvature as a function of the inputs (OLD)
    % [kappa_i_S, kappa_j_S] = meshgrid( cell2mat(pi.path_net_curvature), cell2mat(pj.path_net_curvature) );
    % kappa_S = 0.5*(kappa_i_S + kappa_j_S); % take the mean value
    
    % create an cell array to scale and hold both positive and negatively scaled paths
    gaits = cell( ni, nj ); kappa_S = nan( ni, nj );
    
    % Iterate over each path scaling in each contact state
    for i = 1:ni
        for j = 1:nj
    
            % get the time periods
            tau_i = pi.path_length{i}; tau_j = pj.path_length{i};
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
            
            phi_start = t2periodarr(phi_tau);       % starting times for the periods
            phi_start_i = t2periodarr(phi_tau_i);
            phi_start_j = t2periodarr(phi_tau_j);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % gait time vector
            t = linspace(0, sum(phi_tau), dnum + 1); t = t(1:end-1);

            % Obtain the trajectory information
            xi     = pi.open_trajectory{i}{2};
            yi     = pi.open_trajectory{i}{3};
            thetai = pi.open_trajectory{i}{4};
            zxi    = pi.net_displacement(1,i);
            zyi    = pi.net_displacement(2,i);
            zthetai= pi.net_displacement(3,i);
            a1_i   = pi.open_trajectory{i}{5};
            a2_i   = pi.open_trajectory{i}{6};

            
            xj     = pj.open_trajectory{j}{2};
            yj     = pj.open_trajectory{j}{3};
            thetaj = pj.open_trajectory{j}{4};
            zxj    = pj.net_displacement(1,j);
            zyj    = pj.net_displacement(2,j);
            zthetaj= pj.net_displacement(3,j);
            a1_j   = pj.open_trajectory{j}{5};
            a2_j   = pj.open_trajectory{j}{6};
            
            % get the positive scaling and negative scaling trajectories
            [ gaits{i, j}.trajectory, gaits{i, j}.periods ] ...
                                                        = stitch_noslip2b_trajectories(csi_idx, csj_idx,...
                                                                                    phi_start, phi_tau, phi_state,...
                                                                                    phi_start_i, phi_start_j,...
                                                                                    phi_tau_i, phi_tau_j,...
                                                                                    phi_state_i, phi_state_j,...
                                                                                    t,...
                                                                                    xi, yi, thetai,...
                                                                                    zxi, zyi, zthetai,...
                                                                                    a1_i, a2_i,...
                                                                                    xj, yj, thetaj,...
                                                                                    zxj, zyj, zthetaj,...
                                                                                    a1_j, a2_j);

            % get the net path curvature as a function of the inputs using the net displacement achieved by the gait cycle
            z = [gaits{i, j}.trajectory{15}; 
                 gaits{i, j}.trajectory{16}; 
                 gaits{i, j}.trajectory{17}];
            kappa_S( i, j ) = norm( [eye(2,2), zeros(2,1)]*z, 2 )/( 2*sin(z(3)/2) );
            % the selection matrix inside the 'norm' selects the translational components

        end
    end

    % Return the gaits structure
    dataij.path_i = pi;   % noslip level-2 path objects
    dataij.path_j = pj;
    dataij.u_i = u_i;     % arrays
    dataij.u_j = u_j;
    dataij.u_i_S = u_i_S; % sweep
    dataij.u_j_S = u_j_S;
    dataij.kappa_S = kappa_S;
    dataij.gaits = gaits; % ui, uj swept gaits and associated periods
    dataij.tau = phi_tau; % time periods

end