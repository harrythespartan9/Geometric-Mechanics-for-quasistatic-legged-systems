% This function stitches together trajectories from two paths in complimentary contact states to generate gaits.
function [ trajectory, period ] = stitch_noslip2b_trajectories(csi_idx, csj_idx,...
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
                                                                    a1_j, a2_j)
    
    % indices before, after, active, and switching phases of the contact state under consideration
    idxi_before =    t < phi_start_i(phi_state_i == 1.0);
    idxi_during =    t >= phi_start_i(phi_state_i == 1.0) & t < phi_start_i(phi_state_i == 0.5);
    idxi_switch =    t >= phi_start_i(phi_state_i == 0.5) & t < phi_start_i(phi_state_i == 0.5) + phi_tau_i(phi_state_i == 0.5);
    idxi_after  =    ~idxi_before & ~idxi_during & ~idxi_switch;

    idxj_before =    t < phi_start_j(phi_state_j == 0.5);
    idxj_switch =    t >= phi_start_j(phi_state_j == 0.5) & t < phi_start_j(phi_state_j == 1.0);
    idxj_during =    t >= phi_start_j(phi_state_j == 1) & t < phi_start_j(phi_state_j == 1) + phi_tau_j(phi_state_j == 1);
    idxj_after  =    ~idxj_before & ~idxj_switch & ~idxj_during;

    % compute the points where we need to plot (same for both positive and negative gaits)
    plotbodyq = zeros(1, numel(idxi_before)); plotbodyq( find(idxi_during, 1, 'last') ) = 1; plotbodyq( find(idxj_during, 1, 'last') ) = 1;

    % compute the different active contact states as a function of gait kinematic phase
    phi_period = 0*idxi_before + csi_idx*idxi_during + csj_idx*idxj_during + 0*idxj_after;
    
    % ------------------------------------------------------------------------------------------------------------------------------------------( +, + )--------
    % compute ith and jth leg angles for i and j contact states
    [A1_i, A2_i] = stitchshapefor2bgait(idxi_before, idxi_during, idxi_switch, idxi_after, a1_i, a2_i, 1);
    [A1_j, A2_j] = stitchshapefor2bgait(idxj_before, idxj_during, idxj_switch, idxj_after, a1_j, a2_j, 2);

    % compute the kinematics
    [b_phi, z_phi] = stitchSE2trajectories(xi, yi, thetai, zxi, zyi, zthetai, xj, yj, thetaj, zxj, zyj, zthetaj, idxi_before, idxi_during, idxj_during, idxj_after);

    % Store the positive scaling gait results
    trajectory{1} = t;
    trajectory{2} = b_phi(1,:);
    trajectory{3} = b_phi(2,:);
    trajectory{4} = b_phi(3,:);
    trajectory{5} = A1_i;
    trajectory{6} = A2_i;
    trajectory{7} = A1_j;
    trajectory{8} = A2_j;
    trajectory{9} = plotbodyq;     % don't delete the body configuration plots at these points in time during animation
    trajectory{10} = idxi_during;  % ith shape-space slice when active
    trajectory{11} = idxi_switch;  % ith shape-space slice when in deadband
    trajectory{12} = idxj_during;  % same as above for jth slice
    trajectory{13} = idxj_switch;
    trajectory{14} = phi_period;   % divides the gait cycle into 4 different time periods-- before i, during i, during j, after j
    trajectory{15} = z_phi(1);
    trajectory{16} = z_phi(2);
    trajectory{17} = z_phi(3);

    % Store the gait period information
    period.phi_start   = phi_start;
    period.phi_start_i = phi_start_i;
    period.phi_start_j = phi_start_j;
    period.phi_tau     = phi_tau;
    period.phi_tau_i   = phi_tau_i;
    period.phi_tau_j   = phi_tau_j;
    period.phi_state   = phi_state;
    period.phi_state_i = phi_state_i;
    period.phi_state_j = phi_state_j;

end