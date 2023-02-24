% This function stitches together trajectories from two paths in complimentary contact states to generate gaits.
function [ ptrajectory, mtrajectory, mptrajectory, pmtrajectory, ...
    pperiod, mperiod, mpperiod, pmperiod ] = stitch_noslip2b_trajectories(csi_idx, csj_idx,...
                                                                    phi_start, phi_tau, phi_state,...
                                                                    phi_start_i, phi_start_j,...
                                                                    phi_tau_i, phi_tau_j,...
                                                                    phi_state_i, phi_state_j,...
                                                                    t,...
                                                                    xi, yi, thetai, a1_i, a2_i,...
                                                                    xj, yj, thetaj, a1_j, a2_j)
    
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
    b_phi = stitchSE2trajectories(xi, yi, thetai, xj, yj, thetaj, idxi_before, idxi_during, idxj_during, idxj_after);

    % Store the positive scaling gait results
    ptrajectory{1} = t;
    ptrajectory{2} = b_phi(1,:);
    ptrajectory{3} = b_phi(2,:);
    ptrajectory{4} = b_phi(3,:);
    ptrajectory{5} = A1_i;
    ptrajectory{6} = A2_i;
    ptrajectory{7} = A1_j;
    ptrajectory{8} = A2_j;
    ptrajectory{9} = plotbodyq;     % don't delete the body configuration plots at these points in time during animation
    ptrajectory{10} = idxi_during;  % ith shape-space slice when active
    ptrajectory{11} = idxi_switch;  % ith shape-space slice when in deadband
    ptrajectory{12} = idxj_during;  % same as above for jth slice
    ptrajectory{13} = idxj_switch;
    ptrajectory{14} = phi_period;   % divides the gait cycle into 4 different time periods-- before i, during i, during j, after j

    % Store the gait period information
    pperiod.phi_start   = phi_start;
    pperiod.phi_start_i = phi_start_i;
    pperiod.phi_start_j = phi_start_j;
    pperiod.phi_tau     = phi_tau;
    pperiod.phi_tau_i   = phi_tau_i;
    pperiod.phi_tau_j   = phi_tau_j;
    pperiod.phi_state   = phi_state;
    pperiod.phi_state_i = phi_state_i;
    pperiod.phi_state_j = phi_state_j;

    % ------------------------------------------------------------------------------------------------------------------------------------------( -, - )--------
    % flip the trajectories to scale in the negative direction
    a1_i_mm = fliplr(a1_i); a2_i_mm = fliplr(a2_i);
    xi_mm = fliplr(xi) - xi(end); yi_mm = fliplr(yi) - yi(end); thetai_mm = fliplr(thetai) - thetai(end);
    a1_j_mm = fliplr(a1_j); a2_j_mm = fliplr(a2_j);
    xj_mm = fliplr(xj) - xj(end); yj_mm = fliplr(yj) - yj(end); thetaj_mm = fliplr(thetaj) - thetaj(end);

    % repeat earlier steps
    [A1_i, A2_i] = stitchshapefor2bgait(idxi_before, idxi_during, idxi_switch, idxi_after, a1_i_mm, a2_i_mm, 1);
    [A1_j, A2_j] = stitchshapefor2bgait(idxj_before, idxj_during, idxj_switch, idxj_after, a1_j_mm, a2_j_mm, 2);
    
    b_phi = stitchSE2trajectories(xi_mm, yi_mm, thetai_mm, xj_mm, yj_mm, thetaj_mm, idxi_before, idxi_during, idxj_during, idxj_after);
    
    mtrajectory{1} = t;
    mtrajectory{2} = b_phi(1,:);
    mtrajectory{3} = b_phi(2,:);
    mtrajectory{4} = b_phi(3,:);
    mtrajectory{5} = A1_i;
    mtrajectory{6} = A2_i;
    mtrajectory{7} = A1_j;
    mtrajectory{8} = A2_j;
    mtrajectory{9} = plotbodyq;
    mtrajectory{10} = idxi_during;
    mtrajectory{11} = idxi_switch;
    mtrajectory{12} = idxj_during;
    mtrajectory{13} = idxj_switch;
    mtrajectory{14} = phi_period;

    mperiod.phi_start   = phi_start;
    mperiod.phi_start_i = phi_start_i;
    mperiod.phi_start_j = phi_start_j;
    mperiod.phi_tau     = phi_tau;
    mperiod.phi_tau_i   = phi_tau_i;
    mperiod.phi_tau_j   = phi_tau_j;
    mperiod.phi_state   = phi_state;
    mperiod.phi_state_i = phi_state_i;
    mperiod.phi_state_j = phi_state_j;

    % ------------------------------------------------------------------------------------------------------------------------------------------( -, + )--------
    [A1_i, A2_i] = stitchshapefor2bgait(idxi_before, idxi_during, idxi_switch, idxi_after, a1_i_mm, a2_i_mm, 1);
    [A1_j, A2_j] = stitchshapefor2bgait(idxj_before, idxj_during, idxj_switch, idxj_after, a1_j, a2_j, 2);
    
    b_phi = stitchSE2trajectories(xi_mm, yi_mm, thetai_mm, xj, yj, thetaj, idxi_before, idxi_during, idxj_during, idxj_after);
    
    mptrajectory{1} = t;
    mptrajectory{2} = b_phi(1,:);
    mptrajectory{3} = b_phi(2,:);
    mptrajectory{4} = b_phi(3,:);
    mptrajectory{5} = A1_i;
    mptrajectory{6} = A2_i;
    mptrajectory{7} = A1_j;
    mptrajectory{8} = A2_j;
    mptrajectory{9} = plotbodyq;
    mptrajectory{10} = idxi_during;
    mptrajectory{11} = idxi_switch;
    mptrajectory{12} = idxj_during;
    mptrajectory{13} = idxj_switch;
    mptrajectory{14} = phi_period;

    mpperiod.phi_start   = phi_start;
    mpperiod.phi_start_i = phi_start_i;
    mpperiod.phi_start_j = phi_start_j;
    mpperiod.phi_tau     = phi_tau;
    mpperiod.phi_tau_i   = phi_tau_i;
    mpperiod.phi_tau_j   = phi_tau_j;
    mpperiod.phi_state   = phi_state;
    mpperiod.phi_state_i = phi_state_i;
    mpperiod.phi_state_j = phi_state_j;

    % ------------------------------------------------------------------------------------------------------------------------------------------( +, - )--------
    [A1_i, A2_i] = stitchshapefor2bgait(idxi_before, idxi_during, idxi_switch, idxi_after, a1_i, a2_i, 1);
    [A1_j, A2_j] = stitchshapefor2bgait(idxj_before, idxj_during, idxj_switch, idxj_after, a1_j_mm, a2_j_mm, 2);
    
    b_phi = stitchSE2trajectories(xi, yi, thetai, xj_mm, yj_mm, thetaj_mm, idxi_before, idxi_during, idxj_during, idxj_after);
    
    pmtrajectory{1} = t;
    pmtrajectory{2} = b_phi(1,:);
    pmtrajectory{3} = b_phi(2,:);
    pmtrajectory{4} = b_phi(3,:);
    pmtrajectory{5} = A1_i;
    pmtrajectory{6} = A2_i;
    pmtrajectory{7} = A1_j;
    pmtrajectory{8} = A2_j;
    pmtrajectory{9} = plotbodyq;
    pmtrajectory{10} = idxi_during;
    pmtrajectory{11} = idxi_switch;
    pmtrajectory{12} = idxj_during;
    pmtrajectory{13} = idxj_switch;
    pmtrajectory{14} = phi_period;

    pmperiod.phi_start   = phi_start;
    pmperiod.phi_start_i = phi_start_i;
    pmperiod.phi_start_j = phi_start_j;
    pmperiod.phi_tau     = phi_tau;
    pmperiod.phi_tau_i   = phi_tau_i;
    pmperiod.phi_tau_j   = phi_tau_j;
    pmperiod.phi_state   = phi_state;
    pmperiod.phi_state_i = phi_state_i;
    pmperiod.phi_state_j = phi_state_j;

end