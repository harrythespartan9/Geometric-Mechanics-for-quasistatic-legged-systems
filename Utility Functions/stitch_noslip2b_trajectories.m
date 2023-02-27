% This function stitches together trajectories from two paths in complimentary contact states to generate gaits.
function [ ptrajectory, mtrajectory, mptrajectory, pmtrajectory, ...
    pperiod, mperiod, mpperiod, pmperiod ] = stitch_noslip2b_trajectories(csi_idx, csj_idx,...
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
    ptrajectory{15} = z_phi(1);
    ptrajectory{16} = z_phi(2);
    ptrajectory{17} = z_phi(3);

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
    xi_mm = [xi zxi]; yi_mm = [yi zyi]; thetai_mm = [thetai zthetai];
    xi_mm = fliplr(xi_mm) - xi_mm(end); yi_mm = fliplr(yi_mm) - yi_mm(end); thetai_mm = fliplr(thetai_mm) - thetai_mm(end);
    zxi_mm = xi_mm(end); zyi_mm = yi_mm(end); zthetai_mm = thetai_mm(end);
    xi_mm = xi_mm(1:end-1); yi_mm = yi_mm(1:end-1); thetai_mm = thetai_mm(1:end-1);

    a1_j_mm = fliplr(a1_j); a2_j_mm = fliplr(a2_j);
    xj_mm = [xj zxj]; yj_mm = [yj zyj]; thetaj_mm = [thetaj zthetaj];
    xj_mm = fliplr(xj_mm) - xj_mm(end); yj_mm = fliplr(yj_mm) - yj_mm(end); thetaj_mm = fliplr(thetaj_mm) - thetaj_mm(end);
    zxj_mm = xj_mm(end); zyj_mm = yj_mm(end); zthetaj_mm = thetaj_mm(end);
    xj_mm = xj_mm(1:end-1); yj_mm = yj_mm(1:end-1); thetaj_mm = thetaj_mm(1:end-1);


    % repeat earlier steps
    [A1_i, A2_i] = stitchshapefor2bgait(idxi_before, idxi_during, idxi_switch, idxi_after, a1_i_mm, a2_i_mm, 1);
    [A1_j, A2_j] = stitchshapefor2bgait(idxj_before, idxj_during, idxj_switch, idxj_after, a1_j_mm, a2_j_mm, 2);
    
    [b_phi, z_phi] = stitchSE2trajectories(xi_mm, yi_mm, thetai_mm, zxi_mm, zyi_mm, zthetai_mm, xj_mm, yj_mm, thetaj_mm, zxj_mm, zyj_mm, zthetaj_mm,...
        idxi_before, idxi_during, idxj_during, idxj_after);
    
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
    mtrajectory{15} = z_phi(1);
    mtrajectory{16} = z_phi(2);
    mtrajectory{17} = z_phi(3);

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
    
    [b_phi, z_phi] = stitchSE2trajectories(xi_mm, yi_mm, thetai_mm, zxi_mm, zyi_mm, zthetai_mm, xj, yj, thetaj, zxj, zyj, zthetaj,...
        idxi_before, idxi_during, idxj_during, idxj_after);
    
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
    mptrajectory{15} = z_phi(1);
    mptrajectory{16} = z_phi(2);
    mptrajectory{17} = z_phi(3);

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
    
    [b_phi, z_phi] = stitchSE2trajectories(xi, yi, thetai, zxi, zyi, zthetai, xj_mm, yj_mm, thetaj_mm, zxj_mm, zyj_mm, zthetaj_mm,...
        idxi_before, idxi_during, idxj_during, idxj_after);
    
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
    pmtrajectory{15} = z_phi(1);
    pmtrajectory{16} = z_phi(2);
    pmtrajectory{17} = z_phi(3);

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