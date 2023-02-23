% This function stitches together trajectories from two paths in complimentary contact states to generate gaits.
function [ ptrajectory, mtrajectory, ...
    pperiod, mperiod ] = stitch_noslip2b_trajectories(csi_idx, csj_idx,...
                                                                    phi_start, phi_tau, phi_state,...
                                                                    phi_start_i, phi_start_j,...
                                                                    phi_tau_i, phi_tau_j,...
                                                                    phi_state_i, phi_state_j,...
                                                                    t,...
                                                                    xi, yi, thetai, a1_i, a2_i,...
                                                                    xj, yj, thetaj, a1_j, a2_j)
    
    % indices before, after, active, and switching phases of the contact state under consideration
    idxi_before =    t < phi_start(phi_state == csi_idx);
    idxi_after  =    t >= ( phi_start(phi_state == csi_idx) + phi_tau(phi_state == csi_idx) );
    idxi_during =    ~idxi_before & ~idxi_after;
    idxi_switch =    t >= phi_start_i(phi_state_i == 0.5) & t < ( phi_start_i(phi_state_i == 0.5) + phi_tau_i(phi_state_i == 0.5) );

    idxj_before =    t < phi_start(phi_state == csj_idx);
    idxj_after  =    t >= phi_start(phi_state == csj_idx) + phi_tau(phi_state == csj_idx);
    idxj_during =    ~idxj_before & ~idxj_after;
    idxj_switch =    t >= phi_start_j(phi_state_j == 0.5) & t < ( phi_start_j(phi_state_j == 0.5) + phi_tau_j(phi_state_j == 0.5) );

    % compute the points where we need to plot (same for both positive and negative gaits)
    plotbodyq = zeros(1, numel(idxi_before)); plotbodyq( find(idxi_during, 1, 'last') ) = 1; plotbodyq( find(idxj_during, 1, 'last') ) = 1;

    % compute the different active contact states as a function of gait kinematic phase
    phi_period = 0*idxi_before + csi_idx*idxi_during + csj_idx*idxj_during + 0*idxj_after;
    
    % ----------------------------------------------------------------------------------------------------------------------------------------------------------
    % compute ith and jth leg angles for i and j contact states
    [A1_i, A2_i] = stitchshapefor2bgait(idxi_before, idxi_during, idxi_switch, idxi_after, a1_i, a2_i, 1);
    [A1_j, A2_j] = stitchshapefor2bgait(idxj_before, idxj_during, idxj_switch, idxj_after, a1_j, a2_j, 2);

    % compute the kinematics
    phi_i = [xi; yi; thetai];
    phi_j = repmat( phi_i(:,end), 1, size([xj; yj; thetaj], 2) ) + adginv(thetai(end))*[xj; yj; thetaj];
    b_phi = repmat( phi_i(:,1), 1, sum(idxi_before) ) + ... % before csi
        interp1( 1:numel(xi), phi_i, linspace(1,numel(xi),sum(idxi_during)) ) + ... % during csi
        interp1( 1:numel(xj), phi_j, linspace(1,numel(xj),sum(idxj_during)) ) + ... % during csj
        repmat( phi_j(:,end), 1, sum(idxj_after) ); % after csj

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

    % ----------------------------------------------------------------------------------------------------------------------------------------------------------
    % flip the trajectories to scale in the negative direction
    a1_i = fliplr(a1_i); a2_i = fliplr(a2_i);
    xi = fliplr(xi) - xi(end); yi = fliplr(yi) - yi(end); thetai = fliplr(thetai) - thetai(end);
    a1_j = fliplr(a1_j); a2_j = fliplr(a2_j);
    xj = fliplr(xj) - xj(end); yj = fliplr(yj) - yj(end); thetaj = fliplr(thetaj) - thetaj(end);

    % repeat earlier steps
    [A1_i, A2_i] = stitchshapefor2bgait(idxi_before, idxi_during, idxi_switch, idxi_after, a1_i, a2_i, 1);
    [A1_j, A2_j] = stitchshapefor2bgait(idxj_before, idxj_during, idxj_switch, idxj_after, a1_j, a2_j, 2);
    
    phi_i = [xi; yi; thetai];
    phi_j = repmat( phi_i(:,end), 1, size([xj; yj; thetaj], 2) ) + adginv(thetai(end))*[xj; yj; thetaj];
    b_phi = repmat( phi_i(:,1), 1, sum(idxi_before) ) + ...
        interp1( 1:numel(xi), phi_i, linspace(1,numel(xi),sum(idxi_during)) ) + ...
        interp1( 1:numel(xj), phi_j, linspace(1,numel(xj),sum(idxj_during)) ) + ...
        repmat( phi_j(:,end), 1, sum(idxj_after) );
    
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

end