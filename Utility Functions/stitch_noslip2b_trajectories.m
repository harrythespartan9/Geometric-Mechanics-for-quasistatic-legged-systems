% This function stitches together trajectories from two paths in complimentary contact states to generate gaits.
function [ptrajectory, mtrajectory] = stitch_noslip2b_trajectories(csi_idx, csj_idx,...
                                                                    phi_start, phi_tau, phi_state,...
                                                                    t,...
                                                                    xi, yi, thetai, a1_i, a2_i,...
                                                                    xj, yj, thetaj, a1_j, a2_j)
    
    % indices before, after, and during the contact state under consideration
    idxi_before =    t < phi_start(phi_state == csi_idx);
    idxi_after  =    t >= phi_start(phi_state == csi_idx) + phi_tau(phi_state == csi_idx);
    idxi_during =    ~idxi_before & ~idxi_after;

    idxj_before =    t < phi_start(phi_state == csj_idx);
    idxj_after  =    t >= phi_start(phi_state == csj_idx) + phi_tau(phi_state == csj_idx);
    idxj_during =    ~idxj_before & ~idxj_after;
    
    % ----------------------------------------------------------------------------------------------------------------------------------------------------------
    % compute ith and jth leg angles for i and j contact states
    A1_i = a1_i(1)*idxi_before + interp1( 1:numel(a1_i), a1_i, linspace(1,numel(a1_i),sum(idxi_during) ), 'spline' ) + a1_i(end)*idxi_after;
    A2_i = a2_i(1)*idxi_before + interp1( 1:numel(a2_i), a2_i, linspace(1,numel(a2_i),sum(idxi_during) ), 'spline' ) + a2_i(end)*idxi_after;
    
    A1_j = a1_j(1)*idxj_before + interp1( 1:numel(a1_j), a1_j, linspace(1,numel(a1_j),sum(idxj_during) ), 'spline' ) + a1_j(end)*idxj_after;
    A2_j = a2_j(1)*idxj_before + interp1( 1:numel(a2_j), a2_j, linspace(1,numel(a2_j),sum(idxj_during) ), 'spline' ) + a2_j(end)*idxj_after;

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
    
    % ----------------------------------------------------------------------------------------------------------------------------------------------------------
    % flip the trajectories to scale in the negative direction
    a1_i = fliplr(a1_i); a2_i = fliplr(a2_i);
    xi = fliplr(xi) - xi(end); yi = fliplr(yi) - yi(end); thetai = fliplr(thetai) - thetai(end);
    a1_j = fliplr(a1_j); a2_j = fliplr(a2_j);
    xj = fliplr(xj) - xj(end); yj = fliplr(yj) - yj(end); thetaj = fliplr(thetaj) - thetaj(end);

    % repeat earlier steps
    A1_i = a1_i(1)*idxi_before + interp1( 1:numel(a1_i), a1_i, linspace(1,numel(a1_i),sum(idxi_during) ), 'spline' ) + a1_i(end)*idxi_after;
    A2_i = a2_i(1)*idxi_before + interp1( 1:numel(a2_i), a2_i, linspace(1,numel(a2_i),sum(idxi_during) ), 'spline' ) + a2_i(end)*idxi_after;
    
    A1_j = a1_j(1)*idxj_before + interp1( 1:numel(a1_j), a1_j, linspace(1,numel(a1_j),sum(idxj_during) ), 'spline' ) + a1_j(end)*idxj_after;
    A2_j = a2_j(1)*idxj_before + interp1( 1:numel(a2_j), a2_j, linspace(1,numel(a2_j),sum(idxj_during) ), 'spline' ) + a2_j(end)*idxj_after;
    
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
