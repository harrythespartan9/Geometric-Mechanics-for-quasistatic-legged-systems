function hA = plot_SE2snapshot_QLvl2_Rigid_NoSlip(ax, pltTraj, F)
%ANIMATEQLVL2_RIGID_NOSLIP animate the rigid quadrupedal robot given the trajectory structure
%   The trajectory structure needed to run this function is generated by "StitchQuadSE2gaits.m" and for example refer to "se2_GenericQuad_trot.mlx". The
%   plotting structure follows the general outline in "qlevel2noslip_mp.m". This function returns the plotted objects for deletion in the parent animation
%   function.

    % if the trajectory case is not provided
    if nargin < 3
        F = [1 pltTraj.dnum];
    end
    switch numel(F) ~= 2
        case 1
            error('ERROR! We need both the last frame and current frame to plot the body trajectory.');
        case 0
            fNoOld = F(1); fNo = F(2);
    end

    % Unpack everything needed plotting
    plot_info = pltTraj.plot_info;
    lW = plot_info.lW; 
    lW_s = plot_info.lW_s; lW_r = plot_info.lW_r;
    lW_qf = plot_info.lW_qf; lW_kq = plot_info.lW_kq;
    frame_scale = plot_info.frame_scale; circS = plot_info.circS;

    anim_lim = pltTraj.anim_lim;
    leg__x = pltTraj.leg__x; leg__y = pltTraj.leg__y;
    legtip__x = pltTraj.legtip__x; legtip__y = pltTraj.legtip__y;
    O_leg__x = pltTraj.O_leg__x; O_leg__y = pltTraj.O_leg__y;
    body_link__x = pltTraj.body_link__x; body_link__y = pltTraj.body_link__y;
    body__x = pltTraj.body__x; body__y = pltTraj.body__y;
    bodyf__x = pltTraj.bodyf__x; bodyf__y = pltTraj.bodyf__y;
    ksq__x = pltTraj.ksq__x; ksq__y = pltTraj.ksq__y;
    col_t = pltTraj.col_t; S = pltTraj.S;
    phi_tau = pltTraj.phi_tau; pbq = pltTraj.pbq; x = pltTraj.x; y = pltTraj.y;
    
    % find the current config and snapshot points
    switch fNo == 1
        case 1
            idx = fNo;
        otherwise
            idx = [find(pbq(fNoOld:fNo-1)) + fNoOld-1, fNo];
    end

    % Initialize and start plotting ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    hA = cell(1, 0);
    m = 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot the body trajectory (plotted first because latter plots stack on top) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if (fNo ~= 1) % if not the first time step
        plotSeqSE2traj(ax, F, x, y, lW_s, phi_tau, col_t{7});
    end
    if ~ishold(ax)
        hold on;
    end
    axis equal; axis(anim_lim);

    % plot body and limbs
    for k = idx
        switch k == fNo

            case 0 % not final frame

                for i = 1:4
                    plot(ax, body_link__x{i}(:,k), body_link__y{i}(:,k), '--', 'Color', col_t{5}(:,k), 'LineWidth', 0.25*lW);
                end
                % plot the body axis frames
                quiver(ax, body__x(k), body__y(k), frame_scale*bodyf__x(1,k), frame_scale*bodyf__x(2,k), 'LineWidth', 0.25*lW_qf,...
                    'Color', col_t{5}(:,k),'LineStyle', '--',...
                        'AutoScale', 'off', 'ShowArrowHead', 'off');
                quiver(ax, body__x(k), body__y(k), frame_scale*bodyf__y(1,k), frame_scale*bodyf__y(2,k), 'LineWidth', 0.25*lW_qf,...
                    'Color', col_t{5}(:,k),'LineStyle', '--',...
                    'AutoScale', 'off', 'ShowArrowHead', 'off');
                

            case 1 % final frame
                for i = 1:4
                    if pbq(fNo) == 1
                        plot(ax, body_link__x{i}(:,k), body_link__y{i}(:,k), '--', 'Color', col_t{5}(:,k), 'LineWidth', 0.25*lW);
                    end
                    hA{m}  = plot(ax, body_link__x{i}(:,k), body_link__y{i}(:,k), 'Color', col_t{5}(:,k), 'LineWidth', lW); m = m + 1; % plot body i
                    if i == 1
                        set(ax, 'xticklabel', []); set(ax, 'yticklabel', []); box("off");
                        xline(ax, 0, ':', 'LineWidth', 0.5, 'Color', 'k');
                        yline(ax, 0, ':', 'LineWidth', 0.5, 'Color', 'k');
                    end
                    hA{m}  = plot(ax, leg__x{i}(:,k), leg__y{i}(:,k), 'Color', col_t{i}(:,k), 'LineWidth', lW); m = m + 1; % leg i
                    % if ith limb is in the current contact state
                    if i == S(phi_tau(k), 1) || i == S(phi_tau(k), 2)
                        hA{m}  = plot(ax, O_leg__x{i}(:,k), O_leg__y{i}(:,k), '--', 'Color', col_t{i}(:,k), 'LineWidth', lW_r); m = m + 1; % leg i origin
                        hA{m}  = scatter(ax, legtip__x{i}(:,k), legtip__y{i}(:,k), circS, col_t{i}(:,k)', 'filled'); m = m + 1; % legtip i scatter
                    end
                end
                if pbq(fNo) == 1
                    quiver(ax, body__x(k), body__y(k), frame_scale*bodyf__x(1,k), frame_scale*bodyf__x(2,k), 'LineWidth', 0.25*lW_qf,...
                        'Color', col_t{5}(:,k),'LineStyle', '--','AutoScale', 'off', 'ShowArrowHead', 'off');
                    quiver(ax, body__x(k), body__y(k), frame_scale*bodyf__y(1,k), frame_scale*bodyf__y(2,k), 'LineWidth', 0.25*lW_qf,...
                        'Color', col_t{5}(:,k),'LineStyle', '--','AutoScale', 'off', 'ShowArrowHead', 'off');
                end
                hA{m} = quiver(ax, body__x(k), body__y(k), frame_scale*bodyf__x(1,k), frame_scale*bodyf__x(2,k), 'LineWidth', lW_qf, 'Color', col_t{5}(:,k),...
                        'AutoScale', 'off', 'ShowArrowHead', 'off'); m = m + 1;
                hA{m} = quiver(ax, body__x(k), body__y(k), frame_scale*bodyf__y(1,k), frame_scale*bodyf__y(2,k), 'LineWidth', lW_qf, 'Color', col_t{5}(:,k),...
                    'AutoScale', 'off', 'ShowArrowHead', 'off'); m = m + 1;
                % plot the squared inter-leg distance
                hA{m} = plot(ax, ksq__x{phi_tau(k)}(:,k), ksq__y{phi_tau(k)}(:,k), 'Color', col_t{6}(:,k), 'LineWidth', lW_kq, 'LineStyle', '--'); m = m + 1;

        end
    end


end

%% TRAJECTORY SEQUENTIAL PLOT FUNCTION
function plotSeqSE2traj(ax, F, x, y, lW_s, phi_tau, colS)

    % unpack the previous frame and current frame
    fNoOld = F(1); % starting point
    fNo = F(2);    % end point
    
    % find the points where the contact states change (adding 1 to shift it into the correct index because of diff, else it will be fNoOld-1)
    iShift = find(diff(phi_tau(fNoOld:fNo)) ~= 0) + fNoOld; 
    iShift(end+1) = fNo; % appending the current frame number to the end
    
    % Just iterate over every single trajectory point
    for i = 1:numel(iShift)
        switch i
            case 1
                iStart = fNoOld; % the starting of a single cs trajectory (fNoOld)
            otherwise
                iStart = iEnd; % the last end point will be the current start point
        end
        iEnd = iShift(i); % the end point of a single cs trajectory

        % plot the trajectory chunk
        plot(ax, x(iStart:iEnd), y(iStart:iEnd), 'LineWidth', 2*lW_s, 'Color', colS(:,iStart));
        if i == 1 && ~ishold(ax)
            hold on; % hold all the plots
        end
    end

end

