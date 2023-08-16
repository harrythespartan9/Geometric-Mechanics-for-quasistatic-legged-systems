function [out_fwd, out_stitch] = verifyCurvatureProfile(data, ix, iy, plotFlag, outFlag)
%VERIFYCURVATUREPROFILE compare the difference based curvature estimate with the "Path2" class method estimate
%   Two different kinds of curvature estimates are compared-- 
%   1) the forward difference estimate is coarsely taken from the integrated trajectories
%   2) the "Path2" class estimate is obtained using velocities
%   This function takes these estimates and plots them on top of each other. Also, the mean curvature is estimated by the motion planning function and it is
%   also plotted on top.

    % unpack
    t = data.gaits{ix,iy}.trajectory{1};
    x = data.gaits{ix,iy}.trajectory{2};
    y = data.gaits{ix,iy}.trajectory{3};
    theta = data.gaits{ix,iy}.trajectory{4};
    zx = data.gaits{ix,iy}.trajectory{15}; 
    zy = data.gaits{ix,iy}.trajectory{16}; 
    ztheta = data.gaits{ix,iy}.trajectory{17};
    
    % compute
    dx = [diff(x), zx-x(end)]; 
    dy = [diff(y), zy-y(end)]; 
    dtheta = [diff(theta), ztheta-theta(end)];
    curv_t = dtheta./sqrt(dx.^2 + dy.^2); % backward difference based curvature estimate
    t_stitch = [data.path_i.open_trajectory{ix}{1}, data.path_i.path_length{ix} + data.path_j.open_trajectory{iy}{1}]; % stitch path 2 values
    curv_t_stitch = [data.path_i.path_curvature_traj{ix}, data.path_j.path_curvature_traj{iy}];
    mean_curv_t_stitch = data.path_i.path_net_curvature{ix};
    
    % pack if needed
    switch outFlag
        case 1
            out_fwd = {t, curv_t, mean(curv_t)}';
            out_stitch = {t_stitch, curv_t_stitch, mean(curv_t_stitch)}';
        case 0
            out_fwd = [];
            out_stitch = [];
    end
    
    % plot if needed based on "plotFlag"
    if plotFlag

        figure()
        subplot(3, 1, 1)
        yyaxis left
        plot(t, x, 'k', 'LineWidth', 1.2, 'DisplayName', 'x[t]');
        hold on; grid on;
        plot(t, y, 'k--', 'LineWidth', 1.2, 'DisplayName', 'y[t]');
        xlabel('Time');
        yyaxis right
        plot(t, theta, 'r', 'LineWidth', 1.2, 'DisplayName', 'theta[t]');
        legend('Location','north');

        subplot(3, 1, 2)
        yyaxis left
        plot(t, sqrt(dx.^2 + dy.^2), 'k', 'LineWidth', 1.2, 'DisplayName', 'ds[t]');
        grid on; xlabel('Time');
        yyaxis right
        plot(t, dtheta, 'r', 'LineWidth', 1.2, 'DisplayName', 'dtheta[t]');
        legend('Location','north');

        subplot(3, 1, 3)
        plot(t, curv_t, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Forward Difference');
        hold on; grid on;
        plot(t_stitch, curv_t_stitch, 'b--', 'LineWidth', 1.2, 'DisplayName', '"Path2" computed');
        plot(t_stitch, repmat(data.kappa_S(ix, iy), 1, numel(t_stitch)), 'ro', 'LineWidth', 1.2, 'DisplayName', 'quadMP average curvature');
        plot(t_stitch, repmat(mean(curv_t_stitch), 1, numel(t_stitch)), 'bo', 'LineWidth', 1.2, 'DisplayName', 'stitch average curvature');
        plot(t_stitch, repmat(mean(curv_t), 1, numel(t_stitch)), 'ko', 'LineWidth', 1.2, 'DisplayName', 'stitch average curvature');
        xlabel('Time'); ylabel('Curvature value (1/R)'); legend('Location', 'north');

    end


end

