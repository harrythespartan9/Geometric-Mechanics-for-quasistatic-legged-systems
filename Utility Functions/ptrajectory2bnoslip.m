% The 2-beat gait trajectory of the no-slip quadrupedal model goes through state transitions between different contact states and this is captured by the color
% of the trajectory. When contact state is inactive, we use dark grey and when a contact state is active, we use the corresponding no-slip scalar field to
% highlight the distance between the legs.
function ptrajectory2bnoslip(ax, tau, idx_now, idx_pre, x, y, csi, csj, c_i, lW_s_i, c_j, lW_s_j, col) % [m, h] = % m, h, 
    
    % create shortened version of the trajectory and time vector
    tau = tau(idx_pre:idx_now);
    x = x(idx_pre:idx_now); y = y(idx_pre:idx_now);

    % obtain the starting and ending points for each contact state
    idx_end = [find((diff(tau) ~= 0) == 1) numel(tau)];
    idx_start = [1, idx_end(1:end-1)];

    % plot the trajectories obtained in each contact state
    for n = 1 : numel(idx_end)
        
        % find a point in the current interval to evaluate the contact state at
        tau_now = tau(floor(0.5*( idx_start(n) + idx_end(n) )));

        % plot the trajectory
        if tau_now == csi
            plot(ax, x(idx_start(n):idx_end(n)), y(idx_start(n):idx_end(n)), 'LineWidth', 2*lW_s_i, 'Color', c_i);
        elseif tau_now == csj
            plot(ax, x(idx_start(n):idx_end(n)), y(idx_start(n):idx_end(n)), 'LineWidth', 2*lW_s_j, 'Color', c_j);
        else
            plot(ax, x(idx_start(n):idx_end(n)), y(idx_start(n):idx_end(n)), 'LineWidth', 2*lW_s_j, 'Color', col);
        end
        
    end

end