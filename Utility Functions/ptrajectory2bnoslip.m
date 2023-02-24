% The 2-beat gait trajectory of the no-slip quadrupedal model goes through state transitions between different contact states and this is capture by the color
% of the trajectory. When contact state is inactive, we use dark grey and when a contact state is active, we use the corresponding no-slip scalar field to
% highlight the distance between the legs.
function [m, h] = ptrajectory2bnoslip(m, h, ax, t, tau, T, j, x, y, csi, csj, c_i, lW_s_i, c_j, lW_s_j, col)

    if T(j) ~= 0 % if this is not the first time-step, then proceed

        
        % Find the previous and current indices in the main time vector between which we need to construct a trajectory. The assumption here is that, we shall 
        % not more than one occurrence of the each contact state.
        idx_now = find( t == T(j) );
        if j ~= 2 % if this is not the second time-step
            idx_pre  = find( t == T(j - 1) ) - 1;
        else % if it is the second time-step
            idx_pre  = find( t == T(j - 1) );
        end
    
        % create shortened version of the trajectory and time vector
        tau = tau(idx_pre:idx_now);
        x = x(idx_pre:idx_now); y = y(idx_pre:idx_now);
    
        % the contact states
        cs = unique(tau, 'stable'); % preserves the order
    
        % let's iterate over these states
        for n = 1 : numel(cs)
    
            % if this is the first state, or another state
            if n == 1
                % find the starting point for the cs
                idx_start = find(tau == cs(n), 'first');
            else
                idx_start = find(tau == cs(n), 'first') - 1; % stitch with the last point in previous state
                
            end
            idx_end   = find(tau == cs(n), 'last'); % the ending point for the state.
            
            % iterate the plot count
            m = m+1;
    
            % plot the trajectory
            if cs(n) == csi
                h{m} = plot(ax, x(idx_start:idx_end), y(idx_start:idx_end), 'LineWidth', 2*lW_s_i, 'Color', c_i);
            elseif cs(n) == csj
                h{m} = plot(ax, x(idx_start:idx_end), y(idx_start:idx_end), 'LineWidth', 2*lW_s_j, 'Color', c_j);
            else
                h{m} = plot(ax, x(idx_start:idx_end), y(idx_start:idx_end), 'LineWidth', 2*lW_s_j, 'Color', col);
            end
            
        end


    end

end