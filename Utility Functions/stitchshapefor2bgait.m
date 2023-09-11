function [a1, a2] = stitchshapefor2bgait(before, during, switching, after, a1A, a2A, ij)
    
    switch ij

        case 1
            
            temp_before_i = a1A(1)*ones(1, sum(before)+1); temp_before_i = temp_before_i(1:end-1);
            temp_active_i = interp1( 1:numel(a1A), a1A, linspace(1,numel(a1A), sum(during)+1 ), 'spline' ); temp_active_i = temp_active_i(1:end-1);
            % temp_switch_i = linspace( a1A(end), a1A(1), sum(switching)+1 ); temp_switch_i = temp_switch_i(1:end-1);
            temp_switch_i = interp1( 1:numel(a1A), fliplr(a1A), linspace(1,numel(a1A), sum(switching)+1 ), 'spline' ); temp_switch_i = temp_switch_i(1:end-1);
            temp_after_i = a1A(1)*ones(1, sum(after)+1); temp_after_i = temp_after_i(1:end-1);
            a1 = [ temp_before_i, temp_active_i, temp_switch_i, temp_after_i ];

            temp_before_i = a2A(1)*ones(1, sum(before)+1); temp_before_i = temp_before_i(1:end-1);
            temp_active_i = interp1( 1:numel(a2A), a2A, linspace(1,numel(a2A), sum(during)+1 ), 'spline' ); temp_active_i = temp_active_i(1:end-1);
            % temp_switch_i = linspace( a2A(end), a2A(1), sum(switching)+1 ); temp_switch_i = temp_switch_i(1:end-1);
            temp_switch_i = interp1( 1:numel(a2A), fliplr(a2A), linspace(1,numel(a2A), sum(switching)+1 ), 'spline' ); temp_switch_i = temp_switch_i(1:end-1);
            temp_after_i = a2A(1)*ones(1, sum(after)+1); temp_after_i = temp_after_i(1:end-1); % after & ~switching
            a2 = [ temp_before_i, temp_active_i, temp_switch_i, temp_after_i ];

        case 2
            
            temp_before_j = a1A(end)*ones(1, sum(before)+1); temp_before_j = temp_before_j(1:end-1);
            % temp_switch_j = linspace( a1A(end), a1A(1), sum(switching)+1 ); temp_switch_j = temp_switch_j(1:end-1);
            temp_switch_j = interp1( 1:numel(a1A), fliplr(a1A), linspace(1,numel(a1A), sum(switching)+1 ), 'spline' ); temp_switch_j = temp_switch_j(1:end-1);
            temp_active_j = interp1( 1:numel(a1A), a1A, linspace(1,numel(a1A), sum(during)+1 ), 'spline' ); temp_active_j = temp_active_j(1:end-1);
            temp_after_j = a1A(end)*ones(1, sum(after)+1); temp_after_j = temp_after_j(1:end-1);
            a1 = [ temp_before_j, temp_switch_j, temp_active_j, temp_after_j ];

            temp_before_j = a2A(end)*ones(1, sum(before)+1); temp_before_j = temp_before_j(1:end-1);
            % temp_switch_j = linspace( a2A(end), a2A(1), sum(switching)+1 ); temp_switch_j = temp_switch_j(1:end-1);
            temp_switch_j = interp1( 1:numel(a2A), fliplr(a2A), linspace(1,numel(a2A), sum(switching)+1 ), 'spline' ); temp_switch_j = temp_switch_j(1:end-1);
            temp_active_j = interp1( 1:numel(a2A), a2A, linspace(1,numel(a2A), sum(during)+1 ), 'spline' ); temp_active_j = temp_active_j(1:end-1);
            temp_after_j = a2A(end)*ones(1, sum(after)+1); temp_after_j = temp_after_j(1:end-1);
            a2 = [ temp_before_j, temp_switch_j, temp_active_j, temp_after_j ];

    end
    
end