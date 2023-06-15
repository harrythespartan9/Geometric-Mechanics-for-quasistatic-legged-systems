function swing_lim = findswinglimits(r)
%FINDSWINGLIMITS swing limits of a quadrupedal robot from trajectory
%   Finds the swing limits given the swing time series of a quadrupedal robot. The limits are used in visualizing the reduced shape space and since we are
%   computing a single scalar limit, the

    
    switch iscell(r)

        case 0
            
            error('ERROR! Each component of the swing trajectory needs to be in a cell array.');

        case 1
            
            swing_lim = nan(numel(r), 2);
            for i = 1:numel(r)
                swing_lim(i, :) = abs([min(r{i}, [], 'all'), max(r{i}, [], 'all')]);
            end
            swing_lim = max(swing_lim, [], 'all');
    end

end

