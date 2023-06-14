function swing_lim = findswinglimits(r)
%FINDSWINGLIMITS swing limits of a quadrupedal robot from trajectory
%   Finds the swing limits given the swing time series of a quadrupedal robot. The limits are used in visualizing the reduced shape space and since we are
%   computing a single scalar limit, the

    
    switch iscell(r)

        case 0
            
            error('ERROR! Each component of the swing trajectory needs to be in a cell array.');

        case 1

            for i = 1:numel(r)
                switch i
                    case 1
                        swing_lim = max(abs([max(r{i}, [], 'all'), min(r{i}, [], 'all')]));
                    otherwise
                        temp_lim = max(abs([max(r{i}, [], 'all'), min(r{i}, [], 'all')]));
                        if swing_lim > temp_lim
                            swing_lim = temp_lim;
                        end
                end
            end
        
    end

end

