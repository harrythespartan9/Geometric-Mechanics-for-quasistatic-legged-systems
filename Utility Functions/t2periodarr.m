% based on the time periods of a gait, this function computes the starting time of each of these periods in a cumulative sum.
function outarr = t2periodarr(inarr)
    
    % initialize the output
    outarr = zeros(size(inarr));

    % iterate from second term
    for c = 2:numel(inarr)
        
        % add everything till the previous term
        outarr(c) = sum(inarr(1:c-1));

    end

end