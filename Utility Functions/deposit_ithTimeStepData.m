function out = deposit_ithTimeStepData(in, idx, out)
%deposit_ITHTIMESTEPDATA deposit the ith time-step data 
%   Input a column array of data to obtain an output cell array time series where each cell corresponds to a 1D timeseries trajectory in the basis directions. 
%   This function does the opposite of "retrieve_ithTimeStepData.m" function.

    if iscell(in)
        error(['ERROR! The input needs to be a timeseries column array ' ...
            'with entries corresponding to each basis direction.']);
    end
    
    for i = 1:numel(in)
        out{i}(idx) = in(i);
    end

end

