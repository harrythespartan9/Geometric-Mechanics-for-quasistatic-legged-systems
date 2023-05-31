function out = retrieve_ithTimeStepData(in, idx)
%RETRIEVE_ITHTIMESTEPDATA retrieve the ith time-step data 
%   Output a column array of data given an input cell array time series where each cell corresponds to a 1D timeseries trajectory in the basis directions. The
%   output will have the value in each direction as a column vector at time-step 'i'.

    if ~iscell(in)
        error('ERROR! The input needs to be a timeseries cell array.');
    end
    out = nan(numel(in), 1);
    for i = 1:numel(in)
        out(i) = in{i}(idx);
    end

end

