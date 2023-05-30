function out = splitTo1Dtimeseries(in)
%SPLITTO1DTIMESERIES splits in the input cell array collection into 1D time series traces
%   Given a collection of cell arrays, this function splits each corresponding element into a collection of 1D time series array cells. There are no concistency
%   checks performed in this function. No concistency checks are made! Ensure that the 

    % initialize, iterate, and compute
    trace_num = numel(in{1});
    out = cell(trace_num, 1);
    for i = 1:trace_num
        temp = cell(1, numel(in)); % initialize empty container
        for j = 1:numel(in)
            temp{j} = in{j}{i};
        end
        out{i} = temp;
    end


end

