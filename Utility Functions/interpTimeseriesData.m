function out = interpTimeseriesData(in)
%INTERPTIMESERIESDATA this function interpolates the given time series data
%   The input data is in the form of a timeseries cell array where each cell corresponds to the 1D timeseries trajectory along a basis direction. If it is not a
%   cell array, it is assumed to be a single 1D timeseries numeric array.

    % unpack
    tS = in{1}; t = in{2}; data = in{3}; interp_type = in{4};

    % check and compute
    switch iscell(data)

        case 0 % 1D array case

            out = interp1(t, data, tS, interp_type);

        case 1 % 1D trajectory along each direction case

            out = cell(size(data));
            for i = 1:numel(data)
                out{i} = interp1(t, data, tS, interp_type);
            end

    end

end

