function in_dot = specderiv(in, t)
%SPECDERIV fourier/spectral derivative of an input time series
%   Given an input time series and time vector, the spectral/fourier derivative is computed and returned.
    if iscell(in)
        in_dot = cell(size(in));
        for i = 1:numel(in)
            if numel(in{i}) ~= numel(t)
                error('ERROR! The input time series has a different length from the input time vector.');
            end
        end
    else
        if numel(in) ~= numel(t)
            error('ERROR! The input time series has a different length from the input time vector.');
        else
        end
    end
end

