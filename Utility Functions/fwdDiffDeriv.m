function in_dot = fwdDiffDeriv(in, t)
%FWDDIFFDERIV Compute the fwd-difference derivative for the input time series.
%   Given a time vector and input time series, this function computes and returns the forward difference based time-derivative 'in_dot'. Since this is acausal,
%   this can only be used in the post-processing stage-- can't construct a DT filter with this.
    if iscell(in)
        in_dot = cell(size(in));
        for i = 1:numel(in)
            if numel(in{i}) ~= numel(t)
                error('ERROR! The input time series has a different length from the input time vector.');
            end
            in_dot{i} = [diff(in{i})./diff(t) nan];
        end
    else
        if numel(in) ~= numel(t)
            error('ERROR! The input time series has a different length from the input time vector.');
        else
            in_dot = [diff(in)./diff(t) nan];
        end
    end
end

