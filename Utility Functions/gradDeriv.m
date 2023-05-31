function in_grad_dot = gradDeriv(in, dt)
%FWDDIFFDERIV Compute the gradient-based derivative for the input time series.
%   Given a time vector and input time series, this function computes and returns the gradient-based time-derivative 'in_dot'. Since this is acausal,
%   this can only be used in the post-processing stage because of its "1st Order, ACAUSAL" nature. For more details check "doc gradient".
    if iscell(in)
        in_grad_dot = cell(size(in));
        for i = 1:numel(in)
            in_grad_dot{i} = gradient(in{i}, dt);
        end
    else
        in_grad_dot = gradient(in, dt);
    end
    
end