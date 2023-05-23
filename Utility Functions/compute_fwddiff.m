function xdot = compute_fwddiff(t, x)
%COMPUTE_FWDDIFF This function computes a first order, forward difference based time derivative.
%   We compute the time derivate of an input function by taking the first order forward difference. The derivative corresponding to the last time step will be
%   NaN.
    
    % condition the input
    if iscell(x)
        for i = 1:numel(x)
            if numel(x{i}) ~= numel(t)
                error(['ERROR! The time vector and ' num2str(i) 'th input array should be equally long.']);
            end
        end
    else
        if numel(x) ~= numel(t)
            error('ERROR! The time vector and input array should be equally long.')
        end
    end

    % Compute
    if iscell(x)
        xdot = cell(size(x));
        for i = 1:numel(x)
            xdot{i} = diff(x{i})./diff(t); xdot{i}(end+1) = nan;
        end
    else
        xdot = diff(x)./diff(t); xdot(end+1) = nan;
    end

end

