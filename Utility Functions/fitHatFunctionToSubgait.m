function hatOut = fitHatFunctionToSubgait(tau, subgait)
%FITHATFUNCTIONTOSUBGAIT fit a Hat function to the subgait provided
%   The hat function is composed of two ramps from the starting point to
%   the midpoint (end point = starting point) and backwards. The tau input
%   is the phase (0 to 2\pi) associated with the periodic waveforms in the
%   'subgait' inputs columns.
    
    % get the midpoint at tau = pi and starting point at tau = 0
    zeroPt = subgait(1, :);
    piPt = interp1(tau, subgait, pi, "pchip");

    % check number of points in the discretization-- if even do it a
    % certain way, else do it another way
    hatOut = nan(size(subgait));

    % get the indices for the first and sedond ramp
    firstRampIdx = tau < pi; secondRampIdx = ~firstRampIdx;
    
    % create the upward ramp in first half and downward ramp in the second
    hatOut(firstRampIdx, :) = repmat(zeroPt, nnz(firstRampIdx), 1) ...
        + repmat(piPt - zeroPt, nnz(firstRampIdx), 1)...
                        .*repmat(tau(firstRampIdx), 1, 2)/pi;
    hatOut(secondRampIdx, :) = repmat(piPt, nnz(secondRampIdx), 1) ...
        - repmat(piPt - zeroPt, nnz(secondRampIdx), 1)...
                        .*(repmat(tau(secondRampIdx), 1, 2) - pi)/pi;

end
