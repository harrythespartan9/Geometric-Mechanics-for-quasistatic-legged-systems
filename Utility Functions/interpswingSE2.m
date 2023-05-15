function r = interpswingSE2(he, fh, in)
    
    % Initialize
    r = nan(2, 10);
    
    % Swing value of interest
    rt = linspace(0, in{3}, 10);

    % Compute the foot tip at the interpolated swing values
    for i = 1:10
        r(:, i) = [eye(2, 2), zeros(2, 1)]*seqSE2transformation( [he, fh(in{1}, in{2}, rt(i))] );
    end

end