function r = interpswingSE2(he, fh, in)
    
    % Initialize
    r = nan(3, 10);
    
    % Swing value of interest
    rt = linspace(0, in{3}, 10);

    % Compute the foot tip at the interpolated swing values
    for i = 1:10
        r(:, i) = seqSE2transformation( [he, fh(in{1}, in{2}, rt(i))] );
    end

end