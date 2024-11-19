function aSingConditioned = conditionSingularityValues(aSing)
%CONDITIONSINGULARITYVALUES Condition the singularity values when they 
%are very close to \pi

    % if there are any pi's, condition them (use +pi instead of -pi 
    % wherever possible)
    if any(abs(aSing) < 1.01*pi & abs(aSing) > 0.99*pi)
        ptsNow = aSing(abs(aSing) < 1.01*pi & abs(aSing) > 0/99*pi);
        signOfPtsNow = sign(ptsNow); 
        signOfPtsNow(signOfPtsNow ~= -1) = 1;
        aSingConditioned = signOfPtsNow.*aSing;
    else
        aSingConditioned = aSing;
    end

end

