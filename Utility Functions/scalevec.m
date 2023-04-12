% This script finds a scaling factor for vectors forming a closed shape based on the mean lengths of each side.
function out = scalevec(in)

    % lengths of the vectors
    tempL = vecnorm(in, 2, 2);
    
    % mean
    meanbox = mean(tempL);
    
    % scaling factor
    out = meanbox./tempL;

end