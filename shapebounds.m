% This script checks if the fitted sinusoidal gait is within the bounds of
% the corresponding shape-space slice. The input 
function [bF,tI,tV] = shapebounds(s)

% Shape-space bounds violation---------------------------------------------
    % Check the maxmium and minimum values of the sinusoid, and add 
    % a flag value for each case.
    sb = [si(3) - abs(si(1)), si(3) + abs(si(1))];
    % Make sure the whole sinusoid is not out of bounds-- check if the
    % maximum value is below the negative bound or the minimum value is
    % above the positive bound.
    if sb(1) > ank || sb(2) < -ank % out of bounds
        % If the whole waveform is out of bounds

        bF = 0;
    
    elseif (sb(1) >= -ank && sb(1) <= ank)...
            || (sb(2) >= -ank && sb(2) <= ank) % partly within bounds
        % If either the minimum or maximum value is within bounds

        bF = 1;

    else % fully valid
        % If the complete waveform is within bounds
        
        bF = 0;

    end

    % Obtain the time intervals 

end