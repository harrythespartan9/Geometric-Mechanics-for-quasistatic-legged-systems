% This script checks if the fitted sinusoidal gait is within the bounds of
% the corresponding shape-space slice. The input s is a 1x3 tuple 
function [bF,tI] = shapebounds(s,ank)

% Shape-space bounds violation---------------------------------------------
    % Check the maxmium and minimum values of the sinusoid, and add 
    % a flag value for each case-------------------------------------------
    sb = [s(3) - abs(s(1)), s(3) + abs(s(1))];
    % Make sure the whole sinusoid is not out of bounds-- check if the
    % maximum value is below the negative bound or the minimum value is
    % above the positive bound.
    if sb(1) > ank || sb(2) < -ank % out of bounds or invalid
        % If the whole waveform is out of bounds

        bF = 0;
    
    elseif (sb(1) >= -ank && sb(1) <= ank)...
            || (sb(2) >= -ank && sb(2) <= ank) % partly within bounds
        % If either the minimum or maximum value is within bounds

        bF = 1;

        % Violation information
        upper_violation = sb(2) > ank;
        lower_violation = sb(1) < -ank;

    else % fully valid
        % If the complete waveform is within bounds
        
        bF = 0;

    end

    % Obtain the time intervals--------------------------------------------
    switch bF
        case 0 % invalid gait
            
            tI = [0 1]; % just one interval

        case 1 % partly valid gait
            
            if upper_violation
                tu1 = (1/(2*pi))*asin((ank - s(3))/s(1)) - 1/s(2); % solution 1
                if tu1 < 0
                    tu1 = tu1 + 1;
                end
                tu2 = (1/(2*pi))*asin((-ank + s(3))/s(1)) - 1/s(2); % solution 2
            end

            if lower_violation
                tl1 = (1/(2*pi))*asin((-ank - s(3))/s(1)) - 1/s(2);
                if tl1 < 0
                    tl1 = tl1 + 1;
                end
                tl2 = (1/(2*pi))*asin((ank + s(3))/s(1)) - 1/s(2);
            end
            
            tI = unique([0, tu1, tu2, tl1, tl2, 1]); % time intervals

        case 2 % fully valid gait

            tI = [0, 1];
    end

end