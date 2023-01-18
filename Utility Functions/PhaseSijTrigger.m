% This function triggers the ode integration to stop for level-2 no slip
% quadrupedal locomotion problems.
function [value,isterminal,direction] = PhaseSijTrigger(~,y)

% Phase of the 2-shape slice
value = atan2(y(2),y(1));

% Set that the event is when value is 0, it is terminal
isterminal = 1;

% Set the direction of approach for this value-- tried both 0 and 1, -1
% seems to work the best; the triggering seems to happen at the beginning
% and continues for a cycle. Either way, that's what I'm looking for to
% trigger after a cycle of the gait is complete!
direction = -1;

end