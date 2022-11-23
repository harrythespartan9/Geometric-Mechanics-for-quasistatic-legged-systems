% This script returns the appropriate level-2 contact submanifold functions 
% for a rigid quadrupedal robot. The contact states are organized as 
% follows,
% [12 23 34 41 13 24]
% The algorithm checks what two legs are needed and then checks the 
% corresponding location in the provided kinematic structure.
function k = returnSijfxn(k,cs1,cs2)

% Check contact state storage index in the level-2 cs matrix and return the
% index
cs_idx = find(sum((k.lvl2_cs == cs1) + (k.lvl2_cs == cs2),2) == 2);
k.cs_idx = cs_idx;

end