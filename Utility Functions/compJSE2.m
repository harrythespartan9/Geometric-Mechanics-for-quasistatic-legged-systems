% This function computes the jacobian in the right format and returns it to the upper function.
function out = compJSE2(J_b__i, J_ib__i, i, n, idx)

    
    % Make sure the jacobians are the right size.
    if sum(size(J_b__i) == [3, 3]) ~= 2
    
        if nargin < 5
            error('ERROR! The body jacobian must be a 3x3 SE(3) transform on the group vector.');
        else
            error(['ERROR! The' num2str(idx) ' body jacobian must be a 3x3 SE(3) transform on the group vector.']);
        end
    
    elseif sum(size(J_ib__i) == [3, 1]) ~= 2
    
        if nargin < 5
            error('ERROR! The hip jacobian must be a 3x1 transform on the group vector.');
        else
            error(['ERROR! The' num2str(idx) ' hip jacobian must be a 3x1 transform on the group vector.']);
        end
    
    else
        
        out = [ J_b__i, [ zeros(3, i-1), J_ib__i, zeros(3, n-i) ] ]; % compute the full Jacobian
    
    end


end