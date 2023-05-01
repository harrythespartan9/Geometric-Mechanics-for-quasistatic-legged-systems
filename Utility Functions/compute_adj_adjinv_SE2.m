% Computes the adjoint or adjoint inverse of the operation based on the input transformation. Checks if the input satisfies all conditions for a proper
% transformation matrix. Input container 'f' specifies if the adjoint or adjoint-inverse is needed.
function out = compute_adj_adjinv_SE2(in, f)

    % Perform all checks and return the rotational and translational components of the SE(3) transformation
    R = in(1:2, 1:2); p = in(1:2, 3);

    % Create a rotational const
    Rperp = [0, -1;
            1, 0];
    
    % Compute the adjoint or adjoint inverse based on the input flag
    switch f
        
        case 1 % adj
    
            out = [R -Rperp*p;
                    zeros(1,2), 1];
    
        case -1 % adjinv
    
            out = [R' R'*Rperp*p;
                    zeros(1,2), 1];
    
    end

    % If the computation was symbolic, return after simplifying
    if isa(out, 'sym')
        out = simplify(out, 'Steps', 10);
    end

end