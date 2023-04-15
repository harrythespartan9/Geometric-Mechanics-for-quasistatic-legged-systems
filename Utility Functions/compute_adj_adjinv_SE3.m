% Computes the adjoint or adjoint inverse of the operation based on the input transformation. Checks if the input satisfies all conditions for a proper
% transformation matrix. Input container 'f' specifies if the adjoint or adjoint-inverse is needed.
function out = compute_adj_adjinv_SE3(in, f)

    % Perform all checks and return the rotational and translational components of the SE(3) transformation
    [R, p] = returnSE3vars(in);
    
    % Compute the adjoint or adjoint inverse based on the input flag
    switch f
        
        case 1
    
            out = [R, zeros(size(R));
                crossprodform(p)*R, R];
    
        case -1
    
            out = [R', zeros(size(R));
                -R'*crossprodform(p), R'];
    
    end

    % If the computation was symbolic, return after simplifying
    if isa(out, 'sym')
        out = simplify(out, 'Steps', 10);
    end

end