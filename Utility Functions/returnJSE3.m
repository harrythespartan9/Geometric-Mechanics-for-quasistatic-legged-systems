% function returns the full jacobian the right format.
function out = returnJSE3(J_b__i, J_ib__i, i, n)

    
    % Switch between the single output and multi output case
    switch n
        
        case 1

            out = compJSE3(J_b__i, J_ib__i, i, n); % compute

        otherwise
            
            out = cell(size(J_b__i)); % initialize
            for j = 1:numel(out) % iterate
                
                out{j} = compJSE3(J_b__i{j}, J_ib__i{j}, i{j}, n, j);
                
            end

    end


end