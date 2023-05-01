% function returns the full jacobian the right format.
function out = returnJSE2(J_b__i, J_ib__i, i, n)

    
    % Switch between the single output and multi output case
    switch numel(J_b__i)
        
        case 1

            out = compJSE2(J_b__i, J_ib__i, i, n); % compute

        otherwise
            
            out = cell(size(J_b__i)); % initialize, iterate, and compute
            for j = 1:numel(out)
                out{j} = compJSE2(J_b__i{j}, J_ib__i{j}, i{j}, n, j);
            end

    end


end