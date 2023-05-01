% This function computes the full jacobian from the jacobian about the body and hip joints provided. The goal is to add a column of zeros for hip joints that
% don't directly control the corresponding leg.
function Jout = computefulljacobianSE2(J_b__i, J_ib__i, n)

    % enumerate legs 'i'
    i = num2cell(1:numel(J_b__i));
    
    % Make sure everything is in the correct format and then proceed to a function that puts together the full jacobian.
    if iscell(i)
        
        if numel(i) > 1
        
            if iscell(J_b__i) && iscell(J_ib__i)
                
                if numel(J_b__i) == numel(J_ib__i)
                    
                    Jout = returnJSE2(J_b__i, J_ib__i, i, n); % compute each full jacobian requested
    
                else

                    error('ERROR! The number of entries in the input jacobians should be equal.');

                end
    
            else

                error('ERROR! Both the input jacobians should be cell arrays if multiple jacobians are needed.');

            end
    
        end

    else
        
        if numel(i) > 1

            error('ERROR! If you want evaluate more than one entry, please give all inputs as a cell array.')
            
        elseif numel(i) == 0

            error('ERROR! You need to specify the position of the leg to get the right pfaffian constraint.');

        else

            if iscell(J_b__i) || iscell(J_ib__i)
                
                error('ERROR! If you need a single output, then jacobains need to 4x4 SE3 transforms (numeric or symbolic).');

            else
                
                Jout = returnJSE2(J_b__i, J_ib__i, i, n); % compute the single jacobian requested

            end

        end

    end

end