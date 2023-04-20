% This function sorts an input array left to right columnwise for generating well-conditioned contact states.
function outA = columnwiseL2Rsort(inA)
    
    outA = inA;                         % initialize output array as the input arrat

    for i = 1:size(outA, 2)             % iterate
        
        [~, I] = sort(outA(:, i));      % compute the sorted indicies of the ith column

        outA = outA(I, :);              % rearrange the rows into the sorted version of the current column

    end

end