% This function sorts an input array's first column and then entire rows are sorted based on that.
function outA = sortfirstcolumn(inA)
    
    outA = inA;                         % initialize output array as the input arrat
        
    [~, I] = sort(outA(:, 1));          % compute the sorted indicies of the ith column

    outA = outA(I, :);                  % rearrange the rows into the sorted version of the current column

end