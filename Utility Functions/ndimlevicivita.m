% This function computes the n-dimensional levi-civita symbol for a given permutation. Input a row vector with each term containing the permutations you are
% interested in.
function out = ndimlevicivita(in)
    
    n = size(in, 2);
    inc_ctr = 1;               % initialize current index counter
    out = 1;                   % initialize output levi-civita symbol
    
    while inc_ctr < n          % product
        
        out = out* prod(sign( in(inc_ctr+1:end) - in(inc_ctr) ));
        inc_ctr = inc_ctr + 1;
    
    end

end