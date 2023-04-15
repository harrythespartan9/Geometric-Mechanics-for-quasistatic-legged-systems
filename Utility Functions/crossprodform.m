% This function computes the cross product form of an input vector or array-- can be numeric or symbolic. Can have any dimension order as long as there exists a
% dimension that is 3.
function out = crossprodform(in)
    
    
    % check if there is at least one dimension with size 3
    if sum(size(in) == 3) == 0
        error('ERROR! There should be atleast one dimension with size 3.');
    end

    % reorder the data with the size 3 dimension (the first one) at the beginning, and return the reordered input with the out
    ins = size(in); in3idx = find(ins == 3, 1, "first"); listins = 1:numel(ins);
    in = permute(in, [in3idx, listins(listins ~= in3idx)]);
    numelin = prod(ins)/ins(in3idx);

    % compute the translational SE(3) transformation
    switch numelin

        case 1

            out = [0, -in(3), in(2);
                   in(3), 0, -in(1);
                   -in(2), in(1), 0];

        case 0

            out = cell(1, numelin); % initialize

            for i = 1:numelin % iterate
            
                out{i} = [0, -in(3, i), in(2, i); % compute
                       in(3, i), 0, -in(1, i);
                       -in(2, i), in(1, i), 0];

            end

    end

    % out array formatting
    if iscell(out) % if the output is a cell, condition it.

        switch ~isempty(ins(in3idx+1:numel(ins)))
        
            case 0 % if empty
    
                out = reshape(out, [ins(1:in3idx-1), 1]);
    
            case 1 % not empty
    
                out = reshape(out, [ins(1:in3idx-1), ins(in3idx+1:numel(ins))]);
    
        end

    end

    
end