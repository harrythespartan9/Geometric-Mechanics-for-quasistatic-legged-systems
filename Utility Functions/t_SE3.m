% This function computes the translational SE(3) transformation from the current SE(3) frame. It is better if the 3-element vector's first dimension is 3-- 
% else this transformation is done on the input array. Works on both numeric and symbolic inputs.
function out = t_SE3(in)

    
    % check if there is at least one dimension with size 3
    if sum(size(in) == 3) == 0
        error('ERROR! There should be atleast one dimension with size 3.');
    end

    % reorder the data with the size 3 dimension (the first one) at the beginning, and return the reordered input with the out
    ins = size(in); in3idx = find(ins == 3, 1, "first"); listins = 1:numel(ins);
    in = permute(in, [in3idx, listins(listins ~= in3idx)]);
    numelin = prod(ins)/ins(in3idx);

    % compute the translational transformation
    switch numelin

        case 1

            out = [eye(3,3), in;
              zeros(1,3), 1];

        otherwise
            
            % initialize
            out = cell(1, numelin);

            % iterate and compute the translational SE(3) transformation
            for i = 1:numelin
                
                out{i} = [eye(3,3), in(:,i); % compute
                        zeros(1,3), 1];

            end

    end

        

    % out array formatting
    if iscell(out) % if the output is a cell, condition it.

        switch ~isempty(ins(in3idx+1:numel(ins)))
        
            case 0 % if empty (or if 2D)
    
                out = reshape(out, [ins(1:in3idx-1), 1]);
    
            case 1 % not empty
    
                out = reshape(out, [ins(1:in3idx-1), ins(in3idx+1:numel(ins))]);
    
        end

    end


end