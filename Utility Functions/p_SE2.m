% This function computes the translational component of an input SE(2) transformation.
function out = p_SE2(in)
    
    if iscell(in)
        out = cell(size(in));
        for i = 1:numel(in)
            if sum(size(in{i}) == [3, 3]) ~= 2
                error('ERROR! Please provide 3x3 SE(3) transformation matrices only.');
            end
            out{i} = in{i}(1:2, 3);
        end
    else
        if sum(size(in) == [3, 3]) ~= 2
            error('ERROR! Please provide 3x3 SE(3) transformation matrices only.');
        end
        out = in(1:2, 3);
    end

end