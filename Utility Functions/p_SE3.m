% This function computes the translational component of an input SE(3) transformation.
function out = p_SE3(in)
    
    if iscell(in)
        out = cell(size(in));
        for i = 1:numel(in)
            if sum(size(in{i}) == [4, 4]) ~= 2
                error('ERROR! Please provide 4x4 SE(3) transformation matrices only.');
            end
            out{i} = in{i}(1:3, 4);
        end
    else
        if sum(size(in) == [4, 4]) ~= 2
            error('ERROR! Please provide 4x4 SE(3) transformation matrices only.');
        end
        out = in(1:3, 4);
    end

end