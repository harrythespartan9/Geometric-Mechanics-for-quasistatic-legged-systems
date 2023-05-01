% This function extracts the rotation matrix from the given SE(3) transformation.
function out = R_SE3(in)

    
    if iscell(in)
        out = cell(size(in));
        for i = 1:numel(in)
            if sum(size(in{i}) == [4, 4]) ~= 2
                error('ERROR! Please provide 4x4 SE(3) transformation matrices only.');
            end
            out{i} = in{i}(1:3, 1:3);
        end
    else
        if sum(size(in) == [4, 4]) ~= 2
            error('ERROR! Please provide 4x4 SE(3) transformation matrices only.');
        end
        out = in(1:3, 1:3);
    end


end