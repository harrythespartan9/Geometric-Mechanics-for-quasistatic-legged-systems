% This function extracts the rotation matrix from the given SE(2) transformation.
function out = R_SE2(in)

    
    if iscell(in)
        out = cell(size(in));
        for i = 1:numel(in)
            if sum(size(in{i}) == [3, 3]) ~= 2
                error('ERROR! Please provide 3x3 SE(2) transformation matrices only.');
            end
            temp = in{i}(1:2, 1:2); out{i} = [temp, zeros(2, 1); zeros(1, 2), 1];
        end
    else
        if sum(size(in) == [3, 3]) ~= 2
            error('ERROR! Please provide 3x3 SE(2) transformation matrices only.');
        end
        temp = in(1:2, 1:2); out = [temp, zeros(2, 1); zeros(1, 2), 1];
    end


end