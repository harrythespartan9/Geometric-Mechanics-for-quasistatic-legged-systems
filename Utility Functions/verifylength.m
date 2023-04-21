% This function verifies if every array entry provided in a cell array are of equal length-- this is useful when analyzing discretized trajectory data.
function verifylength(a)
    for i = 1:numel(a)
        if i == 1
            num = numel(a{i});
        else
            if num ~= numel(a{i}) % if the number trajectory path discretizations aren't equal
                error('ERROR! The trajectory discretization in each direction should be the same.');
            end
        end
    end
end