function out = convertToSE2vec(in)
%CONVERTTOSE2VEC converts the input SE(2) trajectory (a 3x1 cell array with each cell containing a 1xt numeric array) from numeric to 1xt cell array format

    switch iscell(in)
        case 0
            error('ERROR! The SE(2) trajectory needs to be a 3x1 cell array with each cell containing a 1xt numeric array.');
        case 1
            if sum(size(in) == [3, 1]) ~= 2 % if it is not a 3x1 cell array {x, y, \theta}'
                error('ERROR! The SE(2) trajectory needs to be a 3x1 cell array with each cell containing a 1xt numeric array.');
            end
            
            % convert to a 1xt cell array
            out = mat2cell(cell2mat(in), 3, ones(1, size(cell2mat(in), 2)));
    end

end

