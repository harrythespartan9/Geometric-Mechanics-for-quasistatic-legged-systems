function out = SE2_mat2vec(in)
%SE2_MAT2VEC Converts the SE(2) transformation from matrix form to vector form.
%   The input SE(2) transformation will be converted to it's 3-element, vector form where the first two elements are the in-plane translation and the final 
%   element is the yaw rotation. From the matrix form, this function computes the SE(2) vector form.
    
    switch iscell(in)
        case 0
            if sum(size(in) == [3, 3]) ~= 2
                error('ERROR! No cell array input can only be a 3x3 SE(2) transformation matrix.');
            end
            out = M2v_SE2(in);
        case 1

            % set a 'symFlag' to see if each entry is a symbol or numeric matrix
            symFlag = false;

            % iterate and compute
            out = cell(size(in));
            for i = 1:numel(in)
                if sum(size(in{i}) == [3,3]) ~= 2
                    error(['ERROR!', ' In the cell ', num2str(i),...
                        ' of the input, there should be atleast one dimension with size 3.']);
                end
                out{i} = M2v_SE2(in{i});
                if ~symFlag
                    if isa(out{i}, 'sym')
                        symFlag = true;
                    end
                end
            end

            % convert the output back to the 3x1 cell, numeric trajectory format
            switch symFlag
                case 1
                    out = mat2cell(cell2sym(out), ones(numel(out{1}), 1), numel(out));
                case 0
                    out = mat2cell(cell2mat(out), ones(numel(out{1}), 1), numel(out));
            end

    end

end