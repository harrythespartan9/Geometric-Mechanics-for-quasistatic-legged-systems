function out = SE2_vec2mat(in)
%SE2_VEC2MAT Converts the SE(2) transformation from vector form to matrix form.
%   The input SE(2) transformation is in it's 3-element, vector form where the first two elements are the in-plane translation and the final element is the yaw
%   rotation. With this as the input, this function computes the SE(2) matrix transformation form.
    
    switch iscell(in)
        case 0
            if sum(size(in) == 3) == 0
                error('ERROR! There should be atleast one dimension with size 3.');
            end
            if numel(in) > 3
                error('ERROR! If there are multiple inputs, they need to be provided in the form of a cell array.');
            end
            out = v2M_SE2(in);
        case 1
            
            % if it is in a SE(2) numeric trajectory format, convert it to a 1xt cell trajectory
            if sum(size(in) == [3, 1]) == 2
                in = convertToSE2vec(in);
            end

            out = cell(size(in));
            for i = 1:numel(in)
                if sum(size(in{i}) == 3) == 0
                    error(['ERROR!', ' In the cell ', num2str(i),...
                        ' of the input, there should be atleast one dimension with size 3.']);
                end
                out{i} = v2M_SE2(in{i});
            end
    end

end