% This function computes the SE(2) rotational matrix (about the out of plane z) by a given angle/set of angles 'in'. 
% Works on both numeric and symbolic inputs.
function out = rot_SE2(in)
    
    switch numel(in)
        
        case 0

            error('ERROR! There should be atleast 1 input angle.');

        case 1
    
            out = [cos(in), -sin(in), 0;
                   sin(in), cos(in), 0;
                   0, 0, 1];

        otherwise
            
            out = cell(size(in));
            for i = 1:numel(in)
                out{i} = [cos(in(i)), -sin(in(i)), 0;
                          sin(in(i)), cos(in(i)), 0;
                          0, 0, 1];
            end
            out = reshape(out, size(in));

    end

end