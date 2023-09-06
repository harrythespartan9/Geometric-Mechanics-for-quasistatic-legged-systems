% This function computes the SE(2) pullback derivative matrix-- a function of the yaw angle in `in' (given as a single angle or a set of angles). 
% Works on both numeric and symbolic inputs.
function out = rot_deriv_SE2(in)
    
    switch numel(in)
        
        case 0

            error('ERROR! There should be atleast 1 input angle.');

        case 1
    
            out = [-sin(in), -cos(in), 0
                    cos(in), -sin(in), 0
                          0,        0, 0];

        otherwise
            
            out = cell(size(in));
            for i = 1:numel(in)
                out{i} = [-sin(in), -cos(in), 0
                           cos(in), -sin(in), 0
                                 0,        0, 0];
            end
            out = reshape(out, size(in));
            
    end

end