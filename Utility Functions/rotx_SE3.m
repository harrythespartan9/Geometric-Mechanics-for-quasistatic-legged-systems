% This function computes the SE(3) rotational transf matrix 'out' the current frame's X axis by a given angle 'in'. 
% Works on both numeric and symbolic inputs.
function out = rotx_SE3(in)
    
    switch numel(in)
        
        case 0

            error('ERROR! There should be atleast 1 input angle.');

        case 1
    
            out = [1, 0, 0, 0;
                   0, cos(in), sin(in), 0;
                   0, -sin(in), cos(in), 0;
                   zeros(1,3), 1];

        otherwise
            
            out = cell(size(in));
            for i = 1:numel(in)
                out{i} = [1, 0, 0, 0;
                          0, cos(in(i)), sin(in(i)), 0;
                          0, -sin(in(i)), cos(in(i)), 0;
                          zeros(1,3), 1];
            end
            out = reshape(out, size(in));

    end

end