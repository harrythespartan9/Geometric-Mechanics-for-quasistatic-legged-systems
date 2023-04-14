% This function computes the SE(3) rotational transf matrix 'out' from the current frame's Z axis by a given angle 'in'. 
% Works on both numeric and symbolic inputs.
function out = rotz_SE3(in)
    
    switch numel(in)
        
        case 0

            error('ERROR! There should be atleast 1 input angle.');

        case 1
    
            out = [cos(in), -sin(in), 0, 0;
                   sin(in), cos(in), 0, 0;
                   0, 0, 1, 0;
                   zeros(1,3), 1];

        otherwise
            
            out = cell(size(in)); % initialize
            for i = 1:numel(in)
                out{i} = [cos(in(i)), -sin(in(i)), 0, 0;
                          sin(in(i)), cos(in(i)), 0, 0;
                          0, 0, 1, 0;
                          zeros(1,3), 1]; % iteratively compute
            end
            out = reshape(out, size(in));

    end

end