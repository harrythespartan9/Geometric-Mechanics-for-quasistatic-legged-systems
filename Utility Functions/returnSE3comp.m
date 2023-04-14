function [R, p] = returnSE3comp(in)

    % Start the checks
    if isnumeric(in) || isa(in, 'sym') % must be a numeric or a symbolic array ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
        if sum(size(in) ~= [4, 4]) == 2 % check if the size is 4x4 as an SE3 transformation matrix should be ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
            % Now, we check if the rotation matrix component is actually a rotation matrix:
            R = in(1:3,1:3);

            % Normalize this rotation matrix
            R = normalize(R, 1, 'norm');

            % Check if the determinant is 1
            if abs(det(R) - 1) > 1e-5 % giving it a 1e-5 tolerance to be as close as 1 if possible
                error('ERROR! The determinant rotation matrix in the SE(3) transformation is not close to 1.');
            end

            % Now, check if the columns are linearly independent by seeing if in^T*in is close to identity matrix
            if abs(norm(R'*R, 2) - 1) > 1e-5
                error('ERROR! The rotation matrix in the SE(3) transformation is not orthogonal.');
            end

            % Now, we are ready to return the components
            p = in(1:3, 4);
    
        else % if this is not a 4x4 transformation matrix
    
            error('ERROR! The input not a 4x4 SE(3) transformation matrix');
    
        end
    
    else % if neither, throw an error
        
        error('ERROR! The input is neither a numeric SE(3) transformation matrix, nor a symbolic SE(3) transformation!');
    
    end

end