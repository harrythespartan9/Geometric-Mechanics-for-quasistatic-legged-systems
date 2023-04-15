% This function checks the condition of the input SE3 transformation matrix and returns the rotation and position components.
function [R, p] = conditionR_SE3(in, f)

    if sum(size(in) == [4, 4]) == 2 % check if the size is 4x4 as an SE3 transformation matrix should be ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                
        % Now, we check if the rotation matrix component is actually a rotation matrix:
        R = in(1:3,1:3);

        % Switch between Numeric and Symbolic cases
        switch f
        
            case 1 % Numeric case

                % Normalize this rotation matrix
                R = normalize(R, 1, 'norm');
                
                % Now, check if the columns are linearly independent by seeing if in^T*in is close to identity matrix
                if abs(norm(R'*R, 2) - 1) > 1e-5
                    error('ERROR! The rotation matrix in the SE(3) transformation is not orthogonal.');
                end

                % Check if the determinant is 1
                if abs(det(R) - 1) > 1e-5 % giving it a 1e-5 tolerance to be as close as 1 if possible
                    error('ERROR! The determinant rotation matrix in the SE(3) transformation is not close to 1.');
                end

            case 0 % Symbolic case
                
                if abs(double(simplify(norm(R'*R, 2))) - 1) > 1e-5 % we are converting the matrix 2-norm to double numeric in order to perform the comparison
                                                                   % if this is not possible; the code will still error out
                    error('ERROR! The rotation matrix in the SE(3) transformation is not orthogonal.');

                end

                if abs(double(simplify(det(R) - 1))) > 1e-5
                    error('ERROR! The determinant rotation matrix in the SE(3) transformation is not close to 1.');
                end
    
        end
    
        % Now, we are ready to return the components
        p = in(1:3, 4);
    
    else % if this is not a 4x4 transformation matrix
    
        error('ERROR! The input not a 4x4 SE(3) transformation matrix.');
    
    end


end