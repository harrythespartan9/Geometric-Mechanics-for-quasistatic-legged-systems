function out = fromSE3toSE2frames(in)
    %FROMSE3TOSE2FRAMES This function accepts SE(3) frames as input and outputs the corresponding SE(2) frames.
    %   The xy translations are maintained, but for rotation angle we average the orientations of x and y axes with the origin.
    %   Input: 1) "in" is a cell array of 4x4 SE(3) transformation matrices or should be standalone 4x4 SE(3) transformation matrix. The SE(3) matrix can be
    %   a symbolic or numeric array. If anything else is input, the function will generate an error accordingly.

    % Extract the rotational and translational components, and compute the SE(2) projection
    if iscell(in)
        out = cell(size(in));
        for i = 1:numel(in)
            if sum(size(in{i}) == [4, 4]) ~= 2 % if it is not a 4x4 matrix
                error('ERROR! The input not a 4x4 SE(3) transformation matrix.');
            else
                Rtemp = in{i}(1:2, 1:2); ptemp = in{i}(1:2, 4); % reduce it to xy directions
                out{i} = v2M_SE2([ptemp; compute_yaw2d(Rtemp)]); % estimate the SE(2) representation
            end      
        end
    else
        if sum(size(in) == [4, 4]) ~= 2 % if it is not a 4x4 matrix
            error('ERROR! The input not a 4x4 SE(3) transformation matrix.');
        else
            Rtemp = in(1:2, 1:2); ptemp = in(1:2, 4);
            out = v2M_SE2([ptemp; compute_yaw2d(Rtemp)]);
        end
    end
    

end

