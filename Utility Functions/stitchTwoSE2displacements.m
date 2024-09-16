function z = stitchTwoSE2displacements(z1, z2)
%STITCHTWOSE2DISPLACEMENTS Stitches two SE(2) displacements
%   Given two SE(2) displacements, this function operates the first group 
%   element on the second. Wont work if z1 and z2 are not of the same size.
    % condition the inputs to the correct format
    z1 = formatDisplacementVector(z1);
    z2 = formatDisplacementVector(z2);
    % ensure that they have the same size
    if size(z1, 1) ~= size(z2, 1)
        error(['ERROR! The number of points in both displacement vectors ' ...
            'should be equal.']);
    end
    nPoints = size(z1, 1);
    % compute the overall displacement by stitching z2 on z1
    switch nPoints
        case 1
            z1 = z1(:); z2 = z2(:);
            z = reshape(z1 + rot_SE2(z1(3))*z2, size(z1));
        otherwise
            % ... convert the displacements to row vectors
            z1 = z1'; z2 = z2'; % points along rows now
            % ... compute the stitched displacement
            z1Pages = permute(z1, [1, 3, 2]); % points along pages now
            z2Pages = permute(z2, [1, 3, 2]);
            R1pages = rotSE2local( ... % rotation matrix pages of z1
                reshape(z1(3, :, :), 1, 1, nPoints) ...
                );
            zPages = z1Pages + pagemtimes(R1pages, z2Pages); % net disp.
            % ... convert to original format
            z = permute(zPages, [1, 3, 2])';
    end
end

%% AUXILIARY FUNCTIONS
% format the displacement vector and return
function zTransposed = formatDisplacementVector(z)
    if any(size(z) == 3)
        if ndims(z) > 3
            error(['ERROR! The displacement vector can not have more than 2 ' ...
                'dimensions.']);
        end
        if size(z, 1) == 3
            zTransposed = z';
        else
            zTransposed = z;
        end
    else
        error(['ERROR! The displacement vector should have three components ' ...
            'as either columns or rows.']);
    end
end
% compute the rotation matrix from the SE(2) displacement vector
% ... if pages of z are provided, this should return in the same format
function R = rotSE2local(zTh)
    R = [        cos(zTh),        -sin(zTh), zeros(size(zTh));
                 sin(zTh),         cos(zTh), zeros(size(zTh));
         zeros(size(zTh)), zeros(size(zTh)), ones(size(zTh))];
end