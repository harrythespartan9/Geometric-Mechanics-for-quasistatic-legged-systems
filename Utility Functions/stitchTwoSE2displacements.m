function z = stitchTwoSE2displacements(z1, z2)
%STITCHTWOSE2DISPLACEMENTS Stitches two SE(2) displacements
%   Given two SE(2) displacements, this function operates the first group 
%   element on the second. Wont work if z1 and z2 are not of the same size.

    z1 = z1(:); z2 = z2(:);
    z = reshape(z1 + rot_SE2(z1(3))*z2, size(z1));

end

