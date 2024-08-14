function [g, z] = stitchTwoSE2trajectories(g1, z1, g2, z2)
%STITCHTWOSE2TRAJECTORIES Stitches two SE(2) trajectories
%   Given two SE(2) trajectories, this function stitches them together with
%   the first trajectory leading the second one.

    % input conditioning
    z1 = z1(:); z2 = z2(:);

    % compute the stitched trajectory
    g = [ g1(1:end-1, :);
        (repmat(z1, 1, size(g2, 1)) + rot_SE2(z1(3))*g2')' ];
    z = z1 + rot_SE2(z1(3))*z2;

end

