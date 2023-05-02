function Yout = compute_yaw2d(Rin)
%COMPUTE_YAW2D this function computes the approximated YAW angle for a given 3x3 rotation matrix.
%   Since the 3x3 rotation matrix encodes 3D rotations, we only use the first and second columns to compute the average yaw assuming Z
%   is still out of plane. No input concistency checks are done in this function, so please refer "fromSE3toSE2frames.m" function for more details.
    cosY = (Rin(1, 1)/norm(Rin(:, 1)) + Rin(2, 2)/norm(Rin(:, 2)))/2; 
    sinY = (Rin(2, 1)/norm(Rin(:, 1)) - Rin(1, 2)/norm(Rin(:, 2)))/2; % averaged using both columns
    Yout = atan(sinY/cosY);
end

