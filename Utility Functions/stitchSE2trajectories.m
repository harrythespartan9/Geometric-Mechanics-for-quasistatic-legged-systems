% This function stitches together the displacements obtained from two different paths together to get the displacement from a composite gait
function [out, z] = stitchSE2trajectories(xi, yi, thetai, zxi, zyi, zthetai, xj, yj, thetaj, zxj, zyj, zthetaj, ibefore, iduring, jduring, jafter)

    % Pullback the second displacement and add it to the first displacement
    phi_i = [xi; yi; thetai];
    phi_j = repmat( [zxi; zyi; zthetai], 1, size([xj; yj; thetaj], 2) ) + rot_SE2(zthetai)*[xj; yj; thetaj];
    phi_i_before = repmat( phi_i(:,1), 1, sum(ibefore)+1 ); phi_i_before = phi_i_before(:, 1:end-1);
    phi_i_during = interp1( 1:numel(xi), phi_i', linspace( 1, numel(xi), sum(iduring)+1 ), 'spline' )'; phi_i_during = phi_i_during(:, 1:end-1);
    phi_j_during = interp1( 1:numel(xj), phi_j', linspace( 1, numel(xj), sum(jduring)+1 ), 'spline' )'; phi_j_during = phi_j_during(:, 1:end-1);
    phi_j_after  = repmat( [zxi; zyi; zthetai] + rot_SE2(zthetai)*[zxj; zyj; zthetaj], 1, sum(jafter)+1 ); phi_j_after = phi_j_after(:, 1:end-1);
    out = [phi_i_before , ... % before csi
        phi_i_during , ... % during csi
        phi_j_during , ... % during csj
        phi_j_after]; % after csj

    % Get the overall net displacement
    z = [zxi; zyi; zthetai] + rot_SE2(zthetai)*[zxj; zyj; zthetaj]; % explicity compute the pullback

end