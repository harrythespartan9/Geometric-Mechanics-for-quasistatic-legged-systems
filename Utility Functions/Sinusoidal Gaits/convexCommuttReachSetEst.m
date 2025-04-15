function appxGaitSpace = convexCommuttReachSetEst( stanceSpace )
%CONVEXCOMMUTTREACHSETEST Perform a weak Minkowski addition of stance phase
%subgaits to obtain a commutative and convex estimate of the full cycle
%reachable set

    % unpack the displacement of the points on the convex hull of the
    % subgaits in each stance space, the number of points in each of
    % those sets, and the average velocity of the subgaits
    z = stanceSpace.zh; n=stanceSpace.n;

    % based on the number of points in the hull of each subgait, we grid 2D 
    % grid those points to obtain all possible combintations of the hull
    % subgaits (Minkowski-ish addition)
    [idx1, idx2] = meshgrid( 1:n(1), 1:n(2) ); 
    idx1 = idx1(:); idx2 = idx2(:); % ... flatten these grids
    % ... augment the z arrays and average subgait velocity arrays into 
    % ... their Minkwoski-ish (augmented) form
    zAug1 = z{1}(idx1, :); zAug2 = z{2}(idx2, :);
    % ... stitch these displacement together to obtain the displacement of
    % ... the total gait cycle
    zAug = stitchTwoSE2displacements( zAug1, zAug2 );

    % ... perform the log operation to obtain the corresponding Lie algebra
    % ... elements
    gCA_Aug = logMapOfALieGroupElement( zAug );

    % pack everything and return
    appxGaitSpace = [];
    appxGaitSpace.z     = zAug;
    appxGaitSpace.gCA   = gCA_Aug;
    
end

