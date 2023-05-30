function shape_vel_proj = projShapeVel2NoSlipVel(shape_vel, noslip_vel)
%PROJECTSHAPEVEL2NOSLIPVEL projects the given shape velocity onto a no-slip velocity
%   Given two velocity vectors-- shape and no-slip velocities, find the projection of the shape onto the noslip basis. Both vectors are assumed to be non-unit
%   vectors and hence the appropriate normalization will be carried out.
    
    norm_noslip = vecnorm(noslip_vel, 2, 1); % norm of noslip_vel vector time series 2xt
    unit_noslip = noslip_vel./norm_noslip; % unit vector along noslip_vel 2xt
    
    shape_vel_proj = sum( shape_vel.*unit_noslip, 1 ) .* unit_noslip; % projection

end

