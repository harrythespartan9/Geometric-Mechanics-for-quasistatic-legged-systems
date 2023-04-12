% Given the bounding box in the form of a collection of closed cyclic vectors, this function computes the transformation from the center of the box to each
% frame. The vectors are defined in a global rest frame and the body frame computed here is aligned with this frame.
function out = boundbox2ht3transform(in)
    
    % initialize the output container
    out = nan(size(in));
    
    % Iterate over the initial condition of each vector that forms a vertex in the closed shape formed by these vectors.
    for i = 1:size(in, 1)
        
        % Compute for the ith vertex requires us to do a circular shift to the ith vertex in our cyclic vector array
        inV = wshift(1, in, i-1); % if it is the first iteration, the vertex is already here, so don't shift, else shift by i-1.

        % Compute the center of the box from the first vertex
        out(i, :) = boundboxcenter(inV);
        
    end

end