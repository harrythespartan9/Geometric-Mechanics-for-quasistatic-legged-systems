% This script computes the center of a bound-box where you have set of vectors forming a closed shape.
function out = boundboxcenter(in)

    % compute the transformation to the center
    out = -mean(cumsum(in, 1));

end