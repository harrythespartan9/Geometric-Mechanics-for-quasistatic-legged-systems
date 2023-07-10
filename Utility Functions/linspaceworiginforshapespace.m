% This script is like the linspace function, but includes the origin in the
% distribution of points returned.
function [X,Y] = linspaceworiginforshapespace(dnum, ank)

    switch numel(ank)
        case 1
            temp_left = linspace(-ank,0,dnum-ceil(dnum/2)+1);  temp_right = linspace(0,ank,ceil(dnum/2));
            X = [temp_left(1:end-1), temp_right]; Y = X;
        case 2
            temp_left = linspace(-ank(1),0,dnum-ceil(dnum/2)+1);  temp_right = linspace(0,ank(2),ceil(dnum/2));
            X = [temp_left(1:end-1), temp_right]; Y = X;
        otherwise
            error("ERROR! The 'ank' input needs to a single entry or two-entry array.");
    end