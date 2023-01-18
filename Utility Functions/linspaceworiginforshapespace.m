% This script is like the linspace function, but includes the origin in the
% distribution of points returned.
function [X,Y] = linspaceworiginforshapespace(dnum,ank)
temp_left = linspace(-ank,0,dnum-ceil(dnum/2)+1);  temp_right = linspace(0,ank,ceil(dnum/2));
X = [temp_left(1:end-1), temp_right]; Y = X;