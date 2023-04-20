% This function computes the difference between array elements along the unit circle in the y-direction.  Since this is a unit magnitude complex group C1 
% defined over complex multiplication, the "c1" difference between C1 and C2 are defined in the multiplicative sense and let's call this X. Consider, 
% C1 x X = C2, then X = C1\C2. Hence, c1diff(C1, C2) = C1\C2 where C1 and C2 are in c1. Then, we convert this back into angular coordinates using atan2.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Inputs: 1) C: this is an array where we are taking the difference between successive columns; in "diff" function notation, this would be diff(C, 1, 2).
% Outputs:1) x: the computed phase angular difference between the set provided
%         2) s: locations where the entries in the column need to be swapped
function [x, s] = c1diff(C)

    x = nan(size(C, 1), size(C, 2)-1);      % initialize
    s = zeros(size(C, 1), size(C, 2)-1);
    for i = 1:size(x, 2)                    % compute difference between successive columns in diff(C, 1, 2) style
        x(:, i) = C(:, i).\C(:, i+1);
        x(:, i) = round(x(:, i), 3);        % rounded off to 3 decimal places; machine precision values will lead to issues later
        x(:, i) = sum(...
                atan2(imag(x(:, i)),...
                real(x(:, i))), 2 );        % get the 4-quadrant arctangent to convert that into angular difference
        s = s | (x(:, i) < 0);              % get the elements that need to be circshifted
    end


end