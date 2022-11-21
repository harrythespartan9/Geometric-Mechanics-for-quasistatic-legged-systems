% This script fits a sine curve to the given data; the algorithm is
% obtained from @Star_Strider 's answer in the following MATLAB answers
% link: https://www.mathworks.com/matlabcentral/answers/121579-curve-fitting-to-a-sinusoidal-function
function s = sinefit(x,y)

% Condition the response to get the sine curver 'specs'
yu = max(y);
yl = min(y);
yr = (yu-yl);                               % Range of ‘y’
yz = y-yu+(yr/2);
ym = mean(y);                               % Estimate offset

fit = @(b,x)  b(1).*(sin(2*pi*x + 2*pi/b(2))) + b(3);    % Function to fit

fcn = @(b) sum((fit(b,x) - y).^2);                       % Least-Squares cost function
s = fminsearch(fcn, [yr/2;  -1;  ym]);                   % Minimise Least-Squares

end