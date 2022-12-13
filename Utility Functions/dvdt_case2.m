% This script returns the derivatives for the case two system'sdot
% current configuration y = [x,y,theta,gamma,alpha_1,alpha_2]
%                           [1,2,3    ,4    ,5      ,6      ]
function dvdt = dvdt_case2(t,v,a,l,n,alpha_hat)

% Analytical derivatives of the archimedean spiral that we are using for
% defining the leg movements
dvdt(5) = alpha_hat*cos(2*pi*n*t) - 2*pi*alpha_hat*n*t*sin(2*pi*n*t);
dvdt(6) = alpha_hat*sin(2*pi*n*t) + 2*pi*alpha_hat*n*t*cos(2*pi*n*t);

% Define the sprawl rate:
dvdt(4) = (a*(2*cos(v(5) - v(4)) - 2*cos(v(5) + v(4)) + sqrt(2)*a*sin(v(5) - v(6))))/(4*cos(v(4))*(a*cos(v(5)) + a*cos(v(6)) + 2*sqrt(2)*sin(v(4))))...
    *dvdt(5)...
    -(a*(2*cos(v(6) + v(4)) - 2*cos(v(6) - v(4)) + sqrt(2)*a*sin(v(5) - v(6))))/(4*cos(v(4))*(a*cos(v(5)) + a*cos(v(6)) + 2*sqrt(2)*sin(v(4))))...
    *dvdt(5);

% Define the rotation rate:
dvdt(3) = (-(2*sqrt(2)*a*cos(v(4))*cos(v(3))^2*(sin(v(5)) + sin(v(6))))/(2*sqrt(2)*sin(v(4)) + a*cos(v(5)) + a*cos(v(6)))^2)*dvdt(4)...
    + ((cos(v(3))^2*(a*l^2*cos(v(5))*(2*sqrt(2)*sin(v(4)) + a*cos(v(5)) + a*cos(v(6))) + a^2*l^2*sin(v(5))*(sin(v(5)) + sin(v(6)))))/(l^2*(2*sqrt(2)*sin(v(4)) + a*cos(v(5)) + a*cos(v(6)))^2))*dvdt(5)...
    + ((cos(v(3))^2*(a*l^2*cos(v(6))*(2*sqrt(2)*sin(v(4)) + a*cos(v(5)) + a*cos(v(6))) + a^2*l^2*sin(v(6))*(sin(v(5)) + sin(v(6)))))/(l^2*(2*sqrt(2)*sin(v(4)) + a*cos(v(5)) + a*cos(v(6)))^2))*dvdt(6);

end