%V2M_SE2 converts the vector form of an SE2 group element to its matrix form for group actions.
function M = v2M_SE2(v)
    
    % Compute the Matrix form directly
    M = [cos(v(3)), -sin(v(3)), v(1);
         sin(v(3)),  cos(v(3)), v(2);
                 0,         0,    1];

end

