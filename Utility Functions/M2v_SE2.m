function v = M2v_SE2(M)
%M2V_SE2 converts the vector form of an SE2 group element to its matrix form for group actions. No concistency checks are performed in this function.

    switch isa(M, 'sym')
        
        case 1 % the symbolic case

            v = simplify([M(1,3); 
                          M(2,3);
                          atan(mean([M(2,1)/M(1,1), -M(1,2)/M(2,2)]))],...
                          'Steps', 10, 'IgnoreAnalyticConstraints', true); % ignoring analytical constraints because the assumptions are not carried over.
            

        case 0 % the numeric case-- if not an error will pop up in some step below

            % initialize
            v = nan(3, 1);
            
            % Compute the angle
            M_sin = mean([-M(1, 2), M(2, 1)]); 
            M_cos = mean([M(1,1), M(2, 2)]); % average sine and cosine values from the rotation matrix about z
            v(3) = atan2(M_sin, M_cos);
        
            % Compute the translation
            v(1:2) = M(1:2, 3);

    end

end

