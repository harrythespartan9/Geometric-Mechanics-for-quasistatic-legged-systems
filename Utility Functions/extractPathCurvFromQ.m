function [netCurve, CurveTraj] = extractPathCurvFromQ(Q, Qdot, ddpsi, A, Adot)
%EXTRACTPATHCURVFROMQ This function extracts the path curvature given local configuration, its velocity, and the shape(directional) derivatives of the gait
%constraint and the local connection vector fields

    % Unpack the inputs
    t       = Q{1};
    x       = Q{2}; x_dot       = Qdot{1};
    y       = Q{3}; y_dot       = Qdot{2};
    theta   = Q{4}; theta_dot   = Qdot{3};
    ai      = Q{5}; ai_dot      = Qdot{4};
    aj      = Q{6}; aj_dot      = Qdot{5};
    

    % Initialize, iterate, and compute the SE(2) acceleration over each cycle
    g_ddot = nan(3, numel(t));
    for i = 1:numel(t)
        
        % get the current time step phase information
        t       = t(i);
        x       = x(i);        x_dot       = x_dot(i);
        y       = y(i);        y_dot       = y_dot(i);
        theta   = theta(i);    theta_dot   = theta_dot(i);
        ai      = ai(i);       ai_dot      = ai_dot(i);
        aj      = aj(i);       aj_dot      = aj_dot(i);

        % Compute each term in the acceleration formulation
        term_theta_dot = 

    end




    CurveTraj = Qdot(3, :)./sqrt(Qdot(1, :).^2 + Qdot(2, :).^2);
    netCurve = mean(CurveTraj);
end

