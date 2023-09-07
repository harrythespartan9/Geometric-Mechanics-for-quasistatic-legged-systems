function [netCurve, CurveTraj] = extractPathCurvFromQ(Q, Qdot, ddpsi, A, Adot)
%EXTRACTPATHCURVFROMQ This function extracts the path curvature given local configuration, its velocity, and the shape(directional) derivatives of the gait
%constraint and the local connection vector fields

    % Unpack the inputs
    t       = Q{1}; t_dot       = ones(size(t));
    x       = Q{2}; x_dot       = Qdot(1, :);
    y       = Q{3}; y_dot       = Qdot(2, :);
    theta   = Q{4}; theta_dot   = Qdot(3, :);
    ai      = Q{5}; ai_dot      = Qdot(4, :);
    aj      = Q{6}; aj_dot      = Qdot(5, :);
    

    % Initialize, iterate, and compute the SE(2) acceleration over each cycle
    ai_ddot = nan(size(ai)); aj_ddot = nan(size(aj));
    g_ddot = nan(3, numel(t));
    for i = 1:numel(t)
        
        % get the current time step phase information
        tNow       = t(i);
        xNow       = x(i);        x_dotNow       = x_dot(i);
        yNow       = y(i);        y_dotNow       = y_dot(i);
        thetaNow   = theta(i);    theta_dotNow   = theta_dot(i);
        aiNow      = ai(i);       ai_dotNow      = ai_dot(i);
        ajNow      = aj(i);       aj_dotNow      = aj_dot(i);

        % Compute each term in the acceleration formulation

    end




    CurveTraj = Qdot(3, :)./sqrt(Qdot(1, :).^2 + Qdot(2, :).^2);
    netCurve = mean(CurveTraj);
end

