function [netCurve, CurveTraj, Q_dot, Q_ddot] = extractPathCurvFromQ(Q, Qdot, ddpsi, A, Adot, params)
%EXTRACTPATHCURVFROMQ This function extracts the path curvature given local configuration, its velocity, the shape(directional) derivatives of the gait
%constraint and local connection vector fields, and some shape parameters of the SE(2) system.

    % Unpack the inputs
    aa      = params{1}; 
    ll      = params{2};
    t       = Q{1};     t_dot       = ones(size(t));
    x       = Q{2};     x_dot       = Qdot(1, :);
    y       = Q{3};     y_dot       = Qdot(2, :);
    theta   = Q{4};     theta_dot   = Qdot(3, :);      g_dot = [x_dot; y_dot; theta_dot];
    ai      = Q{5};     ai_dot      = Qdot(4, :);
    aj      = Q{6};     aj_dot      = Qdot(5, :);

    % Define a SE(2) translational choosing matrix
    Cxy = [eye(2, 2), zeros(2, 1)];

    % Initialize, iterate, and compute the SE(2) acceleration over each cycle
    t_ddot = zeros(size(t_dot));
    g_ddot = nan(3, numel(t));
    ai_ddot = nan(size(ai)); aj_ddot = nan(size(aj));
    for i = 1:numel(t)

        % get the current time step phase information
        tNow       = t(i);
        xNow       = x(i);        x_dotNow       = x_dot(i);
        yNow       = y(i);        y_dotNow       = y_dot(i);
        thetaNow   = theta(i);    theta_dotNow   = theta_dot(i);
        aiNow      = ai(i);       ai_dotNow      = ai_dot(i);
        ajNow      = aj(i);       aj_dotNow      = aj_dot(i);       a_dotNow      = [ai_dotNow; aj_dotNow];

        % Compute each term in the acceleration formulation
        a_ddot = ddpsi(tNow, aa, ll, aiNow, ajNow)*a_dotNow; % shape acceleration
        term_1 = -theta_dotNow*rot_deriv_SE2(thetaNow)*...
            A(tNow, aa, ll, aiNow, ajNow)*a_dotNow;                                   % g_ddot contribution from frame rotation
        term_2 = -rot_SE2(thetaNow)*...
           [Adot{1}(tNow, aa, ll, aiNow, ajNow)*a_dotNow, ...
            Adot{2}(tNow, aa, ll, aiNow, ajNow)*a_dotNow]*a_dotNow;                   % g_ddot contribution from local connection derivative
        term_3 = -rot_SE2(thetaNow)*A(tNow, aa, ll, aiNow, ajNow)*a_ddot;             % g_ddot contribution from shape acceleration

        % Assign current time step data
        ai_ddot(i) = a_ddot(1); aj_ddot(i) = a_ddot(2);
        g_ddot(:, i) = term_1 + term_2 + term_3;

    end

    % Compute the SE(2) trajectory's curvature and net curvature
    CurveTraj = (g_dot(1, :).*g_ddot(2, :) - g_dot(2, :).*g_ddot(1, :)) ./ vecnorm(Cxy*g_dot, 2, 1);
    netCurve = mean(CurveTraj);

    % Pack up the configuration velocity and acceleration as well
    Q_dot  = mat2cell(  [t_dot(:)'; Qdot]                     , ones(size(Qdot, 1)+1, 1), size(Qdot, 2)  );
    Q_ddot = mat2cell(  [t_ddot(:)'; g_ddot; ai_ddot; aj_ddot], ones(size(Qdot, 1)+1, 1), size(Qdot, 2)  ); 

end

