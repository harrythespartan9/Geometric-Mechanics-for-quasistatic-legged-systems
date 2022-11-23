function [k,si,sj,t] = gcSij(k,cs1,cs2)

% Get the functions needed from the structure------------------------------
k = returnSijfxn(k,cs1,cs2);

% Unpack the kinematics structure------------------------------------------
dphi_ij_func = k.dphi_ij{k.cs_idx};
ank = k.ank;

% Create the initial gait--------------------------------------------------
    % Run ODE45 to obtain the constrained gait-----------------------------
    a1_0 = 0.1*ank; % x-coord = 10% of x-range
    a2_0 = 0; % y-coord = 0
    options = odeset('Events',@PhaseSijTrigger); % triggers complete cycle
    [t,y,~,~,~] = ode45( @(t,y) dphi_ij_func(t, k.aa, k.ll, y(1), y(2)),...
                    [0 5], [a1_0; a2_0], options ); % call the ode solver
    t = t/t(end); % rescale time vector to one period (trigger event)
    % Obtain the sinusoidal fit that's actually the exact solution to the
    % ode solver generated one---------------------------------------------
    si = sinefit(t,y(:,1)); % get each shape element from the solution
    sj = sinefit(t,y(:,2)); % return these two fits.

end