% This script computes the gait constraints on the rigid body system with
% non-slipping legs. It outputs the updated kinematic structure
function [k,si,sj,t,bF] = gcSij(k,cs1,cs2)

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

% Shape-space bounds violation---------------------------------------------
    % Run the bounds violation check and obtain the time interval where the
    % points are violated.
    [boundFi,tauIntervali,tauValidi] = shapebounds(si);
    [boundFj,tauIntervalj,tauValidj] = shapebounds(sj);

    % Pack the results into the kinematic structure.

    k.bF{k.cs_idx} = [boundFi; boundFj]; % bound flag

    ni = tauIntervali; nj = tauIntervalj;
    if ni > nj
        tauIntervalj = [tauIntervalj,nan(1,ni-nj)];
        tauValidj = [tauValidj,nan(1,ni-nj)];
    elseif ni < nj
        tauIntervali = [tauIntervali,nan(1,nj-ni)];
        tauValidi = [tauValidi,nan(1,nj-ni)];
    end
    k.tauI{k.cs_idx} = [tauIntervali; tauIntervalj]; % time intervals
    k.tauV{k.cs_idx} = [tauValidi; tauValidj]; % time interval validity

end