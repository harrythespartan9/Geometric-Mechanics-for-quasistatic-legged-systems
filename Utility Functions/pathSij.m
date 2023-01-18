% This script computes the gait constraints on the rigid body system with
% non-slipping legs. It outputs the updated kinematic structure
function mpi = pathSij(datai,mpi)

    % Unpack the data -----------------------------------------------------
    ai0 = mpi.s.ic(1); aj0 = mpi.s.ic(2); % initial condition
    plot_info = datai{1};
    plot_kin = datai{2};
    kinfunc = datai{3};
    aa = datai{2}.aa;
    ll = datai{2}.ll;
    dphi = datai{3}.dphi;

    % Compute the path color (based on the inter-leg distance) ------------
    kijsq0 = ksq(aa, ll, ai0, aj0);

    % Create the initial paths---------------------------------------------
    % Run ODE45 to obtain the constrained gait
    options = odeset('Events',@PhaseSijTrigger); % triggers complete cycle
    [t_plus,y_plus,~,~,~] = ode45( @(t,y) dphi(t, aa, ll, y(1), y(2)),...
                    [0 1], [ai0; aj0], options ); % solve in the plus/forward dirn
    [t_minus,y_minus,~,~,~] = ode45( @(t,y) -dphi(t, aa, ll, y(1), y(2)),...
                    [0 1], [ai0; aj0], options ); % solve in the minus/reverse dirn
    
end