% This script computes the gait constraints on the rigid body system with
% non-slipping legs. It outputs the updated kinematic structure
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

% Shape-space bounds violation---------------------------------------------
    % Run the bounds violation check and obtain the time interval where the
    % points are violated.
    [boundFi,tauIntervali] = shapebounds(si,ank);
    [boundFj,tauIntervalj] = shapebounds(sj,ank);

    % Pack the results into the kinematic structure-- first we need group
    % the results for both shape elements since we are trying to constrain
    % gaits for this specific shape-space slice of shape elements i and j
    %----------------------------------------------------------------------

    k.bF{k.cs_idx} = min([boundFi, boundFj]); % bound flag-- taking the min

    tI = unique([tauIntervali,tauIntervalj]); % get the joint intervals
    k.tauI{k.cs_idx} = tI;

    % Compute if the gait is valid in each interval-- take the
    % middle value in each interval and then check if it is within
    % bounds.
    t_test = tI(1:end-1) + 0.5*diff(tI); % bisect each interval to test validity
    tV = ( si(1)*sin(2*pi*(t_test + 1/si(2))) + si(3) <= ank ) &...
         ( si(1)*sin(2*pi*(t_test + 1/si(2))) + si(3) >= -ank ) &...
         ( sj(1)*sin(2*pi*(t_test + 1/sj(2))) + sj(3) <= ank ) &...
         ( sj(1)*sin(2*pi*(t_test + 1/sj(2))) + sj(3) >= -ank ); % check bounds
    k.tauV{k.cs_idx} = tV(:)'; % ensure that it is a row vector

% Smallest/Largest allowable gait------------------------------------------
    % The non-connecting level-sets mandate a more sophisticated way of
    % obtaining the gains for the minimum and maximum allowable gait
    % constraint trajectory.
    sing_pt_dirn = [si(3); sj(3)]/abs([si(3); sj(3)]);
    % The smallest gait needs to intersect this point,

end