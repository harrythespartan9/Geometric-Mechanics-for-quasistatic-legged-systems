% This script computes and plots the constrained gaits on top of the 
% iso-ksq landscape for the requested level-2 contact submanifold of a 
% rigid quadrupedal robot.
function plotSijwKsq(kin)

% Get the functions needed from the structure------------------------------
kin = returnSijfxn(kin);

% Unpack the kinematics structure------------------------------------------
ksq_ij_func = kin.ksq_ij_func;
dphi_ij_func = kin.dphi_ij_func;
ank = kin.ank;

% Create the initial gait--------------------------------------------------
    % Run ODE45 to obtain the constrained gait-----------------------------
    a1_0 = 0.1*2*ank; % x-coord = 10% of x-range
    a2_0 = 0; % y-coord = 0
    options = odeset('Events',@PhaseSijTrigger); % triggers complete cycle
    [t,y,~,~,~] = ode45( @(t,y) dphi_ij_func(t, aa, ll, y(1), y(2)),...
                    [0 5], [a1_0; a2_0], options ); % call the ode solver
    t = t/t(end); % rescale time vector to one period (trigger event)
    % Obtain the sinusoidal fit that's actually the exact solution to the
    % ode solver generated one---------------------------------------------
    si = sinefit(t,y(:,1)); % get each shape element from the solution
    sj = sinefit(t,y(:,2));
    sfit = @(b,x)  b(1).*(sin(2*pi*x + 2*pi/b(2))) + b(3); % fit function
    yi = sfit(si,t); yj = sfit(sj,t); % obtain the fit for plotting

% Scale the initial gait to a minimum and maximum gait---------------------
    % Here, we just take the 10% of the shape element range gait as minimum 
    % and 90% of the range as maximum--------------------------------------
    % Min and Max multipliers:
    mulm = 1; mulM = 9; % make sure you don't leave the shape-space bounds
    % Get the scaled gaits:
    yim = mulm*yi; yjm = mulm*yj;
    yiM = mulM*yi; yjM = mulM*yj;

% Plotting stuff-----------------------------------------------------------
    % Generate the appropriate text for the plot---------------------------
    cs1_txt = num2str(kin.cs1);
    cs2_txt = num2str(kin.cs2);
    cs_txt = [cs1_txt cs2_txt];

dnum = 100; % Change this based on need; discretization of shape-space  

xx = linspace(-ank,ank,dnum); % shape-space points         
yy = xx;
[ai,aj] = meshgrid(xx,yy);

ksq_sweep = ksq_ij_func(0, aa, ll, ai, aj); % ksq values
ksq_M = ksq_ij_func(0, aa, ll, xx, yy);
ksq_m = ksq_ij_func(0, aa, ll, xx, -yy);
ksq_phi_m = ksq_ij_func(0, aa, ll, yim, yjm);
ksq_phi_M = ksq_ij_func(0, aa, ll, yiM, yjM);

figure('units','pixels','position',[0 0 1920 1080],'Color','w') % plot
p1 = surf(ai,aj,ksq_sweep,'EdgeColor','none','FaceAlpha',0.5);
set(get(get(p1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
axis equal tight; hold on; view(2);
colormap(jet);
colorbar();
p2 = line(xx,yy,ksq_M,'Color','k','LineStyle','-.');
p3 = line(xx,-yy,ksq_m,'Color','k','LineStyle','--');
line(yim,yjm,ksq_phi_m,'Color', 'k','LineStyle','-', 'LineWidth', 1.2,...
    'DisplayName',['$\phi^m_$' num2str(cs_txt)]); % smallest gait
line(yiM,yjM,ksq_phi_M,'Color', 'k','LineStyle','-', 'LineWidth', 2.4,...
    'DisplayName',['$\phi^M_$' num2str(cs_txt)]); % largest gait (fixed linewidth scaling)
xlabel(['$\alpha_' num2str(cs1_txt) '$'],'Interpreter','latex',FontSize=20);
ylabel(['$\alpha_' num2str(cs2_txt) '$'],'Interpreter','latex',FontSize=20);
title(['$k^2_{' num2str(cs_txt) '}$'],'Interpreter','latex',FontSize=20);
set(get(get(p1,'Annotation'),'LegendInformation'),...
    'IconDisplayStyle','off'); % no legend entries for non-gait plots
set(get(get(p2,'Annotation'),'LegendInformation'),...
    'IconDisplayStyle','off');
set(get(get(p3,'Annotation'),'LegendInformation'),...
    'IconDisplayStyle','off');
legend('location','northeast','box','off','Interpreter','latex','FontSize',10);

% -------------------------------------------------------------------------

end