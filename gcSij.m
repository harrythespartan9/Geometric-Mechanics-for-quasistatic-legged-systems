function g = gcSij(kin, aa, ll, ank)

% Unpack the kinematic structure
ksq_12_func = kin.ksq_f;
% Create the appropriate plot text
shape_1_txt = ['alpha_' num2str(kin.cs1)];
shape_2_txt = ['alpha_' num2str(kin.cs2)];
k_txt = ['k^2_{' num2str(kin.cs1) num2str(kin.cs2) '}'];

% Discretization num for the plot
% keep this at 100 for now. This is also the discretization for the ode
% solver that computes the constrained gait.
dnum = 100;          

% Create the x-axis (shape element 1)
xx = linspace(-ank,ank,dnum);       
% Create the y-axis (shape element 2)
yy = xx;

[ai,aj] = meshgrid(xx,yy);
ksq_sweep = ksq_12_func(aa, ll, ai, aj);
ksq_M = ksq_12_func(aa, ll, xx, yy); % major axis of the level sets
ksq_m = ksq_12_func(aa, ll, xx, -yy);% minor axis of the level sets
ksq_x = ksq_12_func(aa, ll, xx, 0*yy);
ksq_y = ksq_12_func(aa, ll, 0*xx, yy);

% Plot the surface and the constrained nominal and max allowable gait.
figure('units','pixels','position',[0 0 1920 1080],'Color','w')
surf(ai,aj,ksq_sweep,'EdgeColor','none');
axis equal tight; hold on; view(2);
colormap(jet);
colorbar();
line(xx,yy,ksq_M,'Color','k','LineStyle','-'); 
line(xx,-yy,ksq_m,'Color','k','LineStyle','--');
line(0*xx,yy,ksq_y,'Color','k','LineStyle',':');
line(xx,0*yy,ksq_x,'Color','k','LineStyle',':');
xlabel(['$\' shape_1_txt '$'],'Interpreter','latex',FontSize=20);
ylabel(['$\' shape_2_txt '$'],'Interpreter','latex',FontSize=20);
title(['$' k_txt '$'],'Interpreter','latex',FontSize=20);

% Let's choose an initial point to compute the nominal gait-- choose the
% first coordinate to be 'i' and the next one be 'j'. Let's choose the
% second coordinate starting point be 0 and the first coordinate be a
% cosinusoidal oscillation with an amplitude of 10% the corresponding shape
% element range.
tau = linspace(0,1,dnum);
% Set Initial conditions
ai_tau_0 = 0.1*2*ank; % set the IC to 10% of the range in the i direction
aj_tau_0 = 0; % set the IC to 0 in the j direction
% NOTE: Note that both shapes can't start from 0,0 since that is a singular
% point in the current level-2 case for multi-disk systems.

% Return the gait constraint values---------------------
% The a representative waveform, and it gives the maximum scaling that is
% allowable of that waveform. The minimum scaling would be aything that is
% non-zero.
g = [];

end