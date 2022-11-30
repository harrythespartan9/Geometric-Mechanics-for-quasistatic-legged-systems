% This script computes and plots the constrained gaits on top of the 
% iso-ksq landscape for the requested level-2 contact submanifold of a 
% rigid quadrupedal robot.
function plotSijwKsq(k,cs1,cs2)

% Obtain the gait constraints----------------------------------------------
[k,si,sj,t] = gcSij(k,cs1,cs2);
sfit = @(b,x)  b(1).*(sin(2*pi*x + 2*pi/b(2))) + b(3); % fit function
yi = sfit(si,t); yj = sfit(sj,t); % obtain the fit for plotting

% Scale the initial gait to a minimum and maximum gait---------------------
% Here, we just take the 10% of the shape element range gait as minimum 
% and 90% of the range as maximum--------------------------------------
% Min and Max multipliers:
mulm = k.mul{k.cs_idx}(1); mulM = k.mul{k.cs_idx}(2); % make sure you don't leave the shape-space bounds
% Get the scaled gaits:
yim = mulm*yi; yjm = mulm*yj;
yiM = mulM*yi; yjM = mulM*yj;

% Plotting stuff-----------------------------------------------------------
% Generate the appropriate text for the plot---------------------------
cs1_txt = num2str(cs1);
cs2_txt = num2str(cs2);
cs_txt = [cs1_txt cs2_txt];

% Check if the colors are input or else we can just use black as default.
if ~isfield(k,'circ')
    % Assign the black to the gait plot.
    circ = (1/255)*[0,0,0];
else
    % Assign the corresponding color of the submanifold
    circ = k.circ{k.cs_idx};
end

% Unpack the kinematics structure------------------------------------------
ksq_ij_func = k.ksq_ij{k.cs_idx};
ank = k.ank;

% Take the indices that just satisfy the shape-space bounds
idxm = (yim < ank) & (yim > -ank) & (yjm < ank) & (yjm > -ank);
idxM = (yiM < ank) & (yiM > -ank) & (yjM < ank) & (yjM > -ank);

dnum = 100; % Change this based on need; discretization of shape-space  

xx = linspace(-ank,ank,dnum); % shape-space points         
yy = xx;
[ai,aj] = meshgrid(xx,yy);

ksq_sweep = ksq_ij_func(0, k.aa, k.ll, ai, aj); % ksq values
ksq_M = ksq_ij_func(0, k.aa, k.ll, xx, yy);
ksq_m = ksq_ij_func(0, k.aa, k.ll, xx, -yy);
ksq_phi_m = ksq_ij_func(0, k.aa, k.ll, yim(idxm), yjm(idxm));
ksq_phi_M = ksq_ij_func(0, k.aa, k.ll, yiM(idxM), yjM(idxM));

figure('units','pixels','position',[0 0 1920 1080],'Color','w') % plot
p1 = surf(ai,aj,ksq_sweep,'EdgeColor','none','FaceAlpha',0.35);
set(get(get(p1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
axis equal tight; hold on; view(2);
colormap(jet);
% colorbar(); % we don't need to display the value since the gaits are
% constrained with an upper and lower achievable limit
p2 = line(xx,yy,ksq_M,'Color','k','LineStyle','-.');
p3 = line(xx,-yy,ksq_m,'Color','k','LineStyle','--');
line(yim(idxm),yjm(idxm),ksq_phi_m,'Color', circ,'LineStyle','-', 'LineWidth', 1.2,...
    'DisplayName',['$' num2str(10*mulm) '% \phi_{' num2str(cs_txt) '}$']); % smallest gait
line(yiM(idxM),yjM(idxM),ksq_phi_M,'Color', circ,'LineStyle','-', 'LineWidth', 2.4,...
    'DisplayName',['$' num2str(10*mulM) '% \phi_{' num2str(cs_txt) '}$']); % largest gait (fixed linewidth scaling)
xlabel(['$\alpha_' num2str(cs1_txt) '$'],'Interpreter','latex',FontSize=20);
ylabel(['$\alpha_' num2str(cs2_txt) '$'],'Interpreter','latex',FontSize=20);
title(['$k^2_{' num2str(cs_txt) '}$'],'Interpreter','latex',FontSize=20);
set(get(get(p1,'Annotation'),'LegendInformation'),...
    'IconDisplayStyle','off'); % no legend entries for non-gait plots
set(get(get(p2,'Annotation'),'LegendInformation'),...
    'IconDisplayStyle','off');
set(get(get(p3,'Annotation'),'LegendInformation'),...
    'IconDisplayStyle','off');
legend('location','northeast','Interpreter','latex',...
    'FontSize',7.5); % ,'box','off'
xlim([-ank ank]); ylim([-ank ank]); % set the plot limits

% -------------------------------------------------------------------------

end