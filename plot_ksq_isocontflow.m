function plot_ksq_isocontflow(k,d_x,d_y,cs1,cs2)

% Get the text for the labels
xtxt = ['$\alpha_' num2str(cs1) '$'];
ytxt = ['$\alpha_' num2str(cs2) '$'];
% ctitle_txt = ['$k^2_{' num2str(cs1) num2str(cs2) '}$ contours'];
% ftitle_txt = ['iso-$k^2_{' num2str(cs1) num2str(cs2) '}$ flow'];
cftitle_txt = ['$k^2_{' num2str(cs1) num2str(cs2) '}$ iso-contours and flow'];

% Unlock the joint limits of the system.
ank = 1*pi; % symmetrically defined about 0.
aa = 1; ll = 1;

% Plot info
dnum = 1001; % set this arbitrarily
skipV = 50; % number of points to skip when plotting the vector field
cLvl = 25; % number of countour levels
lW_contour = (2.0*10)/cLvl; % contour linewidth scales with the number of countour lines
lW_Vector = (2.0/100)*skipV; % vector linewidth scales with the density of the vector field
idxQ = 1:skipV:dnum; % indices to plot the vector field at
xx = linspace(-ank,ank,dnum);         
yy = xx;
[ai,aj] = meshgrid(xx,yy);
ksq_sweep = k(aa, ll, ai, aj);
% ksq_M = k(aa, ll, xx, yy);
% ksq_m = k(aa, ll, xx, -yy);
u_sweep = d_x(aa, ll, ai, aj);
v_sweep = d_y(aa, ll, ai, aj);

% Set the default interpreter before plotting to latex
%%%% https://www.mathworks.com/matlabcentral/answers/346436-how-to-use-latex-interpreter-for-xticklabels
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

% Plot the heat-map and then the level-sets
figure('units','pixels','position',[0 0 1920 1080],'Color','w')

subplot(1,2,1);
% surf(ai,aj,ksq_sweep,'EdgeColor','none','FaceAlpha',1); % 0.35
contour(ai,aj,ksq_sweep,cLvl,'LineWidth',lW_contour);
axis equal tight; hold on; view(2);
colormap(jet);
colorbar('TickLabelInterpreter','latex','FontSize',12);
% line(xx,yy,ksq_M,'Color','k','LineStyle','-.');
% line(xx,-yy,ksq_m,'Color','k','LineStyle','--');
% https://www.mathworks.com/matlabcentral/answers/271912-rotate-ylabel-and-keep-centered
set(get(gca,'YLabel'),'rotation',0,'VerticalAlignment','middle');
% title(ctitle_txt,FontSize=20);
xticks(-pi:pi/2:pi);
xticklabels({'$-\pi$','$-\frac{\pi}{2}$','$0$','$\frac{\pi}{2}$','$\pi$'});
yticks(-pi:pi/2:pi);
yticklabels({'$-\pi$','$-\frac{\pi}{2}$','$0$','$\frac{\pi}{2}$','$\pi$'});
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
xlabel(xtxt,FontSize=20);
ylabel(ytxt,FontSize=20);

subplot(1,2,2);
quiver(ai(idxQ,idxQ),aj(idxQ,idxQ),u_sweep(idxQ,idxQ),v_sweep(idxQ,idxQ),...
    'LineWidth',lW_Vector,'Color','k');
axis equal tight; hold on; view(2);
set(get(gca,'YLabel'),'rotation',0,'VerticalAlignment','middle');
% title(ftitle_txt,FontSize=20);
xticks(-pi:pi/2:pi);
xticklabels({'$-\pi$','$-\frac{\pi}{2}$','$0$','$\frac{\pi}{2}$','$\pi$'});
yticks(-pi:pi/2:pi);
yticklabels({'$-\pi$','$-\frac{\pi}{2}$','$0$','$\frac{\pi}{2}$','$\pi$'});
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
xlabel(xtxt,FontSize=20);
ylabel(ytxt,FontSize=20);

sgtitle(cftitle_txt,FontSize=20);

end