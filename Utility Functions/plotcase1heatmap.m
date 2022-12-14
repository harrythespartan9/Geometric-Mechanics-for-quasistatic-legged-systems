% This function plots the kinematic heat maps of the case 1 system.
function plotcase1heatmap(ax,plt_kin,plt_info,idx)

% Unpack the plotting tools
fA = plt_info.fA; cfLvl = plt_info.cfLvl;
tickFS = plt_info.tickFS; titleFS = plt_info.titleFS;
xtickval = plt_info.xtickval; ytickval = plt_info.xtickval;
xticklab = plt_info.xticklab; yticklab = plt_info.yticklab;
CUB = plt_info.CUB; 
heat_col_lim = plt_info.heat_col_lim{idx(1),idx(2)};
config = plt_info.config{idx(1),idx(2)};
title_txt = plt_info.title_txt{idx(1),idx(2)};

% Unpack the kinematics data
i = plt_kin.i; j = plt_kin.j; ai = plt_kin.ai; aj = plt_kin.aj; 
heat_sweep = plt_kin.heat_sweep{idx(1),idx(2)};

% Make the plot on the axes provided 
contourf(ax,ai,aj,heat_sweep,cfLvl,'LineStyle','none','FaceAlpha',fA);
axis equal tight; hold on;
if config
    % Unpack relevant plot info
    lW_m = plt_info.lW_m;
    circS_q = plt_info.circS_q; col_q = plt_info.col_q;
    % Plot the location of the configuration
    scatter(ax,ai(i,j),aj(i,j),circS_q,col_q,'filled','MarkerEdgeColor','k','LineWidth',lW_m,'Marker','square'); % config
end
colormap(ax,CUB); clim(ax,heat_col_lim);
set(get(ax,'YLabel'),'rotation',0,'VerticalAlignment','middle');
title(ax,title_txt,'Color',gc_col,FontSize=titleFS);
xticks(ax,xtickval); yticks(ax,ytickval);
xticklabels(ax,xticklab); yticklabels(ax,yticklab);
ax.XAxis.FontSize = tickFS; ax.YAxis.FontSize = tickFS;

end