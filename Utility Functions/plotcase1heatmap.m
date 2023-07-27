% This function plots the kinematic heat maps of the case 1 system.
function plotcase1heatmap(ax,plot_kin,plot_info,C,k)

% Unpack the plotting tools
fA = plot_info.fA; cfLvl = plot_info.cfLvl;
tickFS = plot_info.tickFS; titleFS = plot_info.titleFS;
xtickval = plot_info.xtickval; ytickval = plot_info.xtickval;
xticklab = plot_info.xticklab; yticklab = plot_info.yticklab;
gc_col = plot_info.gc_col;
CUB = plot_info.CUB;
idx = [C.idx{2}(C.idx{1} == k), C.idx{3}(C.idx{1} == k)]; % parent grid loc
heat_lim = C.limits; lW_c = plot_info.lW_contour;
configs = plot_info.configs{idx(1),idx(2)};
title_txt = plot_info.title_txt{idx(1),idx(2)};
xlimits = plot_info.xlimits; ylimits = plot_info.ylimits;
ksq_lb = plot_info.ksq_lb; 

% Unpack the kinematics data
ai = plot_kin.ai; aj = plot_kin.aj; ksq_sweep = plot_kin.ksq_sweep;
heat_sweep = eval(plot_info.heat_sweep_txt{idx(1),idx(2)}); % evaluate the 
                                                         % saved text

% Make the plot on the axes provided 
contourf(ax,ai,aj,heat_sweep,cfLvl,'LineStyle','none','FaceAlpha',fA); % plot the filled contour for the entire shape space
axis equal tight; hold on;
contour(ax, ai, aj, heat_sweep, [0 0], 'k:', 'LineWidth', lW_c+1); % plot the 0-contour with a dotted, black trace
contour(ax, ai, aj, ksq_sweep, [ksq_lb ksq_lb], 'k--', 'LineWidth', lW_c+1); % plot the shape space limit near the contracting singularity
if configs
    % Unpack relevant plot info
    i = plot_info.i; j = plot_info.j; % shape-space configuration
    lW_m = plot_info.lW_m;
    circS_q = plot_info.circS_q; col_q = plot_info.col_q;
    % Plot the location of the configuration
    scatter(ax,ai(i,j),aj(i,j),circS_q,col_q,'filled','MarkerEdgeColor','k','LineWidth',lW_m,'Marker','square'); % config
end
colormap(ax,CUB); clim(ax,heat_lim);
set(get(ax,'YLabel'),'rotation',0,'VerticalAlignment','middle');
title(ax,title_txt,'Color','k',FontSize=titleFS);
xticks(ax,xtickval); yticks(ax,ytickval);
xticklabels(ax,xticklab); yticklabels(ax,yticklab);
ax.XAxis.FontSize = tickFS; ax.YAxis.FontSize = tickFS;
xlim(xlimits); ylim(ylimits);

end