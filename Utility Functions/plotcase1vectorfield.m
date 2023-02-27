% This function plots the kinematic connection vector fields of the case 1 
% system.
function plotcase1vectorfield(ax,plot_kin,plot_info,C,k)

% Unpack the plotting tools
tickFS = plot_info.tickFS; titleFS = plot_info.titleFS;
xtickval = plot_info.xtickval; ytickval = plot_info.xtickval;
xticklab = plot_info.xticklab; yticklab = plot_info.yticklab;
idx = [C.idx{2}(C.idx{1} == k), C.idx{3}(C.idx{1} == k)]; % loc in p grid
configs = plot_info.configs{idx(1),idx(2)};
title_txt = plot_info.title_txt{idx(1),idx(2)};
itQ = plot_info.idxQ;
col_backg = plot_info.col_backg; gc_col = plot_info.gc_col;
lW_Vector = plot_info.lW_Vector; i = plot_info.i; j = plot_info.j;
xlimits = plot_info.xlimits; ylimits = plot_info.ylimits;
ksq_lb = plot_info.ksq_lb; lW_c = plot_info.lW_contour;

% Unpack the kinematics data
ai = plot_kin.ai; aj = plot_kin.aj; ksq_sweep = plot_kin.ksq_sweep;
u_sweep = eval(plot_info.vecF_txt{idx(1),idx(2),1}); 
v_sweep = eval(plot_info.vecF_txt{idx(1),idx(2),2});

% Make the plot on the axes provided
quiver(ai(itQ,itQ),aj(itQ,itQ),u_sweep(itQ,itQ),v_sweep(itQ,itQ),...
    'LineWidth',lW_Vector,'Color','k');
axis equal tight; hold on;
contour(ax, ai, aj, ksq_sweep, [ksq_lb ksq_lb], 'k--', 'LineWidth', lW_c+1);
if configs % check if we need to plot the location of the configuration
    % Unpack relevant plot info
    lW_m = plot_info.lW_m;
    circS_q = plot_info.circS_q; col_q = plot_info.col_q;
    % Plot the location of the configuration
    scatter(ax,ai(i,j),aj(i,j),circS_q,col_q,'filled','MarkerEdgeColor','k','LineWidth',lW_m,'Marker','square'); % config
end
set(get(ax,'YLabel'),'rotation',0,'VerticalAlignment','middle');
title(ax,title_txt,'Color','k',FontSize=titleFS);
xticks(ax,xtickval); yticks(ax,ytickval);
xticklabels(ax,xticklab); yticklabels(ax,yticklab);
ax.XAxis.FontSize = tickFS; ax.YAxis.FontSize = tickFS; 
set(ax,'Color',col_backg);
xlim(xlimits); ylim(ylimits);

end