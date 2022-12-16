% This function plots the kinematic heat maps of the case 1 system.
function plotcase1heatmap(ax,p_kin,p_info,idx,trF)

% Unpack the plotting tools
fA = p_info.fA; cfLvl = p_info.cfLvl;
tickFS = p_info.tickFS; titleFS = p_info.titleFS;
xtickval = p_info.xtickval; ytickval = p_info.xtickval;
xticklab = p_info.xticklab; yticklab = p_info.yticklab;
CUB = p_info.CUB;
if trF % if translation heat map
    heat_lim = p_info.heat_col_lim{idx(1),idx(2)}(1,:);
else % if rotation heat map
    heat_lim = p_info.heat_col_lim{idx(1),idx(2)}(2,:);
end
config = p_info.config{idx(1),idx(2)};
title_txt = p_info.title_txt{idx(1),idx(2)};
i = p_info.i; j = p_info.j; % shape-space configuration

% Unpack the kinematics data
ai = p_kin.ai; aj = p_kin.aj; 
heat_sweep = p_kin.heat_sweep{idx(1),idx(2)};

% Make the plot on the axes provided 
contourf(ax,ai,aj,heat_sweep,cfLvl,'LineStyle','none','FaceAlpha',fA);
axis equal tight; hold on;
if config
    % Unpack relevant plot info
    lW_m = p_info.lW_m;
    circS_q = p_info.circS_q; col_q = p_info.col_q;
    % Plot the location of the configuration
    scatter(ax,ai(i,j),aj(i,j),circS_q,col_q,'filled','MarkerEdgeColor','k','LineWidth',lW_m,'Marker','square'); % config
end
colormap(ax,CUB); clim(ax,heat_lim);
set(get(ax,'YLabel'),'rotation',0,'VerticalAlignment','middle');
title(ax,title_txt,'Color',gc_col,FontSize=titleFS);
xticks(ax,xtickval); yticks(ax,ytickval);
xticklabels(ax,xticklab); yticklabels(ax,yticklab);
ax.XAxis.FontSize = tickFS; ax.YAxis.FontSize = tickFS;

end