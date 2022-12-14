% This function plots the kinematic connection vector fields of the case 1 
% system.
function plotcase1vectorfield(ax,plt_kin,plt_info,idx)

% Unpack the plotting tools
tickFS = plt_info.tickFS; titleFS = plt_info.titleFS;
xtickval = plt_info.xtickval; ytickval = plt_info.xtickval;
xticklab = plt_info.xticklab; yticklab = plt_info.yticklab;
config = plt_info.config{idx(1),idx(2)};
title_txt = plt_info.title_txt{idx(1),idx(2)};
idxQ = plt_info.idxQ; 
col_backg = plt_info.col_backg; gc_col = plt_info.gc_col;
lW_Vector = plt_info.lW_Vector;

% Unpack the kinematics data
i = plt_kin.i; j = plt_kin.j; ai = plt_kin.ai; aj = plt_kin.aj; 
u_sweep = plt_kin.vecF.u{idx(1),idx(2)}; v_sweep = plt_kin.vecF.v{idx(1),idx(2)};

% Make the plot on the axes provided 
quiver(ai(idxQ,idxQ),aj(idxQ,idxQ),u_sweep(idxQ,idxQ),v_sweep(idxQ,idxQ),...
    'LineWidth',lW_Vector,'Color','k');
axis equal tight; hold on;
if config % check if we need to plot the location of the configuration
    % Unpack relevant plot info
    lW_m = plt_info.lW_m;
    circS_q = plt_info.circS_q; col_q = plt_info.col_q;
    % Plot the location of the configuration
    scatter(ax,ai(i,j),aj(i,j),circS_q,col_q,'filled','MarkerEdgeColor','k','LineWidth',lW_m,'Marker','square'); % config
end
set(get(ax,'YLabel'),'rotation',0,'VerticalAlignment','middle');
title(ax,title_txt,'Color',gc_col,FontSize=titleFS);
xticks(ax,xtickval); yticks(ax,ytickval);
xticklabels(ax,xticklab); yticklabels(ax,yticklab);
ax.XAxis.FontSize = tickFS; ax.YAxis.FontSize = tickFS; 
set(ax,'Color',col_backg);

end