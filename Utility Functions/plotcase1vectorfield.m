% This function plots the kinematic connection vector fields of the case 1 
% system.
function plotcase1vectorfield(ax,p_kin,p_info,~,idx) % child struct 'C' not
                                                     % being used rn

% Unpack the plotting tools
tickFS = p_info.tickFS; titleFS = p_info.titleFS;
xtickval = p_info.xtickval; ytickval = p_info.xtickval;
xticklab = p_info.xticklab; yticklab = p_info.yticklab;
config = p_info.config{idx(1),idx(2)};
title_txt = p_info.title_txt{idx(1),idx(2)};
itQ = p_info.idxQ;
col_backg = p_info.col_backg; gc_col = p_info.gc_col;
lW_Vector = p_info.lW_Vector; i = p_info.i; j = p_info.j;

% Unpack the kinematics data
ai = p_kin.ai; aj = p_kin.aj;
u_sweep = eval(p_info.vecF_txt{idx(1),idx(2),1}); 
v_sweep = eval(p_info.vecF_txt{idx(1),idx(2),2});

% Make the plot on the axes provided 
quiver(ai(itQ,itQ),aj(itQ,itQ),u_sweep(itQ,itQ),v_sweep(itQ,itQ),...
    'LineWidth',lW_Vector,'Color','k');
axis equal tight; hold on;
if config % check if we need to plot the location of the configuration
    % Unpack relevant plot info
    lW_m = p_info.lW_m;
    circS_q = p_info.circS_q; col_q = p_info.col_q;
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