% This script helps with the motion planning of the level-2 kinematics of
% the 
function [datai,videoi] = qlevel2noslip_mp(datai)

    % Call the appropriate case based on the number of arguments
    switch nargin
        case 1
            if isfield(datai,gait)
                cond = 1; 
            elseif numel(datai) == 4
                cond = 2; 
            end
        case 2
            cond = 3;
    end

    % Unpack plotting information
    lW_V = datai{1}.lW_V; iQ = datai{1}.iQ; % plot parameters
    fA = datai{1}.fA; cfLvl = datai{1}.cfLvl;
    gc_col = datai{1}.gc_col; col_backg = datai{1}.col_backg;
    col = datai{1}.col; CUB = datai{1}.CUB;
    titleFS = datai{1}.titleFS; tickFS = datai{1}.tickFS;
    labelFS = datai{1}.labelFS; sgtitleFS = datai{1}.sgtitleFS;
    sgtitle_txt = datai{1}.sgtitle_txt;
    x_label_txt = datai{1}.x_label_txt;
    y_label_txt = datai{1}.y_label_txt;
    xtickval = datai{1}.ytickval; ytickval = datai{1}.ytickval;
    xticklab = datai{1}.xticklab; yticklab = datai{1}.yticklab;
    xlimits = datai{1}.xlimits; ylimits = datai{1}.ylimits;
    ai = datai{2}.ai; aj = datai{2}.aj;
    
    dphi_x_sweep = datai{2}.dphi_x_sweep; % sweep data
    dphi_y_sweep = datai{2}.dphi_y_sweep;
    dz__x_sweep = datai{2}.dz__x_sweep;
    dz__y_sweep = datai{2}.dz__y_sweep;
    dz__theta_sweep = datai{2}.dz__theta_sweep;

    % Compute the limits on the data-- helps with countour plotting
    [C1_lim,C2_lim] = se2limits(dz__x_sweep,dz__y_sweep,dz__theta_sweep);

    % Compute everything you need to generate the child tiledlayout
    % parent layout
    P = [];
    P.grid = [3 1];
    % child 1-- dz_translation
    C1 = []; C1.limits = C1_lim; % previously defined color limits
    C1.start = [1, 1];
    C1.grid = [2, 1];
    C1.tile_start = (C1.start(1)-1)*P.grid(2) + C1.start(2);
    % child 2-- dz_theta
    C2 = []; C2.limits = C2_lim;
    C2.start = [3, 1];
    C2.grid = [1, 1];
    C2.tile_start = (C2.start(1)-1)*1 + C2.start(2);
    
    % Work through each case
    switch cond
        
        case 1 % just stratified panels

            % Create two child layouts-- one for translational panels and
            % one for rotation panel
            set(groot,'defaultAxesTickLabelInterpreter','latex'); 
            set(groot,'defaulttextinterpreter','latex');
            set(groot,'defaultLegendInterpreter','latex');

            % Create the figure
            figure('units','pixels','position',[0 0 400 1440],'Color','w');
            set(gcf,'Visible','on'); % pop-out figure
            P = tiledlayout(P.grid(1),P.grid(2),'TileSpacing','tight','Padding','tight');

            C1.Layout_Obj = tiledlayout(P,C1.grid(1),C1.grid(2)); % tile layout child connection
            C1.Layout_Obj.Layout.Tile = C1.tile_start; C1.Layout_Obj.Layout.TileSpan = C1.grid;
        
            ax = nexttile(P,1); % dz__x
            contourf(ax,ai,aj,dz__x_sweep,cfLvl,'LineStyle','none','FaceAlpha',fA);
            axis equal tight; hold on; view(2);
            quiver(ax,ai(iQ,iQ),aj(iQ,iQ),dphi_x_sweep(iQ,iQ),dphi_y_sweep(iQ,iQ),0.5,'LineWidth',lW_V,'Color',gc_col);
            colormap(ax,CUB); clim(ax,C1_lim);
            set(get(ax,'YLabel'),'rotation',0,'VerticalAlignment','middle');
            title(ax,'$dz^{x}$','Color',gc_col,FontSize=titleFS);
            xlabel(ax,x_label_txt,FontSize=labelFS); 
            ylabel(ax,y_label_txt,FontSize=labelFS);
            xticks(ax, xtickval); yticks(ax, ytickval);
            xticklabels(ax, xticklab); yticklabels(ax, yticklab);
            ax.XAxis.FontSize = tickFS; ax.YAxis.FontSize = tickFS; 
            set(ax,'Color',col_backg);
            xlim(xlimits); ylim(ylimits);
            
            ax = nexttile(P,2); % dz__y
            contourf(ax,ai,aj,dz__y_sweep,cfLvl,'LineStyle','none','FaceAlpha',fA);
            axis equal tight; hold on; view(2);
            quiver(ax,ai(iQ,iQ),aj(iQ,iQ),dphi_x_sweep(iQ,iQ),dphi_y_sweep(iQ,iQ),0.5,'LineWidth',lW_V,'Color',gc_col);
            colormap(ax,CUB); clim(ax,C1_lim);
            set(get(ax,'YLabel'),'rotation',0,'VerticalAlignment','middle');
            title(ax,'$dz^{y}$','Color',gc_col,FontSize=titleFS);
            xticks(ax, xtickval); yticks(ax, ytickval);
            xticklabels(ax, xticklab); yticklabels(ax, yticklab);
            ax.XAxis.FontSize = tickFS; ax.YAxis.FontSize = tickFS; 
            set(ax,'Color',col_backg);
            xlim(xlimits); ylim(ylimits);
            
            ax = nexttile(P,3); % dz__\theta
            contourf(ax,ai,aj,dz__theta_sweep,cfLvl,'LineStyle','none','FaceAlpha',fA);
            axis equal tight; hold on; view(2);
            quiver(ax,ai(iQ,iQ),aj(iQ,iQ),dphi_x_sweep(iQ,iQ),dphi_y_sweep(iQ,iQ),0.5,'LineWidth',lW_V,'Color',gc_col);
            colormap(ax,CUB); clim(ax,C2_lim);
            set(get(ax,'YLabel'),'rotation',0,'VerticalAlignment','middle');
            title(ax,'$dz^{\theta}$','Color',gc_col,FontSize=titleFS);
            xticks(ax, xtickval); yticks(ax, ytickval);
            xticklabels(ax, xticklab); yticklabels(ax, yticklab);
            ax.XAxis.FontSize = tickFS; ax.YAxis.FontSize = tickFS; 
            set(ax,'Color',col_backg);
            xlim(xlimits); ylim(ylimits);
        
            title(P,sgtitle_txt,'Color',col(1,:),'Interpreter','latex','FontSize',sgtitleFS);


            % Just stratified panels were plotted, not 
            videoi = [];

        case 2 % stratified panels with trajectory+animation



    end

