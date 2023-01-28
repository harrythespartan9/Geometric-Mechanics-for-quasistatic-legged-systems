% This script helps with the motion planning of the level-2 kinematics of
% the 
function datai = qlevel2noslip_mp(datai)

    % If we have a gait, then it is a 
    if isfield(datai,'gait')
        cond = 2; 
    else
        cond = 1; 
    end

    % Unpack plotting information
%     lW_V = datai{1}.lW_V; iQ = datai{1}.iQ; % plot parameters
    lW_c = datai{1}.lW_contour; fA = datai{1}.fA; 
    cfLvl = datai{1}.cfLvl; cLvl = datai{1}.cLvl;
    gc_col = datai{1}.gc_col; col_backg = datai{1}.col_backg;
    col = datai{1}.col; CUB = datai{1}.CUB;
    titleFS = datai{1}.titleFS; tickFS = datai{1}.tickFS; cbarFS = datai{1}.cbarFS;
    labelFS = datai{1}.labelFS; sgtitleFS = datai{1}.sgtitleFS;
    sgtitle_txt = datai{1}.sgtitle_txt;
    x_label_txt = datai{1}.x_label_txt;
    y_label_txt = datai{1}.y_label_txt;
    xtickval = datai{1}.ytickval; ytickval = datai{1}.ytickval;
    xticklab = datai{1}.xticklab; yticklab = datai{1}.yticklab;
    xlimits = datai{1}.xlimits; ylimits = datai{1}.ylimits;
    ksq_lb =  datai{1}.ksq_lb;
    
    % sweep data
    ai = datai{2}.ai; aj = datai{2}.aj;
%     dphi_x_sweep = datai{2}.dphi_x_sweep; 
%     dphi_y_sweep = datai{2}.dphi_y_sweep;
    ksq_sweep = datai{2}.ksq_sweep;
    dz__x_sweep = datai{2}.dz__x_sweep;
    dz__y_sweep = datai{2}.dz__y_sweep;
    dz__theta_sweep = datai{2}.dz__theta_sweep;

    % Compute the limits on the data-- helps with countour plotting
    [C1_lim,C2_lim] = se2limits(dz__x_sweep,dz__y_sweep,dz__theta_sweep);

    % Create two child layouts-- one for translational panels and
    % one for rotation panel
    set(groot,'defaultAxesTickLabelInterpreter','latex'); 
    set(groot,'defaulttextinterpreter','latex');
    set(groot,'defaultLegendInterpreter','latex');

    % Compute everything you need to generate the child tiledlayout
    switch cond
        
        case 1 % just stratified panels
            
            % parent layout
            P = [];
            P.grid = [3 1];

            % gait constraint props to aid planning
            colP = datai{1}.colP;
            lS = datai{1}.lS;

        case 2 % stratified panels with trajectory+animation

            P = [];
            P.grid = [3 5];

    end
    
    % get the figure dimensions
    m = 2160*(P.grid(2))/5; % scaled figure x-resolution
    n = 1620; % fixed figure y-resolution

    % child 1-- dz_translation
    C1 = []; C1.limits = C1_lim; % previously defined color limits
    C1.start = [1, 1];
    C1.grid = [2, 1]; C1.num = prod(C1.grid);
    C1.tile_start = (C1.start(1)-1)*P.grid(2) + C1.start(2);
    C1.sweeptxt = {'dz__x_sweep', 'dz__y_sweep'};
    C1.titletxt = {'$dz^{x}$', '$dz^{y}$'};
    % child 2-- dz_theta
    C2 = []; C2.limits = C2_lim;
    C2.start = [3, 1];
    C2.grid = [1, 1]; C2.num = prod(C2.grid);
    C2.tile_start = (C2.start(1)-1)*P.grid(2) + C2.start(2);
    C2.sweeptxt = {'dz__theta_sweep'};
    C2.titletxt = {'$dz^{\theta}$'};

    % Create the figure
    figure('units','pixels','position',[0 0 m n],'Color','w');
    set(gcf,'Visible','on'); % pop-out figure
    P = tiledlayout(P.grid(1),P.grid(2),'TileSpacing','tight','Padding','tight');
    
    % Initialize the child layouts
    C1.Layout_Obj = tiledlayout(P,C1.grid(1),C1.grid(2)); 
    C1.Layout_Obj.Layout.Tile = C1.tile_start; C1.Layout_Obj.Layout.TileSpan = C1.grid;
    C2.Layout_Obj = tiledlayout(P,C2.grid(1),C2.grid(2)); 
    C2.Layout_Obj.Layout.Tile = C2.tile_start; C2.Layout_Obj.Layout.TileSpan = C2.grid;
    
    % Child 1
    ax = cell(1, C1.num);
    for i = 1:C1.num
        ax{i} = nexttile(C1.Layout_Obj,i); % dz__x & dz__y
        contourf(ax{i},ai,aj,datai{2}.(C1.sweeptxt{i}),cfLvl,'FaceAlpha',fA,'LineWidth',lW_c); %,'LineStyle','none'
        axis equal tight; hold on; view(2);
        if cond == 1
            contour(ax{i}, ai,aj,ksq_sweep,cLvl,lS,'LineWidth',lW_c+0.5,'EdgeColor',colP); % gait constraint contours for path planning
        end
        contour(ax{i}, ai,aj,ksq_sweep,[ksq_lb ksq_lb],'k--','LineWidth',lW_c); % exclusion zone
        colormap(ax{i},CUB); clim(ax{i},C1_lim);
        set(get(ax{i},'YLabel'),'rotation',0,'VerticalAlignment','middle');
        title(ax{i},C1.titletxt{i},'Color',gc_col,FontSize=titleFS);
        if i == 1
            xlabel(ax{i},x_label_txt,FontSize=labelFS); 
            ylabel(ax{i},y_label_txt,FontSize=labelFS);
        end
        xticks(ax{i}, xtickval); yticks(ax{i}, ytickval);
        xticklabels(ax{i}, xticklab); yticklabels(ax{i}, yticklab);
        ax{i}.XAxis.FontSize = tickFS; ax{i}.YAxis.FontSize = tickFS; 
        set(ax{i},'Color',col_backg);
        xlim(xlimits); ylim(ylimits);
    end
    C1.axes = ax;
    C1.colorB = colorbar(C1.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS); C1.colorB.Layout.Tile = 'South'; % plot the translation colorbar
    
    % Child 2
    ax = cell(1, C2.num);
    for i = 1:C2.num
        ax{i} = nexttile(C2.Layout_Obj,i); % just dz__\theta
        contourf(ax{i},ai,aj,datai{2}.(C1.sweeptxt{i}),cfLvl,'FaceAlpha',fA,'LineWidth',lW_c); % ,'LineStyle','none'
        axis equal tight; hold on; view(2);
        if cond == 1
            contour(ax{i}, ai,aj,ksq_sweep,cLvl,lS,'LineWidth',lW_c+0.5,'EdgeColor',colP); % gait constraint contours for path planning
        end
        contour(ax{i}, ai,aj,ksq_sweep,[ksq_lb ksq_lb],'--k','LineWidth',lW_c);
        colormap(ax{i},CUB); clim(ax{i},C2_lim);
        set(get(ax{i},'YLabel'),'rotation',0,'VerticalAlignment','middle');
        title(ax{i},C2.titletxt{i},'Color',gc_col,FontSize=titleFS);
        xticks(ax{i}, xtickval); yticks(ax{i}, ytickval);
        xticklabels(ax{i}, xticklab); yticklabels(ax{i}, yticklab);
        ax{i}.XAxis.FontSize = tickFS; ax{i}.YAxis.FontSize = tickFS; 
        set(ax{i},'Color',col_backg);
        xlim(xlimits); ylim(ylimits);
    end
    C2.axes = ax;
    C2.colorB = colorbar(C2.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS); C2.colorB.Layout.Tile = 'South'; % plot the translation colorbar
    
    switch cond

        case 1

            % label the figure
            title(P,sgtitle_txt,'Color',col(1,:),'Interpreter','latex','FontSize',sgtitleFS);

        case 2

    end

end

