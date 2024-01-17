% This script plots the net stratified panel of a 2-beat gait as a function of each reduced shape space separated vertically. This function requires the some
% utility functions in the 'freezeColors_utils' folder.
function plotNetPanelofNoslip2bgaits(datai, dataj, dataij)
    
    % unpack plotting data
    path_i = dataij.path_i; path_j = dataij.path_j;
    lW_V_i = datai{1}.lW_V; iQ_i = datai{1}.iQ;
    lW_c_i = datai{1}.lW_contour; fA_i = datai{1}.fA; 
    cfLvl_i = datai{1}.cfLvl; cLvl_i = datai{1}.cLvl;
    gc_col_i = datai{1}.gc_col; gc_col_j = dataj{1}.gc_col; col_backg_i = datai{1}.col_backg;
    col_i = datai{1}.col; CUB_i = datai{1}.CUB;
    titleFS_i = datai{1}.titleFS; tickFS_i = datai{1}.tickFS; cbarFS_i = datai{1}.cbarFS;
    labelFS_i = datai{1}.labelFS; sgtitleFS_i = datai{1}.sgtitleFS;
    vidF_i = datai{6};
    path_i = datai{5};
    pltkin_i = datai{4};
    a_i = datai{3}.aa; l_i = datai{3}.ll;
    lW_i = datai{1}.lW;
    lW_r_i = datai{1}.lW_r;
    lW_kq_i = datai{1}.lW_kq;
    lW_b_i = datai{1}.lW_b;
    lW_qf_i = datai{1}.lW_qf;
    circS_i = datai{1}.circS;
    frame_scale_i = 0.25;
    c_i = path_i.path_active_color;
    zx_i = path_i.net_displacement(1,:);
    zy_i = path_i.net_displacement(2,:);
    ztheta_i = path_i.net_displacement(3,:);
    dnum_i = datai{3}.dnum;
    dnum_j = datai{3}.dnum;
    cs_idx_i = datai{3}.cs_idx;
    ank = datai{3}.ank;
    cs_i = datai{3}.cs;
    cs_j = dataj{3}.cs;
    xlimits = datai{1}.xlimits; ylimits = datai{1}.ylimits;

    
    % unpack data for plotting
    u_i_stpt = dataij.u(1);     % set-points for the gait construction
    u_j_stpt = dataij.u(2);
    u_i = dataij.u_i;           % arrays
    u_j = dataij.u_j;
    u_i_S = dataij.u_i_S;       % 2D arrays/sweeps
    u_j_S = dataij.u_j_S;
    % idxLoc = (fliplr(u_i_S) == u_i_stpt) & (flipud(u_j_S) == u_j_stpt); % index location of the current setpoint
    idxLoc = (u_i_S == u_i_stpt) & (u_j_S == u_j_stpt);
    kappa_S = dataij.kappa_S;
    gaits = dataij.gaits;       % ui, uj swept gaits
    
    % get the axis limits
    ui_lim = [min(u_i) max(u_i)];
    uj_lim = [min(u_j) max(u_j)];

    % get the net displacement information as a function of the input space
    zxu = nan(numel(u_i), numel(u_j)); zyu = zxu; zthetau = zxu;
    for idxi = 1:numel(u_i)
        for idxj = 1:numel(u_j)
            zxu(idxi, idxj)     = gaits{idxi, idxj}.trajectory{15};
            zyu(idxi, idxj)     = gaits{idxi, idxj}.trajectory{16};
            zthetau(idxi, idxj) = gaits{idxi, idxj}.trajectory{17};
        end
    end
    z.zxu = zxu; z.zyu = zyu; z.zthetau = zthetau;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % gait requirements and other vars
    dnum = 5;
    idxi = dataij.u_i == dataij.u(1);
    idxj = dataij.u_j == dataij.u(2);
    Qi   = path_i.open_trajectory;
    Qdoti= path_i.open_trajectory_vel;
    Qj   = path_j.open_trajectory;
    Qdotj= path_j.open_trajectory_vel;
    topSurfLvl = 1;                      % set the level of the top surface here

    % compute the full strat panel and stance paths
    stancei = [Qi{idxi}{5}; 
               Qi{idxi}{6}];
    veli = [Qdoti{idxi}{2}; 
            Qdoti{idxi}{3};
            Qdoti{idxi}{4}];
    stancej = [Qj{idxj}{5}; 
               Qj{idxj}{6}];
    velj = [Qdotj{idxj}{2}; 
            Qdotj{idxj}{3};
            Qdotj{idxj}{4}]; % stance paths and body velocity along those paths
    fullStrat = veli + fliplr(velj); % compute the panels as the sum of the individual panels
    
    xlimits = datai{1}.xlimits; ylimits = datai{1}.ylimits; zlimits = [-0.1, topSurfLvl+0.1]; % limits for the plotting box
    
    % surfaces at {B_ij, S_ij} for each cs (2d redu. spaces at each 2-beat stance phase)
    surfSi = cell(1, 4); surfSj = surfSi;
    surfSi{1} = linspace(xlimits(1), xlimits(2), dnum); surfSi{2} = linspace(ylimits(1), ylimits(2), dnum); 
    surfSj{1} = linspace(xlimits(1), xlimits(2), dnum); surfSj{2} = linspace(ylimits(1), ylimits(2), dnum); 
    [surfSi{1}, surfSi{2}] = meshgrid(surfSi{1}, surfSi{2}); surfSi{3} = zeros(size(surfSi{2})); % the surfaces @ijth cs at 0
    [surfSj{1}, surfSj{2}] = meshgrid(surfSj{1}, surfSj{2}); surfSj{3} = topSurfLvl*ones(size(surfSj{2})); %              @klth cs at 1
    surfSi{4} = repmat( reshape(gc_col_i, [1, 1, 3]), size(surfSi{1}) );
    surfSj{4} = repmat( reshape(gc_col_j, [1, 1, 3]), size(surfSj{1}) ); % colors for each surface

    % compute the total panels and related stuff
    [cIxy,cIth] = se2limits(datai{2}.dz__x_sweep, datai{2}.dz__y_sweep, datai{2}.dz__theta_sweep);
    [cJxy,cJth] = se2limits(dataj{2}.dz__x_sweep, dataj{2}.dz__y_sweep, dataj{2}.dz__theta_sweep);
    C1_lim = cIxy + cJxy;
    C2_lim = cIth + cJth; % color limits for the axes

    % compute the full stratified panel surface function for plot call
    fullSurf = fullStratSurfcompute(stancei, stancej, fullStrat, {{C1_lim, C2_lim}, CUB_i}, topSurfLvl);

    % compute the azimuthal and elevation angles of "view" based on the surface normal
    [Nx, Ny, Nz] = surfnorm(fullSurf.X, fullSurf.Y, fullSurf.Z); 
    % az = mean(  90 - atan2d(Ny, Nx), "all"  );    % "mean surface normal" view
    az = mean(  atan2d(Ny, Nx), "all"  ) - 9;           % "curvature" view
    el = mean(  -atan2d(Nz, real(sqrt(Nx'*Nx + Ny'*Ny))), "all"  ) + 15; % a slightly elevated view % 10 (mine) % 15 (Kaushik's)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % setting latex as the default interpreter
    set(groot,'defaultAxesTickLabelInterpreter','latex'); 
    set(groot,'defaulttextinterpreter','latex');
    set(groot,'defaultLegendInterpreter','latex');

    % parent layout
    P = [];
    P.grid = [3 1];
    
    % get the figure dimensions
    m = 2200*(P.grid(2))/5; % scaled figure x-resolution % 2200 % 1800
    n = 1500; % fixed figure y-resolution % 1800 (previously) % 1500 % 900

    % child 1-- ith dz_translation
    C1 = []; C1.limits = C1_lim; % previously defined color limits
    C1.start = [1, 1];
    C1.grid = [2, 1]; C1.num = prod(C1.grid); C1.span_grid = C1.grid; % the tiles in the child layout matches the parent
    C1.tile_start = (C1.start(1)-1)*P.grid(2) + C1.start(2);
    C1.sweeptxt = {'x', 'y'};
    C1.titletxt = {'$$dz^{x}$$', '$$dz^{y}$$'};

    % child 2-- ith dz_theta
    C2 = []; C2.limits = C2_lim;
    C2.start = [3, 1];
    C2.grid = [1, 1]; C2.num = prod(C2.grid); C2.span_grid = C2.grid;
    C2.tile_start = (C2.start(1)-1)*P.grid(2) + C2.start(2);
    C2.sweeptxt = {'theta'};
    C2.titletxt = {'$$dz^{\theta}$$'};

    % Create the figure
    figure('units','pixels','position',[100 -200 m n],'Color','w');
    set(gcf,'Visible','on'); % pop-out figure
    P = tiledlayout(P.grid(1),P.grid(2),'TileSpacing','tight','Padding','tight');

    % Initialize the child layouts
    C1.Layout_Obj = tiledlayout(P,C1.grid(1),C1.grid(2),'TileSpacing','tight','Padding','tight'); 
    C1.Layout_Obj.Layout.Tile = C1.tile_start; C1.Layout_Obj.Layout.TileSpan = C1.span_grid;
    C2.Layout_Obj = tiledlayout(P,C2.grid(1),C2.grid(2),'TileSpacing','tight','Padding','tight'); 
    C2.Layout_Obj.Layout.Tile = C2.tile_start; C2.Layout_Obj.Layout.TileSpan = C2.span_grid;

    % Child 1
    ax = cell(1, C1.num); count_idx = 1;
    for i = 1:C1.num
        ax{i} = nexttile(C1.Layout_Obj, i); % dz^x & dz^y
        surf(ax{i}, surfSi{1}, surfSi{2}, surfSi{3}, surfSi{4}, 'LineStyle', 'none','FaceAlpha', 0.1);
        axis equal tight; hold on; view(az, el);
        surf(ax{i}, surfSj{1}, surfSj{2}, surfSj{3}, surfSj{4}, 'LineStyle', 'none','FaceAlpha', 0.1); % surfs at each level
        surf(ax{i}, fullSurf.X, fullSurf.Y, fullSurf.Z, fullSurf.C{count_idx}, 'LineStyle', 'none'); % full stratified panel surface
        colormap(ax{i}, CUB_i); clim(ax{i}, C1_lim);
        plot3(ax{i}, stancei(1,:), stancei(2,:), zeros(size(stancei, 2)), '-', 'Color', gc_col_i, 'LineWidth', lW_c_i+1);
        plot3(ax{i}, stancej(1,:), stancej(2,:), topSurfLvl*ones(size(stancej, 2)), '-', 'Color', gc_col_j, 'LineWidth', lW_c_i+1); % stance paths
        scatter3(ax{i}, stancei(1,end), stancei(2,end), 0, circS_i, gc_col_i, 'filled');
        scatter3(ax{i}, stancej(1,end), stancej(2,end), topSurfLvl, circS_i, gc_col_j, 'filled'); % stance scatters at the end
        set(get(ax{i},'YLabel'),'rotation',0,'VerticalAlignment','middle');
        title(ax{i},C1.titletxt{i},'Color','k',FontSize=titleFS_i);
        % if i == 1
        %     xlabel(ax{i},'$$\alpha_{i}$$','Color','k',FontSize=labelFS_i); 
        %     ylabel(ax{i},'$$\alpha_{j}$$','Color','k',FontSize=labelFS_i);
        % end
        set(ax{i}, 'XTick', []); set(ax{i}, 'YTick', []); set(ax{i}, 'ZTick', []); set(ax{i},'Color',col_backg_i);
        xlim(xlimits); ylim(ylimits); zlim(zlimits);
        count_idx = count_idx + 1;
    end
    C1.axes = ax;
    C1.colorB = colorbar(C1.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS_i); C1.colorB.Layout.Tile = 'South'; % plot the translation colorbar

    % Child 2
    ax = cell(1, C2.num);
    for i = 1:C2.num
        ax{i} = nexttile(C2.Layout_Obj,i); % just z__\theta
        surf(ax{i}, surfSi{1}, surfSi{2}, surfSi{3}, surfSi{4}, 'LineStyle', 'none','FaceAlpha', 0.1);
        axis equal tight; hold on; view(az, el);
        surf(ax{i}, surfSj{1}, surfSj{2}, surfSj{3}, surfSj{4}, 'LineStyle', 'none','FaceAlpha', 0.1);
        surf(ax{i}, fullSurf.X, fullSurf.Y, fullSurf.Z, fullSurf.C{count_idx}, 'LineStyle', 'none');
        colormap(ax{i}, CUB_i); clim(ax{i}, C1_lim);
        plot3(ax{i}, stancei(1,:), stancei(2,:), zeros(size(stancei, 2)), '-', 'Color', gc_col_i, 'LineWidth', lW_c_i+1);
        plot3(ax{i}, stancej(1,:), stancej(2,:), topSurfLvl*ones(size(stancej, 2)), '-', 'Color', gc_col_j, 'LineWidth', lW_c_i+1);
        scatter3(ax{i}, stancei(1,end), stancei(2,end), 0, circS_i, gc_col_i, 'filled');
        scatter3(ax{i}, stancej(1,end), stancej(2,end), topSurfLvl, circS_i, gc_col_j, 'filled');
        colormap(ax{i}, CUB_i); clim(ax{i}, C2_lim);  
        set(get(ax{i},'YLabel'),'rotation',0,'VerticalAlignment','middle');
        title(ax{i},C2.titletxt{i},'Color','k',FontSize=titleFS_i);
        set(ax{i}, 'XTick', []); set(ax{i}, 'YTick', []); set(ax{i}, 'ZTick', []); set(ax{i},'Color',col_backg_i);
        xlim(xlimits); ylim(ylimits); zlim(zlimits);
        count_idx = count_idx + 1;
    end
    C2.axes = ax;
    C2.colorB = colorbar(C2.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS_i); C2.colorB.Layout.Tile = 'South'; % plot the rotation colorbar

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % % Create the figure
    % figure('units','pixels','position',[100 -200 m n],'Color','w');
    % set(gcf,'Visible','on'); % pop-out figure
    % tiledlayout(3, 1,'TileSpacing','tight','Padding','tight');
    % 
    % for count_idx = 1:3
    % 
    %     axs = nexttile();
    %     contourf(repmat(fullStrat(count_idx, :), [size(fullStrat, 2) 1]), size(fullStrat, 2), 'LineStyle', 'none');
    %     axis square equal; colormap(axs, CUB_i); clim(axs, C1_lim);
    %     set(axs, 'XTick', []); set(axs, 'YTick', []);
    % 
    % end


end