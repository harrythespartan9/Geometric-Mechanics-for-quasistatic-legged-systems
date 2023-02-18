% This script helps with the motion planning of the level-2 kinematics of
% the no-slip, quadrupedal robot.
function [datai, dataj, dataij] = qlevel2noslip_mp(datai, dataj)

    % Check which case we are in
    switch nargin
        case 1
            if numel(datai) == 3
                cond = 1;
            elseif numel(datai) == 5
                cond = 2;
            else
                error('ERROR: Data structure should have 3 or 5 fields only!');
            end
        case 2
            if numel(datai) ~= 5 || numel(dataj) ~= 5
                error('ERROR: Both data structures should have 5 fields only!');
            else
                cond = 3;
            end
    end
    % if cond is not initialized
    if ~exist('cond', 'var')
        error('ERROR: incorrect arguments for the function.');
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
        
        case 1 % just infinitesimal disps
            
            % parent layout
            P = [];
            P.grid = [3 1];

            % path constraint props to aid planning
            colP = datai{1}.colP;
            lS = datai{1}.lS;

        case 2 % infinitesimal disps with trajectory + animation

            P = [];
            P.grid = [3 6];

            % child 3-- SE(2) trajectory and net displacements
            C3 = [];
            C3.start = [1, 5];
            C3.grid = [3, 1];  C3.num = prod(C3.grid);
            C3.span_grid = C3.grid;
            C3.tile_start = (C3.start(1)-1)*P.grid(2) + C3.start(2);
            C3.titletxt = '$b(\psi)$';
            % child 4--  SE(2) net displacements
            C4 = [];
            C4.start = [1, 6]; 
            C4.grid = [3, 1]; C4.num = prod(C4.grid);
            C4.span_grid = C4.grid;
            C4.tile_start = (C4.start(1)-1)*P.grid(2) + C4.start(2);
            C4.titletxt = '$z(\psi)$';

            % Unpack plotting tools
            lW_s = datai{1}.lW_s;


    end
    
    % get the figure dimensions
    m = 2160*(P.grid(2))/6; % scaled figure x-resolution
    n = 1620; % fixed figure y-resolution

    % child 1-- dz_translation
    C1 = []; C1.limits = C1_lim; % previously defined color limits
    C1.start = [1, 1];
    C1.grid = [2, 1]; C1.num = prod(C1.grid); C1.span_grid = C1.grid; % the tiles in the child layout matches the parent
    C1.tile_start = (C1.start(1)-1)*P.grid(2) + C1.start(2);
    C1.sweeptxt = {'dz__x_sweep', 'dz__y_sweep'};
    C1.titletxt = {'$dz^{x}_{\psi}$', '$dz^{y}_{\psi}$'};
    % child 2-- dz_theta
    C2 = []; C2.limits = C2_lim;
    C2.start = [3, 1];
    C2.grid = [1, 1]; C2.num = prod(C2.grid); C2.span_grid = C2.grid;
    C2.tile_start = (C2.start(1)-1)*P.grid(2) + C2.start(2);
    C2.sweeptxt = {'dz__theta_sweep'};
    C2.titletxt = {'$dz^{\theta}_{\psi}$'};

    % Create the figure
    f = figure('units','pixels','position',[0 0 m n],'Color','w');
    set(gcf,'Visible','on'); % pop-out figure
    P = tiledlayout(P.grid(1),P.grid(2),'TileSpacing','tight','Padding','tight');
    
    % Initialize the child layouts
    C1.Layout_Obj = tiledlayout(P,C1.grid(1),C1.grid(2),'TileSpacing','tight','Padding','tight'); 
    C1.Layout_Obj.Layout.Tile = C1.tile_start; C1.Layout_Obj.Layout.TileSpan = C1.span_grid;
    C2.Layout_Obj = tiledlayout(P,C2.grid(1),C2.grid(2),'TileSpacing','tight','Padding','tight'); 
    C2.Layout_Obj.Layout.Tile = C2.tile_start; C2.Layout_Obj.Layout.TileSpan = C2.span_grid;
    
    % Child 1
    ax = cell(1, C1.num);
    for i = 1:C1.num
        ax{i} = nexttile(C1.Layout_Obj,i); % dz__x & dz__y
        if cond == 1
            contourf(ax{i},ai,aj,datai{2}.(C1.sweeptxt{i}),cfLvl,'FaceAlpha',fA,'LineWidth',lW_c);
            axis equal tight; hold on; view(2);
            contour(ax{i}, ai,aj,ksq_sweep,cLvl,lS,'LineWidth',lW_c+0.5,'EdgeColor',colP); % path constraint contours for path planning
        else
            contourf(ax{i},ai,aj,datai{2}.(C1.sweeptxt{i}),cfLvl,'FaceAlpha',fA,'LineWidth',lW_c,'LineStyle','none'); %,'LineStyle','none'
            axis equal tight; hold on; view(2);
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
    if cond == 1
        C1.colorB = colorbar(C1.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS); C1.colorB.Layout.Tile = 'South'; % plot the translation colorbar
    end
    
    % Child 2
    ax = cell(1, C2.num);
    for i = 1:C2.num
        ax{i} = nexttile(C2.Layout_Obj,i); % just dz__\theta
        if cond == 1
            contourf(ax{i},ai,aj,datai{2}.(C2.sweeptxt{i}),cfLvl,'FaceAlpha',fA,'LineWidth',lW_c);
            axis equal tight; hold on; view(2);
            contour(ax{i}, ai,aj,ksq_sweep,cLvl,lS,'LineWidth',lW_c+0.5,'EdgeColor',colP); % path constraint contours for path planning
        else
            contourf(ax{i},ai,aj,datai{2}.(C2.sweeptxt{i}),cfLvl,'FaceAlpha',fA,'LineWidth',lW_c,'LineStyle','none');
            axis equal tight; hold on; view(2);
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
    if cond == 1
        C2.colorB = colorbar(C2.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS); C2.colorB.Layout.Tile = 'South'; % plot the translation colorbar
    end
    
    switch cond

        case 1

            % label the figure
            title(P,sgtitle_txt,'Color',col(1,:),'Interpreter','latex','FontSize',sgtitleFS);

            % return empty out strcuts
            dataj = []; dataij = [];

        case 2

            % Obtain the path information
            path = datai{5};
            pltkin = datai{4};
            a = datai{3}.aa; l = datai{3}.ll;
            cs = datai{3}.cs;
            lW = datai{1}.lW;
            lW_r = datai{1}.lW_r;
            lW_kq = datai{1}.lW_kq;
            lW_b = datai{1}.lW_b;
            lW_qf = datai{1}.lW_qf;
            circS = datai{1}.circS;
            frame_scale = 0.25;
            c = path.path_active_color;
            zx = path.net_displacement(1,:);
            zy = path.net_displacement(2,:);
            ztheta = path.net_displacement(3,:);
            dnum = datai{3}.dnum; 
            cs_idx = datai{3}.cs_idx;

            % path scaling vector (the main path is sclaed)
            pscale = 10*(1:10);

            % initialize the tiled layouts
            C3.Layout_Obj = tiledlayout(P,C3.grid(1),C3.grid(2),'TileSpacing','tight','Padding','tight'); 
            C3.Layout_Obj.Layout.Tile = C3.tile_start; C3.Layout_Obj.Layout.TileSpan = C3.span_grid;
            C4.Layout_Obj = tiledlayout(P,C4.grid(1),C4.grid(2),'TileSpacing','tight','Padding','tight'); 
            C4.Layout_Obj.Layout.Tile = C4.tile_start; C4.Layout_Obj.Layout.TileSpan = C4.span_grid;
            
            % compute the span of the animation in world coordinates-- for the largest path
            xM = path.open_trajectory{10}{2}; yM = path.open_trajectory{10}{3};
            bl = 4*l; % body length of the robot-- need to add this to the rigid robot properties
            thresh = 1; % one body length
            lim_temp = max( [min(xM, [], 'all')-thresh*bl, max(xM, [], 'all')+thresh*bl, min(yM, [], 'all')-thresh*bl, max(yM, [], 'all')+thresh*bl]...
                , [], 'all' )* [-1, 1, -1, 1]; % 2* body length margin

            % Video stuff
            video = VideoWriter(strcat('data_animation','.mp4'),'MPEG-4');
            video.FrameRate = 60;
            video.Quality = 100;
            open(video);
            
            % Set the main title
            title(P,sgtitle_txt,'Color',col(1,:),'Interpreter','latex','FontSize',sgtitleFS);

            % Create a structure to hold the final frame information for each path scaling
            skp = 3; 
            skp_path = 1:skp:numel(path.open_trajectory);
            datai{1}.skp_path = skp_path;
            F = cell(1, numel(skp_path));

            % Iterate over each open trajectory case
            for i = skp_path % plotting 10%, 40%, 70%, and 100%

                % Unpack the path data for the current trajectory
                t      = path.open_trajectory{i}{1};
                x      = path.open_trajectory{i}{2};
                y      = path.open_trajectory{i}{3};
                theta  = path.open_trajectory{i}{4};
                ai     = path.open_trajectory{i}{5};
                aj     = path.open_trajectory{i}{6};
%                 numD    = numel(t);
                
                % angle of each limb
                alp = cell(1,4); % store the angle trajectory
                for j = 1:4
                    if sum(j == cs) == 0
                        alp{j} = zeros(size(ai));
                    else
                        switch find(j == cs)
                            case 1
                                alp{j} = ai;
                            case 2
                                alp{j} = aj;
                        end
                    end
                end

                % Child 1-- add the shape space trajectory on top
                h1 = cell(1,C1.num); 
%                 h1_A = cell(1,C1.num);
                for j = 1:C1.num
                    if i ~= 1
                        delete(h1_s{j}); delete(h1{j}); % delete from previous path
                    end
                    h1{j} = plot(C1.axes{j}, ai, aj, 'LineWidth', lW_s, 'Color', c);
%                     h1_A{j} = quiver( C1.axes{j}, ai(ceil(numD/2)-1), aj(ceil(numD/2)-1), ai(ceil(numD/2))-ai(ceil(numD/2)-1),...
%                             aj(ceil(numD/2))-aj(ceil(numD/2)-1), 'LineWidth', lW_s, 'Color', c, 'MaxHeadSize', 1 );
                end

                % Child 2-- same as above
                h2 = cell(1,C2.num); 
%                 h2_A = cell(1,C2.num);
                for j = 1:C2.num
                    if i ~= 1
                        delete(h2_s{j}); delete(h2{j});
                    end
                    h2{j} = plot(C2.axes{j}, ai, aj, 'LineWidth', lW_s, 'Color', c);
%                     h2_A{j} = quiver( C2.axes{j}, ai(ceil(numD/2)-1), aj(ceil(numD/2)-1), ai(ceil(numD/2))-ai(ceil(numD/2)-1),...
%                             aj(ceil(numD/2))-aj(ceil(numD/2)-1), 'LineWidth', lW_s, 'Color', c, 'MaxHeadSize', 1 );
                end
                
                % if this is the first frame, create the child 3 plots
                if i == 1
                    
                    % limit thresholds for se(2) trajectories
                    lowxy = 0.05; % 5% of the leg length
                    lowtheta = deg2rad(3); % 3 degrees of orientation change

                    % Child 3-- SE(2) trajectories
                    [lim_bxy, lim_btheta] = se2limits(x, y, theta); % limits
                    if lim_bxy(2) < lowxy
                        lim_bxy = 0.1*[-1, 1];
                    end
                    if lim_btheta(2) < lowtheta
                        lim_btheta = lowtheta*[-1, 1];
                    end
                    ax = cell(1,C3.num);
                    for j = 1:C3.num
                        ax{j} = nexttile(C3.Layout_Obj, j);
                        switch j
                            case 1
                                bx = plot(ax{j}, t, x, '-', 'LineWidth', lW_s, 'Color', c);
                                ylabel(ax{j}, '$b^x$', FontSize=labelFS); ylim(ax{j}, lim_bxy);
                            case 2
                                by = plot(ax{j}, t, y, '-', 'LineWidth', lW_s, 'Color', c);
                                ylabel(ax{j}, '$b^y$', FontSize=labelFS); ylim(ax{j}, lim_bxy);
                            case 3
                                btheta = plot(ax{j}, t, theta, '-', 'LineWidth', lW_s, 'Color', c);
                                ylabel(ax{j}, '$b^{\theta}$', FontSize=labelFS);
                                xlabel(ax{j}, '$t$', FontSize=labelFS); ylim(ax{j}, lim_btheta);
                        end
                        set(get(ax{j},'YLabel'),'rotation',0,'VerticalAlignment','middle');
                        ax{j}.XAxis.FontSize = tickFS-5; ax{j}.YAxis.FontSize = tickFS-5;
                        grid on; hold on; axis square; xlim(ax{j}, [min(t) max(t)]);
                    end
                    title(C3.Layout_Obj, C3.titletxt, 'Color', gc_col, 'Interpreter', 'latex', FontSize=titleFS);
                    C3.axes = ax;

                    % Child 4-- SE(2) net displacements
                    [lim_zxy, lim_ztheta] = se2limits(zx, zy, ztheta);
                    if lim_zxy(2) < lowxy
                        lim_zxy = lowxy*[-1, 1];
                    end
                    if lim_ztheta(2) < lowtheta
                        lim_ztheta = lowtheta*[-1, 1];
                    end
                    ax = cell(1,C4.num);
                    for j = 1:C4.num
                        ax{j} = nexttile(C4.Layout_Obj, j);
                        switch j
                            case 1
                                plot(ax{j}, pscale, zx, '-', 'LineWidth', lW_s, 'Color', c);
                                grid on; hold on;
                                zxi = scatter(ax{j}, pscale(i), zx(i), circS, c, 'filled');
                                ylabel(ax{j}, '$z^x$', FontSize=labelFS); ylim(lim_zxy);
                            case 2
                                plot(ax{j}, pscale, zy, '-', 'LineWidth', lW_s, 'Color', c);
                                grid on; hold on; axis square;
                                zyi = scatter(ax{j}, pscale(i), zy(i), circS, c, 'filled');
                                ylabel(ax{j}, '$z^y$', FontSize=labelFS); ylim(lim_zxy);
                            case 3
                                plot(ax{j}, pscale, ztheta, '-', 'LineWidth', lW_s, 'Color', c);
                                grid on; hold on; axis square;
                                zthetai = scatter(ax{j}, pscale(i), ztheta(i), circS, c, 'filled');
                                ylabel(ax{j}, '$z^{\theta}$', FontSize=labelFS);
                                xlabel(ax{j}, 'Path scaling (\%)', FontSize=labelFS);  ylim(lim_ztheta);
                        end
                        set(get(ax{j},'YLabel'),'rotation',0,'VerticalAlignment','middle'); axis square;
                        ax{j}.XAxis.FontSize = tickFS-5; ax{j}.YAxis.FontSize = tickFS-5;
                        xlim([min(pscale) max(pscale)]);
                    end
                    title(C4.Layout_Obj, C4.titletxt, 'Color', gc_col, 'Interpreter', 'latex', FontSize=titleFS);
                    C4.axes = ax;

                else
                    
                    % remove plots
%                     delete(bx, by, btheta,... % clear the last path's SE(2) trajectory
%                         bxt, byt, bthetat,... % clear the last path's SE(2) scatter at the final time step
%                         zxi, zyi, zthetai); % clear the SE(2) net displacement scatter for previous path
                    delete(bx);
                    delete(by);
                    delete(btheta);
                    delete(bxt);
                    delete(byt);
                    delete(bthetat);
                    delete(zxi);
                    delete(zyi);
                    delete(zthetai);

                    % Child 3-- plot the current SE(2)
                    [lim_bxy, lim_btheta] = se2limits(x, y, theta);
                    if lim_bxy(2) < lowxy
                        lim_bxy = 0.1*[-1, 1];
                    end
                    if lim_btheta(2) < lowtheta
                        lim_btheta = lowtheta*[-1, 1];
                    end
                    for j = 1:C3.num
                        switch j
                            case 1
                                bx = plot(C3.axes{j}, t, x, '-', 'LineWidth', lW_s, 'Color', c); ylim(C3.axes{j}, lim_bxy);
                            case 2
                                by = plot(C3.axes{j}, t, y, '-', 'LineWidth', lW_s, 'Color', c); ylim(C3.axes{j}, lim_bxy);
                            case 3
                                btheta = plot(C3.axes{j}, t, theta, '-', 'LineWidth', lW_s, 'Color', c); ylim(C3.axes{j}, lim_btheta);
                        end
                        xlim(C3.axes{j}, [min(t) max(t)]);
                    end

                    % Child 4-- scatter the current net SE(2) displacement
                    for j = 1:C4.num
                        switch j
                            case 1
                                zxi = scatter(C4.axes{j}, pscale(i), zx(i), circS, c, 'filled');
                            case 2
                                zyi = scatter(C4.axes{j}, pscale(i), zy(i), circS, c, 'filled');
                            case 3
                                zthetai = scatter(C4.axes{j}, pscale(i), ztheta(i), circS, c, 'filled');
                        end
                    end


                end

                % Animation and updates to other plots ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                
                % compute everything you need to plot the system with
                            % stuff pertaining to the specific contact state
                leg_i__x = pltkin.(['legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_x'])(a, l, ai, x, y, theta); leg_i__x = [leg_i__x(1:dnum); leg_i__x(dnum+1:end)];
                leg_i__y = pltkin.(['legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_y'])(a, l, ai, x, y, theta); leg_i__y = [leg_i__y(1:dnum); leg_i__y(dnum+1:end)];
                leg_j__x = pltkin.(['legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_x'])(a, l, aj, x, y, theta); leg_j__x = [leg_j__x(1:dnum); leg_j__x(dnum+1:end)];
                leg_j__y = pltkin.(['legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_y'])(a, l, aj, x, y, theta); leg_j__y = [leg_j__y(1:dnum); leg_j__y(dnum+1:end)]; % ik it is duplicate arrays, easiest way for now
                legtip_i__x = pltkin.(['leg' num2str(cs(1)) '_x'])(a, l, ai, x, y, theta);
                legtip_i__y = pltkin.(['leg' num2str(cs(1)) '_y'])(a, l, ai, x, y, theta);
                legtip_j__x = pltkin.(['leg' num2str(cs(2)) '_x'])(a, l, aj, x, y, theta); 
                legtip_j__y = pltkin.(['leg' num2str(cs(2)) '_y'])(a, l, aj, x, y, theta);
                mean_leg_i__x = pltkin.(['legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_x'])(a, l, zeros(1,numel(x)), x, y, theta); mean_leg_i__x = [mean_leg_i__x(1:dnum); mean_leg_i__x(dnum+1:end)];
                mean_leg_i__y = pltkin.(['legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_y'])(a, l, zeros(1,numel(x)), x, y, theta); mean_leg_i__y = [mean_leg_i__y(1:dnum); mean_leg_i__y(dnum+1:end)];
                mean_leg_j__x = pltkin.(['legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_x'])(a, l, zeros(1,numel(x)), x, y, theta); mean_leg_j__x = [mean_leg_j__x(1:dnum); mean_leg_j__x(dnum+1:end)];
                mean_leg_j__y = pltkin.(['legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_y'])(a, l, zeros(1,numel(x)), x, y, theta); mean_leg_j__y = [mean_leg_j__y(1:dnum); mean_leg_j__y(dnum+1:end)];
                ksqij__x = pltkin.(['k_leg' num2str(cs(1)) '_leg' num2str(cs(2)) '_x'])(a, l, ai, aj, x, y, theta); ksqij__x = [ksqij__x(1:dnum); ksqij__x(dnum+1:end)];
                ksqij__y = pltkin.(['k_leg' num2str(cs(1)) '_leg' num2str(cs(2)) '_y'])(a, l, ai, aj, x, y, theta); ksqij__y = [ksqij__y(1:dnum); ksqij__y(dnum+1:end)];
                            % standard stuff
                leg_1__x = pltkin.legbase1_leg1_x(a, l, alp{1}, x, y, theta); leg_1__x = [leg_1__x(1:dnum); leg_1__x(dnum+1:end)];
                leg_1__y = pltkin.legbase1_leg1_y(a, l, alp{1}, x, y, theta); leg_1__y = [leg_1__y(1:dnum); leg_1__y(dnum+1:end)];
                leg_2__x = pltkin.legbase2_leg2_x(a, l, alp{2}, x, y, theta); leg_2__x = [leg_2__x(1:dnum); leg_2__x(dnum+1:end)];
                leg_2__y = pltkin.legbase2_leg2_y(a, l, alp{2}, x, y, theta); leg_2__y = [leg_2__y(1:dnum); leg_2__y(dnum+1:end)];
                leg_3__x = pltkin.legbase3_leg3_x(a, l, alp{3}, x, y, theta); leg_3__x = [leg_3__x(1:dnum); leg_3__x(dnum+1:end)];
                leg_3__y = pltkin.legbase3_leg3_y(a, l, alp{3}, x, y, theta); leg_3__y = [leg_3__y(1:dnum); leg_3__y(dnum+1:end)];
                leg_4__x = pltkin.legbase4_leg4_x(a, l, alp{4}, x, y, theta); leg_4__x = [leg_4__x(1:dnum); leg_4__x(dnum+1:end)];
                leg_4__y = pltkin.legbase4_leg4_y(a, l, alp{4}, x, y, theta); leg_4__y = [leg_4__y(1:dnum); leg_4__y(dnum+1:end)];
                top__x = pltkin.topright2topleft_x(l, x, y, theta); top__x = [top__x(1:dnum); top__x(dnum+1:end)];
                top__y = pltkin.topright2topleft_y(l, x, y, theta); top__y = [top__y(1:dnum); top__y(dnum+1:end)];
                left__x = pltkin.topleft2botleft_x(l, x, y, theta); left__x = [left__x(1:dnum); left__x(dnum+1:end)];
                left__y = pltkin.topleft2botleft_y(l, x, y, theta); left__y = [left__y(1:dnum); left__y(dnum+1:end)];
                bot__x = pltkin.botleft2botright_x(l, x, y, theta); bot__x = [bot__x(1:dnum); bot__x(dnum+1:end)];
                bot__y = pltkin.botleft2botright_y(l, x, y, theta); bot__y = [bot__y(1:dnum); bot__y(dnum+1:end)];
                right__x = pltkin.botright2topright_x(l, x, y, theta); right__x = [right__x(1:dnum); right__x(dnum+1:end)];
                right__y = pltkin.botright2topright_y(l, x, y, theta); right__y = [right__y(1:dnum); right__y(dnum+1:end)];
                body__x = pltkin.body_x(x, y, theta); 
                body__y = pltkin.body_y(x, y, theta);
                bodyf__x = pltkin.bodyf_x(x, y, theta); 
                bodyf__y = pltkin.bodyf_y(x, y, theta);
                
                % Child 5-- Animation of the SE(2) trajectory
                if i == 1
                    axA = nexttile(P, [3, 3]); 
                else
                    for k = 1:numel(hA)
                        delete(hA{k});
                    end
                end
                xline(axA, 0, ':', 'LineWidth', 0.5, 'Color', 'k'); 
                axis equal square; hold on; 
                set(axA, 'xticklabel', []); set(axA, 'yticklabel', []);
                yline(axA, 0, ':', 'LineWidth', 0.5, 'Color', 'k'); % x and y axes at the origin

                for j = 1:numel(t) % iterate
                    
                    if j ~= 1
                        for k = 1:numel(hA)
                            delete(hA{k});
                        end
                        for k = 1:numel(h1_s)
                            delete(h1_s{k});
                        end
                        for k = 1:numel(h2_s)
                            delete(h2_s{k});
                        end
                        delete(bxt); delete(byt); delete(bthetat);
                    end
                    
                    % Child 5 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    % plot the standard stuff
                    hA{1} = plot(axA, leg_1__x(:,j), leg_1__y(:,j), 'Color', col(7,:), 'LineWidth', lW);
                    hA{2} = plot(axA, leg_2__x(:,j), leg_2__y(:,j), 'Color', col(7,:), 'LineWidth', lW);
                    hA{3} = plot(axA, leg_3__x(:,j), leg_3__y(:,j), 'Color', col(7,:), 'LineWidth', lW);
                    hA{4} = plot(axA, leg_4__x(:,j), leg_4__y(:,j), 'Color', col(7,:), 'LineWidth', lW);
                    hA{5} = plot(axA, top__x(:,j), top__y(:,j), 'Color', col(7,:), 'LineWidth', lW_b);
                    hA{6} = plot(axA, left__x(:,j), left__y(:,j), 'Color', col(7,:), 'LineWidth', lW_b);
                    hA{7} = plot(axA, bot__x(:,j), bot__y(:,j), 'Color', col(7,:), 'LineWidth', lW_b);
                    hA{8} = plot(axA, right__x(:,j), right__y(:,j), 'Color', col(7,:), 'LineWidth', lW_b);
                    hA{9} = quiver(axA, body__x(j), body__y(j), frame_scale*bodyf__x(1,j), frame_scale*bodyf__x(2,j), 'LineWidth', lW_qf, 'Color', c,...
                        'AutoScale', 'off', 'ShowArrowHead', 'off'); % SE(2) body frame 'b'
                    hA{10} = quiver(axA, body__x(j), body__y(j), frame_scale*bodyf__y(1,j), frame_scale*bodyf__y(2,j), 'LineWidth', lW_qf, 'Color', c,...
                        'AutoScale', 'off', 'ShowArrowHead', 'off');
                    % limit the position space for viewing
                    axis(axA, lim_temp);
                    % plot the body trajectory stuff
                    hA{11} = plot(axA, body__x(1:j), body__y(1:j), 'LineWidth', lW_s, 'Color', c);
                    % plot the active contact state related stuff
                    hA{12} = plot(axA, leg_i__x(:,j), leg_i__y(:,j), 'Color', col(cs_idx,:), 'LineWidth', lW); 
                    hA{13} = plot(axA, leg_j__x(:,j), leg_j__y(:,j), 'Color', col(cs_idx,:), 'LineWidth', lW);
                    hA{14} = plot(axA, mean_leg_i__x(:,j), mean_leg_i__y(:,j), 'LineStyle', '--', 'Color', col(cs_idx,:), 'LineWidth', lW_r);
                    hA{15} = plot(axA, mean_leg_j__x(:,j), mean_leg_j__y(:,j), 'LineStyle', '--', 'Color', col(cs_idx,:), 'LineWidth', lW_r);
                    hA{16} = scatter(axA, legtip_i__x(j), legtip_i__y(j), circS, col(cs_idx,:), 'filled');
                    hA{17} = scatter(axA, legtip_j__x(j), legtip_j__y(j), circS, col(cs_idx,:), 'filled');
                    hA{18} = plot(axA, ksqij__x(:,j), ksqij__y(:,j), 'Color', c, 'LineWidth', lW_kq, 'LineStyle', '--'); % ksq line
                    title(axA, ['$' num2str(i*10) '\%$ path'], 'Color', col(1,:), 'Interpreter', 'latex', FontSize=titleFS);
                    % Child 1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    h1_s = cell(1,C1.num);
                    for k = 1:C1.num
                        h1_s{k} = scatter(C1.axes{k}, ai(j), aj(j), circS, c, 'filled');
                    end

                    % Child 2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    h2_s = cell(1,C2.num);
                    for k = 1:C2.num
                        h2_s{k} = scatter(C2.axes{k}, ai(j), aj(j), circS, c, 'filled');
                    end

                    % Child 3 current point SE(2) trajectory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    for k = 1:C3.num
                        switch k
                            case 1
                                bxt = scatter(C3.axes{k}, t(j), x(j), circS, c, 'filled');
                            case 2
                                byt = scatter(C3.axes{k}, t(j), y(j), circS, c, 'filled');
                            case 3
                                bthetat = scatter(C3.axes{k}, t(j), theta(j), circS, c, 'filled');
                        end
                    end
                    
                    % updated the figure
                    drawnow();
                    
                    % store the frame
                    writeVideo(video,getframe(f));

                end

                % Save the final frame to the 'F' struct
                F{i} = f;
                
                % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            end

            % close the video
            close(video);

            % store the video data
            datai{end+1} = video;

            % store the frame data
            datai{end+1} = F;

            % Return empty out structs
            dataj = []; dataij = [];
            
    end

end

