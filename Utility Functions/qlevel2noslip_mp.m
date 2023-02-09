% This script helps with the motion planning of the level-2 kinematics of
% the no-slip, quadrupedal robot.
function [datai, dataj] = qlevel2noslip_mp(datai, dataj)

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
            P.grid = [3 5];

            % child 3-- SE(2) trajectory and net displacements
            C3 = [];
            C3.start = [1, 5];
            C3.grid = [3, 1]; C3.span_grid = [2, 1]; % two tiles spanning 3 grids
            C3.tile_start = (C3.start(1)-1)*P.grid(2) + C3.start(2);
            C3.titletxt = {'$b(\psi)$', '$z_{\psi}$'};
            % child 4-- animation window
            C4 = [];
            C4.start = [1, 2];
            C4.grid = [3, 3]; C4.span_grid = C4.grid;
            C4.tile_start = (C4.start(1)-1)*P.grid(2) + C4.start(2);

            % Unpack plotting tools
            lW_s = datai{1}.lW_s;
            col_s1 = datai{1}.col_s1;
            col_s2 = datai{1}.col_s2;
            col_s3 = datai{1}.col_s3; % x, y, and \theta colors in order

    end

    % get the figure dimensions
    m = 2160*(P.grid(2))/5; % scaled figure x-resolution
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
    figure('units','pixels','position',[0 0 m n],'Color','w');
    set(gcf,'Visible','on'); % pop-out figure
    P = tiledlayout(P.grid(1),P.grid(2),'TileSpacing','tight','Padding','tight');
    
    % Initialize the child layouts
    C1.Layout_Obj = tiledlayout(P,C1.grid(1),C1.grid(2)); 
    C1.Layout_Obj.Layout.Tile = C1.tile_start; C1.Layout_Obj.Layout.TileSpan = C1.span_grid;
    C2.Layout_Obj = tiledlayout(P,C2.grid(1),C2.grid(2)); 
    C2.Layout_Obj.Layout.Tile = C2.tile_start; C2.Layout_Obj.Layout.TileSpan = C2.span_grid;
    
    % Child 1
    ax = cell(1, C1.num);
    for i = 1:C1.num
        ax{i} = nexttile(C1.Layout_Obj,i); % dz__x & dz__y
        contourf(ax{i},ai,aj,datai{2}.(C1.sweeptxt{i}),cfLvl,'FaceAlpha',fA,'LineWidth',lW_c); %,'LineStyle','none'
        axis equal tight; hold on; view(2);
        if cond == 1
            contour(ax{i}, ai,aj,ksq_sweep,cLvl,lS,'LineWidth',lW_c+0.5,'EdgeColor',colP); % path constraint contours for path planning
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
        contourf(ax{i},ai,aj,datai{2}.(C2.sweeptxt{i}),cfLvl,'FaceAlpha',fA,'LineWidth',lW_c); % ,'LineStyle','none'
        axis equal tight; hold on; view(2);
        if cond == 1
            contour(ax{i}, ai,aj,ksq_sweep,cLvl,lS,'LineWidth',lW_c+0.5,'EdgeColor',colP); % path constraint contours for path planning
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

            % return an empty dataj struct
            dataj = [];

        case 2
            
            % define the submanifold contact state ordering
            S = [1 2;
                 2 3;
                 3 4;
                 4 1;
                 1 3;
                 2 4];

            % Obtain the path information
            path = datai{5};
            pltkin = datai{4};
            a = datai{3}.aa; l = datai{3}.ll;
            cs_idx = datai{3}.cs_idx;
            cs = S(cs_idx,:);
            lW = datai{1}.lW;
            lW_r = datai{1}.lW_r;
            lW_kq = datai{1}.lW_kq;
            lW_b = datai{1}.lW_b;
            lW_qf = datai{1}.lW_qf;
            circS = datai{1}.circS;
            % frame_scale = p_info{si}.frame_scale;
            frame_scale = 0.25;

            % path scaling vector (the main path is sclaed)
            percent_scale = 10*(1:10);

            % initialize the tiled layouts
            C3.Layout_Obj = tiledlayout(P,C3.grid(1),C3.grid(2)); 
            C3.Layout_Obj.Layout.Tile = C3.tile_start; C3.Layout_Obj.Layout.TileSpan = C3.span_grid;
            C4.Layout_Obj = tiledlayout(P,C4.grid(1),C4.grid(2)); 
            C4.Layout_Obj.Layout.Tile = C4.tile_start; C4.Layout_Obj.Layout.TileSpan = C4.span_grid;

            % Iterate over each open trajectory case
            for i = 1:numel(path.open_trajectory)

                % Unpack the path data for the current trajectory
                t      = path.open_trajectory{i}{1};
                x      = path.open_trajectory{i}{2};
                y      = path.open_trajectory{i}{3};
                theta  = path.open_trajectory{i}{4};
                ai     = path.open_trajectory{i}{5};
                aj     = path.open_trajectory{i}{6};

                % Child 1-- add the shape space trajectory on top
                for j = 1:C1.num
                    if i == 1
                        h1 = plot(C1.axes{j}, ai, aj, 'LineWidth', lW_s, 'Color', path.path_active_color); % add the shape-space path \psi
                    else
                        delete(h1, h1_s);
                        h1 = plot(C1.axes{j}, ai, aj, 'LineWidth', lW_s, 'Color', path.path_active_color);
                    end
                end
                % Child 2-- same as above
                for j = 1:C2.num
                    if i == 1
                        h2 = plot(C2.axes{j}, ai, aj, 'LineWidth', lW_s, 'Color', path.path_active_color);
                    else
                        delete(h2, h2_s);
                        h2 = plot(C2.axes{j}, ai, aj, 'LineWidth', lW_s, 'Color', path.path_active_color);
                    end
                end
                
                % if this is the first frame, create the child 3 plots
                if i == 1

                    % Child 3-- SE(2) trajectories and net displacements
                    ax{1} = nexttile(C3.Layout_Obj, 1); % SE2 trajectories
                    axL1 = yyaxis(ax{1}, 'left');
                    hx = plot(axL1, t, x, 'k-', 'LineWidth', lW_s, 'Color', col_s1);
                    hold on;
                    hy = plot(axL1, t, y, 'k--', 'LineWidth', lW_s, 'Color', col_s2);
                    ylabel('$x(t), y(t)$ (Leg-length = 1)');
                    axR1 = yyaxis(ax{1}, 'right');
                    htheta = plot(axR1, t, y, 'k-.', 'LineWidth', lW_s, 'Color', col_s3);
                    ylabel('$\theta (t)$ (rad)');
                    
                    ax{2} = nexttile(C3.Layout_Obj, 2); % SE2 net displacements
                    axL2 = yyaxis(ax{2}, 'left');
                    bx = bar(axL2, percent_scale, path.net_displacements(1,:), 'EdgeColor', 'none', 'FaceColor', col_s1); % no edge colors
                    hold on;
                    by = bar(axL2, percent_scale, path.net_displacements(2,:), 'EdgeColor', 'none', 'FaceColor', col_s2);
                    ylabel(axL1, '$z^{x, y} (t)$ (Leg-length = 1)');
                    axR2 = yyaxis(ax{2}, 'right');
                    btheta = bar(axR2, percent_scale, path.net_displacements(3,:), 'EdgeColor', 'none', 'FaceColor', col_s3);
                    ylabel(axR1, '$z^{\theta} (t)$ (rad)');

                    C3.axes = ax;

                else

                    delete(hx, hy, htheta, bx, by, btheta, axL1, axR1); % remove the plots from the axes
                    
                    axL1 = yyaxis(C3.axes{1}, 'left');
                    hx = plot(axL1, t, x, 'k-', 'LineWidth', lW_s, 'Color', col_s1);
                    hy = plot(axL1, t, y, 'k--', 'LineWidth', lW_s, 'Color', col_s2);
                    axR1 = yyaxis(C3.axes{1}, 'right');
                    htheta = plot(axR1, t, y, 'k-.', 'LineWidth', lW_s, 'Color', col_s3);

                end

                % Child 3-- update the current net displacement by modifying the edgecolor
                bx(i).EdgeColor = path.path_active_color;
                bx(i).LineWidth = lW_s;
                by(i).EdgeColor = path.path_active_color;
                by(i).LineWidth = lW_s;
                btheta(i).EdgeColor = path.path_active_color;
                btheta(i).LineWidth = lW_s;

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
                mean_leg_i__x = pltkin.(['legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_x'])(a, l, zeros(1,numel(T)), x, y, theta); mean_leg_i__x = [mean_leg_i__x(1:dnum); mean_leg_i__x(dnum+1:end)];
                mean_leg_i__y = pltkin.(['legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_y'])(a, l, zeros(1,numel(T)), x, y, theta); mean_leg_i__y = [mean_leg_i__y(1:dnum); mean_leg_i__y(dnum+1:end)];
                mean_leg_j__x = pltkin.(['legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_x'])(a, l, zeros(1,numel(T)), x, y, theta); mean_leg_j__x = [mean_leg_j__x(1:dnum); mean_leg_j__x(dnum+1:end)];
                mean_leg_j__y = pltkin.(['legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_y'])(a, l, zeros(1,numel(T)), x, y, theta); mean_leg_j__y = [mean_leg_j__y(1:dnum); mean_leg_j__y(dnum+1:end)];
                ksqij__x = pltkin.(['k_leg' num2str(cs(1)) '_leg' num2str(cs(2)) '_x'])(a, l, ai, aj, x, y, theta); ksqij__x = [ksqij__x(1:dnum); ksqij__x(dnum+1:end)];
                ksqij__y = pltkin.(['k_leg' num2str(cs(1)) '_leg' num2str(cs(2)) '_y'])(a, l, ai, aj, x, y, theta); ksqij__y = [ksqij__y(1:dnum); ksqij__y(dnum+1:end)];
                            % standard stuff
                leg_i__x = pltkin.legbase1_leg1_x(a, l, ai, x, y, theta); leg_i__x = [leg_i__x(1:dnum); leg_i__x(dnum+1:end)];
                leg_i__y = pltkin.legbase1_leg1_y(a, l, ai, x, y, theta); leg_i__y = [leg_i__y(1:dnum); leg_i__y(dnum+1:end)];
                leg_j__x = pltkin.legbase2_leg2_x(a, l, aj, x, y, theta); leg_j__x = [leg_j__x(1:dnum); leg_j__x(dnum+1:end)];
                leg_j__y = pltkin.legbase2_leg2_y(a, l, aj, x, y, theta); leg_j__y = [leg_j__y(1:dnum); leg_j__y(dnum+1:end)];
                leg_3__x = pltkin.legbase3_leg3_x(a, l, zeros(1,numel(T)), x, y, theta); leg_3__x = [leg_3__x(1:dnum); leg_3__x(dnum+1:end)];
                leg_3__y = pltkin.legbase3_leg3_y(a, l, zeros(1,numel(T)), x, y, theta); leg_3__y = [leg_3__y(1:dnum); leg_3__y(dnum+1:end)];
                leg_4__x = pltkin.legbase4_leg4_x(a, l, zeros(1,numel(T)), x, y, theta); leg_4__x = [leg_4__x(1:dnum); leg_4__x(dnum+1:end)];
                leg_4__y = pltkin.legbase4_leg4_y(a, l, zeros(1,numel(T)), x, y, theta); leg_4__y = [leg_4__y(1:dnum); leg_4__y(dnum+1:end)];
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

                % find the limits for the animation surface (the body length of the rigid robot is 4l)
                lim_temp = [ min(body_x, [], 'all')-4*l, min(body_y, [], 'all')-4*l, max(body_x, [], 'all')+4*l, max(body_y, [], 'all')+4*l ];
                
                % Child 4-- Animation of the SE(2) trajectory
                if i == 1
                    ax = nexttile(C4.Layout_Obj); 
                    C4.axes = ax;
                else
                    delete(hA);
                end
                xline(C4.axes, 0, ':', 'LineWidth', 0.5, 'Color', 'k'); 
                axis equal square; hold on;
                yline(C4.axes, 0, ':', 'LineWidth', 0.5, 'Color', 'k'); % x and y axes at the origin

                for j = 1:numel(t) % iterate

                    if j ~= numel(t)
                        delete(hA, h1_s, h2_s, h3_x, h3_y, h3_theta);
                    end
                    
                    % Child 4 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    % plot the standard stuff
                    hA{1} = plot(C4.axes, leg_i__x(:,j), leg_i__y(:,j), 'Color', col(7,:), 'LineWidth', lW);
                    hA{2} = plot(C4.axes, leg_j__x(:,j), leg_j__y(:,j), 'Color', col(7,:), 'LineWidth', lW);
                    hA{3} = plot(C4.axes, leg_3__x(:,j), leg_3__y(:,j), 'Color', col(7,:), 'LineWidth', lW);
                    hA{4} = plot(C4.axes, leg_4__x(:,j), leg_4__y(:,j), 'Color', col(7,:), 'LineWidth', lW);
                    hA{5} = plot(C4.axes, top__x(:,j), top__y(:,j), 'Color', col(7,:), 'LineWidth', lW_b);
                    hA{6} = plot(C4.axes, left__x(:,j), left__y(:,j), 'Color', col(7,:), 'LineWidth', lW_b);
                    hA{7} = plot(C4.axes, bot__x(:,j), bot__y(:,j), 'Color', col(7,:), 'LineWidth', lW_b);
                    hA{8} = plot(C4.axes, right__x(:,j), right__y(:,j), 'Color', col(7,:), 'LineWidth', lW_b);
                    hA{9} = quiver(C4.axes, body__x(j), body__y(j), frame_scale*bodyf__x(1,j), frame_scale*bodyf__x(2,j), 'LineWidth', lW_qf, 'Color', path.path_active_color,...
                        'AutoScale', 'off', 'ShowArrowHead', 'off'); % SE(2) body frame 'b'
                    hA{10} = quiver(C4.axes, body__x(j), body__y(j), frame_scale*bodyf__y(1,j), frame_scale*bodyf__y(2,j), 'LineWidth', lW_qf, 'Color', path.path_active_color,...
                        'AutoScale', 'off', 'ShowArrowHead', 'off');
                    axis(C4.axes, [-lim_temp, lim_temp, -lim_temp, lim_temp]);
                    % plot the body trajectory stuff
                    hA{11} = plot(C4.axes, body__x(1:j), body__y(1:j), 'LineWidth', lW_s, 'Color', path.path_active_color);
                    % plot the active contact state related stuff
                    hA{12} = plot(C4.axes, leg_i__x(:,j), leg_i__y(:,j), 'Color', col(cs_idx,:), 'LineWidth', lW); 
                    hA{13} = plot(C4.axes, leg_j__x(:,j), leg_j__y(:,j), 'Color', col(cs_idx,:), 'LineWidth', lW);
                    hA{14} = plot(C4.axes, mean_leg_i__x(:,j), mean_leg_i__y(:,j), 'LineStyle', '--', 'Color', col(cs_idx,:), 'LineWidth', lW_r);
                    hA{15} = plot(C4.axes, mean_leg_j__x(:,j), mean_leg_j__y(:,j), 'LineStyle', '--', 'Color', col(cs_idx,:), 'LineWidth', lW_r);
                    hA{16} = scatter(C4.axes, legtip_i__x(j), legtip_i__y(j), circS, col(cs_idx,:), 'filled');
                    hA{17} = scatter(C4.axes, legtip_j__x(j), legtip_j__y(j), circS, col(cs_idx,:), 'filled');
                    hA{18} = plot(C4.axes, ksqij__x(:,j), ksqij__y(:,j), 'Color', c, 'LineWidth', lW_kq, 'LineStyle', '--'); % ksq line
                    title(['$' num2str(i*10) '%$'],'Color',col(1,:),'Interpreter','latex');

                    % Child 1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    for k = 1:C1.num
                        h1_s = scatter(C1.axes{k}, ai(j), aj(j), circS, path.path_active_color, 'filled');
                    end

                    % Child 2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    for k = 1:C2.num
                        h2_s = scatter(C2.axes{k}, ai(j), aj(j), circS, path.path_active_color, 'filled');
                    end

                    % Child 3 just SE(2) trajectory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    h3_x     = scatter(axL1, t(j), x(j), circS, col_s1, 'filled');
                    h3_y     = scatter(axL1, t(j), y(j), circS, col_s2, 'filled');
                    h3_theta = scatter(axR1, t(j), theta(j), circS, col_s3, 'filled');
                    
                    % updated the figure
                    drawnow();

                end

                if i ~= numel(path.open_trajectory)
                    % Child 3-- switch off the edgecolor after plotting if not the last path to be plotted
                    bx(i).EdgeColor = 'none';
                    by(i).EdgeColor = 'none';
                    btheta(i).EdgeColor = 'none';
                end
                
                % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            end


    end

end

