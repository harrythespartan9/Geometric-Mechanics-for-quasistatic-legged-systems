% This script helps with the motion planning of the level-2 kinematics of
% the no-slip, quadrupedal robot.
function dataij = qlevel2noslip_mp(datai, dataj, dataij)

    % Check which case we are in
    switch nargin
        case 1
            if numel(datai) == 3
                cond = 1;
            elseif numel(datai) == 6
                cond = 2;
            end
        case 2
            dataij = []; 
            dataij.vidF = false; % just return the complete gait trajectory information
            dataij.gaitC_num = 25;
            cond = 3;
        case 3
            cond = 3;
    end
    % if cond is not initialized
    if ~exist('cond', 'var')
        error('ERROR: incorrect arguments for the function.');
    end

    % Unpack plotting information for ith contact state
%     lW_V_i = datai{1}.lW_V; iQ_i = datai{1}.iQ;
    lW_c_i = datai{1}.lW_contour; fA_i = datai{1}.fA; 
    cfLvl_i = datai{1}.cfLvl; cLvl_i = datai{1}.cLvl;
    gc_col_i = datai{1}.gc_col; col_backg_i = datai{1}.col_backg;
    col_i = datai{1}.col; CUB_i = datai{1}.CUB;
    titleFS_i = datai{1}.titleFS; tickFS_i = datai{1}.tickFS; cbarFS_i = datai{1}.cbarFS;
    labelFS_i = datai{1}.labelFS; sgtitleFS_i = datai{1}.sgtitleFS;
    sgtitle_txt_i = datai{1}.sgtitle_txt;
    x_label_txt_i = datai{1}.x_label_txt;
    y_label_txt_i = datai{1}.y_label_txt;
    xtickval_i = datai{1}.ytickval; ytickval_i = datai{1}.ytickval;
    xticklab_i = datai{1}.xticklab; yticklab_i = datai{1}.yticklab;
    xlimits_i = datai{1}.xlimits; ylimits_i = datai{1}.ylimits;
    ksq_lb_i =  datai{1}.ksq_lb;
    domainPercentage = 5; % arrow length as a function of the shape-space domain
    arrAngle = deg2rad(18); % using 30degrees as an experimental angle
    
    % sweep data for ith contact state
    a1_i = datai{2}.ai; a2_i = datai{2}.aj;
    ksq_sweep_i = datai{2}.ksq_sweep;
    dz__x_sweep_i = datai{2}.dz__x_sweep;
    dz__y_sweep_i = datai{2}.dz__y_sweep;
    dz__theta_sweep_i = datai{2}.dz__theta_sweep;

    % Compute the limits on the data-- helps with countour plotting
    [C1_lim,C2_lim] = se2limits(dz__x_sweep_i,dz__y_sweep_i,dz__theta_sweep_i);
    
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
%             colP_i = datai{1}.colP;
            lS_i = datai{1}.lS;

        case 2 % infinitesimal disps with trajectory + animation

            % parent grid
            P = [];
            P.grid = [3 6];

            % child 3-- SE(2) trajectory and net displacements
            C3 = [];
            C3.start = [1, 5];
            C3.grid = [3, 1];  C3.num = prod(C3.grid);
            C3.span_grid = C3.grid;
            C3.tile_start = (C3.start(1)-1)*P.grid(2) + C3.start(2);
            C3.titletxt = '$$b(\hat{\phi})$$';
            % child 4--  SE(2) net displacements
            C4 = [];
            C4.start = [1, 6]; 
            C4.grid = [3, 1]; C4.num = prod(C4.grid);
            C4.span_grid = C4.grid;
            C4.tile_start = (C4.start(1)-1)*P.grid(2) + C4.start(2);
            C4.titletxt = '$$z(\hat{\phi})$$';

            % Unpack plotting tools
            lW_s_i = datai{1}.lW_s;
            
            % contact state
            cs_i = datai{3}.cs;

        case 3

            % choose the path scaling inputs to plot gaits at
            if ~isfield(dataij, 'u')
                dataij.u = [+100, +100]/100;    % +x, +y just plot max input gait
            end

            % parent grid
            P = [];
            P.grid = [3 5];

            % create a colormap for displacement heatmaps as a function of the gain space
            CUB_alt = [178,24,43;
                    214,96,77;
                    244,165,130;
                    253,219,199;
                    247,247,247;
                    209,229,240;
                    146,197,222;
                    67,147,195;
                    33,102,172]/255; % base
            CUB_alt = interp1(linspace(0,100,size(CUB_alt,1)), CUB_alt, linspace(0,100,size(jet,1)), "spline"); CUB_alt(CUB_alt > 1) = 1; % interpolate to expand
            dataij.CUB_alt = CUB_alt;

            % Unpack plotting information for ith contact state
        %     lW_V_i = datai{1}.lW_V; iQ_i = datai{1}.iQ;
            lW_c_j = dataj{1}.lW_contour; fA_j = dataj{1}.fA; 
            cfLvl_j = dataj{1}.cfLvl; cLvl_j = dataj{1}.cLvl;
            gc_col_j = dataj{1}.gc_col; col_backg_j = dataj{1}.col_backg;
            col_j = dataj{1}.col; CUB_j = dataj{1}.CUB;
            titleFS_j = dataj{1}.titleFS; tickFS_j = dataj{1}.tickFS; cbarFS_j = dataj{1}.cbarFS;
            labelFS_j = dataj{1}.labelFS; sgtitleFS_j = dataj{1}.sgtitleFS;
            sgtitle_txt_j = dataj{1}.sgtitle_txt;
            x_label_txt_j = dataj{1}.x_label_txt;
            y_label_txt_j = dataj{1}.y_label_txt;
            xtickval_j = dataj{1}.ytickval; ytickval_j = dataj{1}.ytickval;
            xticklab_j = dataj{1}.xticklab; yticklab_j = dataj{1}.yticklab;
            xlimits_j = dataj{1}.xlimits; ylimits_j = dataj{1}.ylimits;
            ksq_lb_j =  dataj{1}.ksq_lb;
            
            % sweep data for ith contact state
            a1_j = dataj{2}.ai; a2_j = dataj{2}.aj;
            ksq_sweep_j = dataj{2}.ksq_sweep;
            dz__x_sweep_j = dataj{2}.dz__x_sweep;
            dz__y_sweep_j = dataj{2}.dz__y_sweep;
            dz__theta_sweep_j = dataj{2}.dz__theta_sweep;

            % contact state
            cs_i = datai{3}.cs;
            cs_j = dataj{3}.cs;
        
            % Compute the limits on the data-- helps with countour plotting of stratified panels
            [C3_lim,C4_lim] = se2limits(dz__x_sweep_j,dz__y_sweep_j,dz__theta_sweep_j);

            % child 3-- jth dz_translation
            C3 = []; C3.limits = C3_lim;
            C3.start = [1, 5];
            C3.grid = [2, 1]; C3.num = prod(C3.grid); C3.span_grid = C3.grid;
            C3.tile_start = (C3.start(1)-1)*P.grid(2) + C3.start(2);
            C3.sweeptxt = {'dz__x_sweep', 'dz__y_sweep'};
            if cond == 3
                C3.titletxt = {['$$dz^{x}_{' num2str(cs_j(1)) num2str(cs_j(2)) '}$$'],...
            ['$$dz^{y}_{' num2str(cs_j(1)) num2str(cs_j(2)) '}$$']};
            else
                C3.titletxt = {'$$dz^{x}$$', '$$dz^{y}$$'};
            end
            % child 4-- jth dz_rotation
            C4 = []; C4.limits = C4_lim;
            C4.start = [3, 5];
            C4.grid = [1, 1]; C4.num = prod(C4.grid); C4.span_grid = C4.grid;
            C4.tile_start = (C4.start(1)-1)*P.grid(2) + C4.start(2);
            C4.sweeptxt = {'dz__theta_sweep'};
            if cond == 3
                C4.titletxt = {['$$dz^{\theta}_{' num2str(cs_j(1)) num2str(cs_j(2)) '}$$']};
            else
                C4.titletxt = {'$$dz^{\theta}$$'};
            end

            % Unpack plotting tools
            lW_s_i = datai{1}.lW_s;
            lW_s_j = dataj{1}.lW_s;

    end
    
    % get the figure dimensions
    m = 2160*(P.grid(2))/5; % scaled figure x-resolution
    n = 1800; % fixed figure y-resolution

    % child 1-- ith dz_translation
    C1 = []; C1.limits = C1_lim; % previously defined color limits
    C1.start = [1, 1];
    C1.grid = [2, 1]; C1.num = prod(C1.grid); C1.span_grid = C1.grid; % the tiles in the child layout matches the parent
    C1.tile_start = (C1.start(1)-1)*P.grid(2) + C1.start(2);
    C1.sweeptxt = {'dz__x_sweep', 'dz__y_sweep'};
    if cond == 3
        C1.titletxt = {['$$dz^{x}_{' num2str(cs_i(1)) num2str(cs_i(2)) '}$$'],...
            ['$$dz^{y}_{' num2str(cs_i(1)) num2str(cs_i(2)) '}$$']};
    else
        C1.titletxt = {'$$dz^{x}$$', '$$dz^{y}$$'};
    end
    % child 2-- ith dz_theta
    C2 = []; C2.limits = C2_lim;
    C2.start = [3, 1];
    C2.grid = [1, 1]; C2.num = prod(C2.grid); C2.span_grid = C2.grid;
    C2.tile_start = (C2.start(1)-1)*P.grid(2) + C2.start(2);
    C2.sweeptxt = {'dz__theta_sweep'};
    if cond == 3
        C2.titletxt = {['$$dz^{\theta}_{' num2str(cs_i(1)) num2str(cs_i(2)) '}$$']};
    else
        C2.titletxt = {'$$dz^{\theta}$$'};
    end

    % Create the figure
    f = figure('units','pixels','position',[100 -200 m n],'Color','w');
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
            contourf(ax{i},a1_i,a2_i,datai{2}.(C1.sweeptxt{i}),cfLvl_i,'FaceAlpha',fA_i,'LineWidth',lW_c_i);
            axis equal tight; hold on; view(2);
            contour(ax{i}, a1_i,a2_i,ksq_sweep_i,cLvl_i,lS_i,'LineWidth',lW_c_i+0.5,'EdgeColor',gc_col_i); % path constraint contours for path planning
        else
            contourf(ax{i},a1_i,a2_i,datai{2}.(C1.sweeptxt{i}),cfLvl_i,'FaceAlpha',fA_i,'LineWidth',lW_c_i,'LineStyle','none'); %,'LineStyle','none'
            axis equal tight; hold on; view(2);
        end
        contour(ax{i}, a1_i,a2_i,ksq_sweep_i,[ksq_lb_i ksq_lb_i],'k--','LineWidth',lW_c_i); % exclusion zone
        colormap(ax{i},CUB_i); clim(ax{i},C1_lim);
        set(get(ax{i},'YLabel'),'rotation',0,'VerticalAlignment','middle');
%         if cond ~= 3
        title(ax{i},C1.titletxt{i},'Color','k',FontSize=titleFS_i);
%         end
        if i == 1
            xlabel(ax{i},x_label_txt_i,FontSize=labelFS_i); 
            ylabel(ax{i},y_label_txt_i,FontSize=labelFS_i);
        end
        xticks(ax{i}, xtickval_i); yticks(ax{i}, ytickval_i);
        xticklabels(ax{i}, xticklab_i); yticklabels(ax{i}, yticklab_i);
        ax{i}.XAxis.FontSize = tickFS_i; ax{i}.YAxis.FontSize = tickFS_i; 
        set(ax{i},'Color',col_backg_i);
        xlim(xlimits_i); ylim(ylimits_i);
    end
    C1.axes = ax;
    if cond == 1
        C1.colorB = colorbar(C1.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS_i); C1.colorB.Layout.Tile = 'South'; % plot the translation colorbar
%     elseif cond == 3
%         title(C1.Layout_Obj, ['$S_{' num2str(cs_i(1)) num2str(cs_i(2)) '}$'], 'Color', gc_col_i, 'Interpreter', 'latex', FontSize=titleFS_i);
    end
    
    % Child 2
    ax = cell(1, C2.num);
    for i = 1:C2.num
        ax{i} = nexttile(C2.Layout_Obj,i); % just dz__\theta
        if cond == 1
            contourf(ax{i},a1_i,a2_i,datai{2}.(C2.sweeptxt{i}),cfLvl_i,'FaceAlpha',fA_i,'LineWidth',lW_c_i);
            axis equal tight; hold on; view(2);
            contour(ax{i}, a1_i,a2_i,ksq_sweep_i,cLvl_i,lS_i,'LineWidth',lW_c_i+0.5,'EdgeColor',gc_col_i); % path constraint contours for path planning
        else
            contourf(ax{i},a1_i,a2_i,datai{2}.(C2.sweeptxt{i}),cfLvl_i,'FaceAlpha',fA_i,'LineWidth',lW_c_i,'LineStyle','none');
            axis equal tight; hold on; view(2);
        end
        contour(ax{i}, a1_i,a2_i,ksq_sweep_i,[ksq_lb_i ksq_lb_i],'--k','LineWidth',lW_c_i);
        colormap(ax{i},CUB_i); clim(ax{i},C2_lim);
        set(get(ax{i},'YLabel'),'rotation',0,'VerticalAlignment','middle');
%         if cond ~= 3
        title(ax{i},C2.titletxt{i},'Color','k',FontSize=titleFS_i);
%         end
        xticks(ax{i}, xtickval_i); yticks(ax{i}, ytickval_i);
        xticklabels(ax{i}, xticklab_i); yticklabels(ax{i}, yticklab_i);
        ax{i}.XAxis.FontSize = tickFS_i; ax{i}.YAxis.FontSize = tickFS_i; 
        set(ax{i},'Color',col_backg_i);
        xlim(xlimits_i); ylim(ylimits_i);
    end
    C2.axes = ax;
    if cond == 1
        C2.colorB = colorbar(C2.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS_i); C2.colorB.Layout.Tile = 'South'; % plot the translation colorbar
    end
    
    switch cond

        case 1

            % label the figure
            title(P,sgtitle_txt_i,'Color',gc_col_i,'Interpreter','latex','FontSize',sgtitleFS_i);

            % return empty out strcuts
            dataij = [];

        case 2

            % Obtain the path information
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
            cs_idx_i = datai{3}.cs_idx;
            ank = datai{3}.ank;

            % get the integration direction
            dirn_i = path_i.int_dirn;

            % path vector (the main path is "scaled" or "slid") % "slid" added on 07/27/2023
            if isfield(datai{1}, "mode")
                switch datai{1}.mode
                    case "scale"
                        pscale = 10*([-fliplr(1:10) 1:10]);
                    case "slide"
                        pscale = 10*([-fliplr(1:10) 0 1:10]);
                end
            else
                pscale = 10*([-fliplr(1:10) 1:10]);
            end

            % initialize the tiled layouts
            C3.Layout_Obj = tiledlayout(P,C3.grid(1),C3.grid(2),'TileSpacing','tight','Padding','tight'); 
            C3.Layout_Obj.Layout.Tile = C3.tile_start; C3.Layout_Obj.Layout.TileSpan = C3.span_grid;
            C4.Layout_Obj = tiledlayout(P,C4.grid(1),C4.grid(2),'TileSpacing','tight','Padding','tight'); 
            C4.Layout_Obj.Layout.Tile = C4.tile_start; C4.Layout_Obj.Layout.TileSpan = C4.span_grid;
            
            % compute the span of the animation in world coordinates-- for the largest path
            xM = path_i.open_trajectory{20}{2}; yM = path_i.open_trajectory{20}{3};
            bl = 4*l_i; % body length of the robot-- need to add this to the rigid robot properties
            thresh = 1; % one body length
            lim_temp = max( [min(xM, [], 'all')-thresh*bl, max(xM, [], 'all')+thresh*bl, min(yM, [], 'all')-thresh*bl, max(yM, [], 'all')+thresh*bl]...
                , [], 'all' )* [-1, 1, -1, 1]; % 2* body length margin

            % Video stuff
            if vidF_i
                video = VideoWriter(['data_animation','.mp4'],'MPEG-4');
                video.FrameRate = 120; %  60
                video.Quality = 100;
                open(video);
            end
            
            % Set the main title
            title(P,sgtitle_txt_i,'Color',gc_col_i,'Interpreter','latex','FontSize',sgtitleFS_i);
            
            % Create a structure to hold the final frame information for each path scaling
            if vidF_i
                skp_path = [1, 8, 13, 20];
                datai{1}.skp_path = skp_path;
            else
                if isfield(datai{1}, 'skp_path')
                    skp_path = datai{1}.skp_path;
                else
                    skp_path = 20; % if no path is requested, the plot the full path length
                end
            end
            

            % Iterate over each open trajectory case
            for i = skp_path % plotting 10%, 40%, 70%, and 100%

                % delete the arrows from last time
                if i ~= skp_path(1)
                    if exist('h1_A', 'var')
                        for j = 1:numel(h1_A)
                            delete(h1_A{j});
                        end
                    end
                    if exist('h1', 'var')
                        for j = 1:numel(h1)
                            delete(h1{j});
                        end
                    end
                    if exist('h1b', 'var')
                        for j = 1:numel(h1b)
                            delete(h1b{j});
                        end
                    end
                    if exist('h2_A', 'var')
                        for j = 1:numel(h2_A)
                            delete(h2_A{j});
                        end
                    end
                    if exist('h2', 'var')
                        for j = 1:numel(h2)
                            delete(h2{j});
                        end
                    end
                    if exist('h2b', 'var')
                        for j = 1:numel(h2b)
                            delete(h2b{j});
                        end
                    end
                end

                % Obtain the path lengths (or t in our case)
                t_int_i = path_i.path_length{i};

                % get the initial condition of the path
                ic_i = path_i.initial_condition{i};

                % Unpack the path data for the current trajectory
                t_i      = path_i.open_trajectory{i}{1};
                x_i      = path_i.open_trajectory{i}{2};
                y_i      = path_i.open_trajectory{i}{3};
                theta_i  = path_i.open_trajectory{i}{4};
                a1_i     = path_i.open_trajectory{i}{5};
                a2_i     = path_i.open_trajectory{i}{6};
%                 numD    = numel(t);
                
                % angle of each limb
                alp_i = cell(1,4); % store the angle trajectory
                for j = 1:4
                    if sum(j == cs_i) == 0
                        alp_i{j} = zeros(size(a1_i));
                    else
                        switch find(j == cs_i)
                            case 1
                                alp_i{j} = a1_i;
                            case 2
                                alp_i{j} = a2_i;
                        end
                    end
                end

                % set the arrow size and angle for manual plotting
                arrSize = domainPercentage/100*2*ank; % convert the length of the arrow to radians

                % Child 1-- add the shape space trajectory on top
                h1 = cell(1,C1.num); h1b = h1;
                h1_A = cell(1,C1.num);
                for j = 1:C1.num
                    if i ~= 1 && exist('h1_s', 'var')
                        delete(h1_s{j});
                    end
                    h1{j} = plot(C1.axes{j}, a1_i, a2_i, 'LineWidth', lW_s_i, 'Color', gc_col_i);
                    % h1b{j} = plot(C1.axes{j}, [a1_i(1) a1_i(end)], [a2_i(1) a2_i(end)], '--', 'LineWidth', lW_s_i, 'Color', col_i(7,:));
                    h1_A{j} = plotpatharrow(C1.axes{j}, a1_i, a2_i, arrSize*t_int_i/2, arrAngle, lW_s_i, gc_col_i);
                end

                % Child 2-- same as above
                h2 = cell(1,C2.num); h2b = h2;
                h2_A = cell(1,C2.num);
                for j = 1:C2.num
                    if i ~= 1 && exist('h2_s', 'var')
                        delete(h2_s{j}); % delete(h2{j});
                    end
                    h2{j} = plot(C2.axes{j}, a1_i, a2_i, 'LineWidth', lW_s_i, 'Color', gc_col_i);
                    % h2b{j} = plot(C2.axes{j}, [a1_i(1) a1_i(end)], [a2_i(1) a2_i(end)], '--', 'LineWidth', lW_s_i, 'Color', col_i(7,:));
                    h2_A{j} = plotpatharrow(C2.axes{j}, a1_i, a2_i, arrSize*t_int_i/2, arrAngle, lW_s_i, gc_col_i);
                end

                % Obtain the integration time for the ic and main path
                temp_t = cumsum(path_i.int_time);
                t_int_i = path_i.path_length{i}/temp_t(2) * temp_t;
                
                % if this is the first frame, create the child 3 plots
                if i == skp_path(1)
                    
                    % limit thresholds for se(2) trajectories
                    lowxy = 0.05; % 5% of the leg length
                    lowtheta = deg2rad(3); % 3 degrees of orientation change

                    % Child 3-- SE(2) trajectories
                    [lim_bxy, lim_btheta] = se2limits(x_i, y_i, theta_i); % limits
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
                                bx = plot(ax{j}, t_i, x_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i);
                                ylabel(ax{j}, '$$b^x$$', FontSize=labelFS_i); ylim(ax{j}, lim_bxy);
                            case 2
                                by = plot(ax{j}, t_i, y_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i);
                                ylabel(ax{j}, '$$b^y$$', FontSize=labelFS_i); ylim(ax{j}, lim_bxy);
                            case 3
                                btheta = plot(ax{j}, t_i, theta_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i);
                                ylabel(ax{j}, '$$b^{\theta}$$', FontSize=labelFS_i);
                                xlabel(ax{j}, '$$t$$', FontSize=labelFS_i); ylim(ax{j}, lim_btheta);
                        end
                        set(get(ax{j},'YLabel'),'rotation',0,'VerticalAlignment','middle');
                        ax{j}.XAxis.FontSize = tickFS_i-5; ax{j}.YAxis.FontSize = tickFS_i-5;
                        grid on; hold on; axis square; xlim(ax{j}, [min(t_i) max(t_i)]);
                    end
                    title(C3.Layout_Obj, C3.titletxt, 'Color', 'k', 'Interpreter', 'latex', FontSize=titleFS_i);
                    C3.axes = ax;

                    % Child 4-- SE(2) net displacements
                    [lim_zxy, lim_ztheta] = se2limits(zx_i, zy_i, ztheta_i);
                    if max(abs(lim_zxy)) < lowxy
                        lim_zxy = lowxy*[-1, 1];
                    end
                    if max(abs(lim_ztheta)) < lowtheta
                        lim_ztheta = lowtheta*[-1, 1];
                    end
                    ax = cell(1,C4.num);
                    for j = 1:C4.num
                        ax{j} = nexttile(C4.Layout_Obj, j);
                        switch j
                            case 1
                                plot(ax{j}, pscale/100, zx_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i);
                                grid on; hold on;
                                zxiter = scatter(ax{j}, pscale(i), zx_i(i), circS_i, gc_col_i, 'filled');
                                ylabel(ax{j}, '$$z^x$$', FontSize=labelFS_i); ylim(lim_zxy);
                            case 2
                                plot(ax{j}, pscale/100, zy_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i);
                                grid on; hold on; axis square;
                                zyiter = scatter(ax{j}, pscale(i), zy_i(i), circS_i, gc_col_i, 'filled');
                                ylabel(ax{j}, '$$z^y$$', FontSize=labelFS_i); ylim(lim_zxy);
                            case 3
                                plot(ax{j}, pscale/100, ztheta_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i); % added /100 to plot in decimal
                                grid on; hold on; axis square;
                                zthetaiter = scatter(ax{j}, pscale(i), ztheta_i(i), circS_i, gc_col_i, 'filled');
                                ylabel(ax{j}, '$$z^{\theta}$$', FontSize=labelFS_i);
                                xlabel(ax{j}, '$$u$$', FontSize=labelFS_i);  ylim(lim_ztheta); % 'Path scaling (\%)'
                        end
                        set(get(ax{j},'YLabel'),'rotation',0,'VerticalAlignment','middle'); axis square;
                        ax{j}.XAxis.FontSize = tickFS_i-5; ax{j}.YAxis.FontSize = tickFS_i-5;
                        xlim([min(pscale/100) max(pscale/100)]);
                    end
                    title(C4.Layout_Obj, C4.titletxt, 'Color', 'k', 'Interpreter', 'latex', FontSize=titleFS_i);
                    C4.axes = ax;

                else
                    
                    % remove plots
%                     delete(bx, by, btheta,... % clear the last path's SE(2) trajectory
%                         bxt, byt, bthetat,... % clear the last path's SE(2) scatter at the final time step
%                         zxi, zyi, zthetai); % clear the SE(2) net displacement scatter for previous path
                    if exist('bx', 'var')
                        delete(bx);
                    end
                    if exist('by', 'var')
                        delete(by);
                    end
                    if exist('btheta', 'var')
                        delete(btheta);
                    end
                    if exist('bxt', 'var')
                        delete(bxt);
                    end
                    if exist('byt', 'var')
                        delete(byt);
                    end
                    if exist('bthetat', 'var')
                        delete(bthetat);
                    end
                    if exist('zxiter', 'var')
                        delete(zxiter);
                    end
                    if exist('zyiter', 'var')
                        delete(zyiter);
                    end
                    if exist('zthetaiter', 'var')
                        delete(zthetaiter);
                    end

                    % reinitialize some parameters if not initialized
                    if ~exist('lowxy', 'var')
                        lowxy = 0.05;
                        lowtheta = deg2rad(3);
                    end

                    % Child 3-- plot the current SE(2)
                    [lim_bxy, lim_btheta] = se2limits(x_i, y_i, theta_i);
                    if lim_bxy(2) < lowxy
                        lim_bxy = 0.1*[-1, 1];
                    end
                    if lim_btheta(2) < lowtheta
                        lim_btheta = lowtheta*[-1, 1];
                    end
                    for j = 1:C3.num
                        switch j
                            case 1
                                bx = plot(C3.axes{j}, t_i, x_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i); ylim(C3.axes{j}, lim_bxy);
                            case 2
                                by = plot(C3.axes{j}, t_i, y_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i); ylim(C3.axes{j}, lim_bxy);
                            case 3
                                btheta = plot(C3.axes{j}, t_i, theta_i, '-', 'LineWidth', lW_s_i, 'Color', gc_col_i); ylim(C3.axes{j}, lim_btheta);
                        end
                        xlim(C3.axes{j}, [min(t_i) max(t_i)]);
                    end

                    % Child 4-- scatter the current net SE(2) displacement
                    for j = 1:C4.num
                        switch j
                            case 1
                                zxiter = scatter(C4.axes{j}, pscale(i), zx_i(i), circS_i, gc_col_i, 'filled');
                            case 2
                                zyiter = scatter(C4.axes{j}, pscale(i), zy_i(i), circS_i, gc_col_i, 'filled');
                            case 3
                                zthetaiter = scatter(C4.axes{j}, pscale(i), ztheta_i(i), circS_i, gc_col_i, 'filled');
                        end
                    end


                end

                % Animation and updates to other plots ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                
                % compute everything you need to plot the system with
                            % stuff pertaining to the specific contact state
                leg_i__x = pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_x'])(a_i, l_i, a1_i, x_i, y_i, theta_i); leg_i__x = [leg_i__x(1:dnum_i); leg_i__x(dnum_i+1:end)];
                leg_i__y = pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_y'])(a_i, l_i, a1_i, x_i, y_i, theta_i); leg_i__y = [leg_i__y(1:dnum_i); leg_i__y(dnum_i+1:end)];
                leg_j__x = pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_x'])(a_i, l_i, a2_i, x_i, y_i, theta_i); leg_j__x = [leg_j__x(1:dnum_i); leg_j__x(dnum_i+1:end)];
                leg_j__y = pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_y'])(a_i, l_i, a2_i, x_i, y_i, theta_i); leg_j__y = [leg_j__y(1:dnum_i); leg_j__y(dnum_i+1:end)]; % ik it is duplicate arrays, easiest way for now
                legtip_i__x = pltkin_i.(['leg' num2str(cs_i(1)) '_x'])(a_i, l_i, a1_i, x_i, y_i, theta_i);
                legtip_i__y = pltkin_i.(['leg' num2str(cs_i(1)) '_y'])(a_i, l_i, a1_i, x_i, y_i, theta_i);
                legtip_j__x = pltkin_i.(['leg' num2str(cs_i(2)) '_x'])(a_i, l_i, a2_i, x_i, y_i, theta_i); 
                legtip_j__y = pltkin_i.(['leg' num2str(cs_i(2)) '_y'])(a_i, l_i, a2_i, x_i, y_i, theta_i);
                mean_leg_i__x = pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_x'])(a_i, l_i, zeros(1,numel(x_i)), x_i, y_i, theta_i); mean_leg_i__x = [mean_leg_i__x(1:dnum_i); mean_leg_i__x(dnum_i+1:end)];
                mean_leg_i__y = pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_y'])(a_i, l_i, zeros(1,numel(x_i)), x_i, y_i, theta_i); mean_leg_i__y = [mean_leg_i__y(1:dnum_i); mean_leg_i__y(dnum_i+1:end)];
                mean_leg_j__x = pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_x'])(a_i, l_i, zeros(1,numel(x_i)), x_i, y_i, theta_i); mean_leg_j__x = [mean_leg_j__x(1:dnum_i); mean_leg_j__x(dnum_i+1:end)];
                mean_leg_j__y = pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_y'])(a_i, l_i, zeros(1,numel(x_i)), x_i, y_i, theta_i); mean_leg_j__y = [mean_leg_j__y(1:dnum_i); mean_leg_j__y(dnum_i+1:end)];
                ksqij__x = pltkin_i.(['k_leg' num2str(cs_i(1)) '_leg' num2str(cs_i(2)) '_x'])(a_i, l_i, a1_i, a2_i, x_i, y_i, theta_i); ksqij__x = [ksqij__x(1:dnum_i); ksqij__x(dnum_i+1:end)];
                ksqij__y = pltkin_i.(['k_leg' num2str(cs_i(1)) '_leg' num2str(cs_i(2)) '_y'])(a_i, l_i, a1_i, a2_i, x_i, y_i, theta_i); ksqij__y = [ksqij__y(1:dnum_i); ksqij__y(dnum_i+1:end)];
                            % standard stuff
                leg_1__x = pltkin_i.legbase1_leg1_x(a_i, l_i, alp_i{1}, x_i, y_i, theta_i); leg_1__x = [leg_1__x(1:dnum_i); leg_1__x(dnum_i+1:end)];
                leg_1__y = pltkin_i.legbase1_leg1_y(a_i, l_i, alp_i{1}, x_i, y_i, theta_i); leg_1__y = [leg_1__y(1:dnum_i); leg_1__y(dnum_i+1:end)];
                leg_2__x = pltkin_i.legbase2_leg2_x(a_i, l_i, alp_i{2}, x_i, y_i, theta_i); leg_2__x = [leg_2__x(1:dnum_i); leg_2__x(dnum_i+1:end)];
                leg_2__y = pltkin_i.legbase2_leg2_y(a_i, l_i, alp_i{2}, x_i, y_i, theta_i); leg_2__y = [leg_2__y(1:dnum_i); leg_2__y(dnum_i+1:end)];
                leg_3__x = pltkin_i.legbase3_leg3_x(a_i, l_i, alp_i{3}, x_i, y_i, theta_i); leg_3__x = [leg_3__x(1:dnum_i); leg_3__x(dnum_i+1:end)];
                leg_3__y = pltkin_i.legbase3_leg3_y(a_i, l_i, alp_i{3}, x_i, y_i, theta_i); leg_3__y = [leg_3__y(1:dnum_i); leg_3__y(dnum_i+1:end)];
                leg_4__x = pltkin_i.legbase4_leg4_x(a_i, l_i, alp_i{4}, x_i, y_i, theta_i); leg_4__x = [leg_4__x(1:dnum_i); leg_4__x(dnum_i+1:end)];
                leg_4__y = pltkin_i.legbase4_leg4_y(a_i, l_i, alp_i{4}, x_i, y_i, theta_i); leg_4__y = [leg_4__y(1:dnum_i); leg_4__y(dnum_i+1:end)];
                top__x = pltkin_i.topright2topleft_x(l_i, x_i, y_i, theta_i); top__x = [top__x(1:dnum_i); top__x(dnum_i+1:end)];
                top__y = pltkin_i.topright2topleft_y(l_i, x_i, y_i, theta_i); top__y = [top__y(1:dnum_i); top__y(dnum_i+1:end)];
                left__x = pltkin_i.topleft2botleft_x(l_i, x_i, y_i, theta_i); left__x = [left__x(1:dnum_i); left__x(dnum_i+1:end)];
                left__y = pltkin_i.topleft2botleft_y(l_i, x_i, y_i, theta_i); left__y = [left__y(1:dnum_i); left__y(dnum_i+1:end)];
                bot__x = pltkin_i.botleft2botright_x(l_i, x_i, y_i, theta_i); bot__x = [bot__x(1:dnum_i); bot__x(dnum_i+1:end)];
                bot__y = pltkin_i.botleft2botright_y(l_i, x_i, y_i, theta_i); bot__y = [bot__y(1:dnum_i); bot__y(dnum_i+1:end)];
                right__x = pltkin_i.botright2topright_x(l_i, x_i, y_i, theta_i); right__x = [right__x(1:dnum_i); right__x(dnum_i+1:end)];
                right__y = pltkin_i.botright2topright_y(l_i, x_i, y_i, theta_i); right__y = [right__y(1:dnum_i); right__y(dnum_i+1:end)];
                body__x = pltkin_i.body_x(x_i, y_i, theta_i); 
                body__y = pltkin_i.body_y(x_i, y_i, theta_i);
                bodyf__x = pltkin_i.bodyf_x(x_i, y_i, theta_i); 
                bodyf__y = pltkin_i.bodyf_y(x_i, y_i, theta_i);
                
                % Child 5-- Animation of the SE(2) trajectory
                if i == 1
                    axA = nexttile(P, [3, 3]); 
                elseif exist('hA', 'var')
                    for k = 1:numel(hA)
                        delete(hA{k});
                    end
                end
                xline(axA, 0, ':', 'LineWidth', 0.5, 'Color', 'k'); 
                axis equal square; hold on; 
                set(axA, 'xticklabel', []); set(axA, 'yticklabel', []);
                yline(axA, 0, ':', 'LineWidth', 0.5, 'Color', 'k'); % x and y axes at the origin

                % Check if we want a video-- if yes, create a time stamp vector as long as t. Else just plot the final frame and save the figure as a frame.
                if vidF_i
                    jiter = 1:numel(t_i);
                elseif ~vidF_i
                    jiter = numel(t_i);
                else
                    error('ERROR: vidF needs to be a boolean flag. You can''t be here!!');
                end

                for j = jiter % iterate over time
                    
                    if j ~= 1 && numel(jiter) ~= 1

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
                    hA{1} = plot(axA, leg_1__x(:,j), leg_1__y(:,j), 'Color', col_i(7,:), 'LineWidth', lW_i);
                    hA{2} = plot(axA, leg_2__x(:,j), leg_2__y(:,j), 'Color', col_i(7,:), 'LineWidth', lW_i);
                    hA{3} = plot(axA, leg_3__x(:,j), leg_3__y(:,j), 'Color', col_i(7,:), 'LineWidth', lW_i);
                    hA{4} = plot(axA, leg_4__x(:,j), leg_4__y(:,j), 'Color', col_i(7,:), 'LineWidth', lW_i);
                    hA{5} = plot(axA, top__x(:,j), top__y(:,j), 'Color', col_i(7,:), 'LineWidth', lW_b_i);
                    hA{6} = plot(axA, left__x(:,j), left__y(:,j), 'Color', col_i(7,:), 'LineWidth', lW_b_i);
                    hA{7} = plot(axA, bot__x(:,j), bot__y(:,j), 'Color', col_i(7,:), 'LineWidth', lW_b_i);
                    hA{8} = plot(axA, right__x(:,j), right__y(:,j), 'Color', col_i(7,:), 'LineWidth', lW_b_i);
                    hA{9} = quiver(axA, body__x(j), body__y(j), frame_scale_i*bodyf__x(1,j), frame_scale_i*bodyf__x(2,j), 'LineWidth', lW_qf_i, 'Color', gc_col_i,...
                        'AutoScale', 'off', 'ShowArrowHead', 'off'); % SE(2) body frame 'b'
                    hA{10} = quiver(axA, body__x(j), body__y(j), frame_scale_i*bodyf__y(1,j), frame_scale_i*bodyf__y(2,j), 'LineWidth', lW_qf_i, 'Color', gc_col_i,...
                        'AutoScale', 'off', 'ShowArrowHead', 'off');
                    % limit the position space for viewing
                    axis(axA, lim_temp);
                    % plot the body trajectory stuff
                    hA{11} = plot(axA, body__x(1:j), body__y(1:j), 'LineWidth', 2*lW_s_i, 'Color', gc_col_i);
                    % plot the active contact state related stuff
                    hA{12} = plot(axA, leg_i__x(:,j), leg_i__y(:,j), 'Color', gc_col_i, 'LineWidth', lW_i); 
                    hA{13} = plot(axA, leg_j__x(:,j), leg_j__y(:,j), 'Color', gc_col_i, 'LineWidth', lW_i);
                    hA{14} = plot(axA, mean_leg_i__x(:,j), mean_leg_i__y(:,j), 'LineStyle', '--', 'Color', gc_col_i, 'LineWidth', lW_r_i);
                    hA{15} = plot(axA, mean_leg_j__x(:,j), mean_leg_j__y(:,j), 'LineStyle', '--', 'Color', gc_col_i, 'LineWidth', lW_r_i);
                    hA{16} = scatter(axA, legtip_i__x(j), legtip_i__y(j), circS_i, gc_col_i, 'filled');
                    hA{17} = scatter(axA, legtip_j__x(j), legtip_j__y(j), circS_i, gc_col_i, 'filled');
                    hA{18} = plot(axA, ksqij__x(:,j), ksqij__y(:,j), 'Color', c_i, 'LineWidth', lW_kq_i, 'LineStyle', '--'); % ksq line
%                     pathtitle(axA, cs_i, pscale(i)/100, 1, ic_i, dirn_i*t_int_i, titleFS_i); %%%%%%%%%%%%%%%%%%% LEGACIED ON 20230404 (YYYYMMDD)
                    pathtitle(axA, cs_i, pscale(i)/100, 1, ic_i, dirn_i*t_int_i, titleFS_i);
                    % Child 1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    h1_s = cell(1,C1.num);
                    for k = 1:C1.num
                        h1_s{k} = scatter(C1.axes{k}, a1_i(j), a2_i(j), circS_i, gc_col_i, 'filled');
                    end

                    % Child 2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    h2_s = cell(1,C2.num);
                    for k = 1:C2.num
                        h2_s{k} = scatter(C2.axes{k}, a1_i(j), a2_i(j), circS_i, gc_col_i, 'filled');
                    end

                    % Child 3 current point SE(2) trajectory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    for k = 1:C3.num
                        switch k
                            case 1
                                bxt = scatter(C3.axes{k}, t_i(j), x_i(j), circS_i, gc_col_i, 'filled');
                            case 2
                                byt = scatter(C3.axes{k}, t_i(j), y_i(j), circS_i, gc_col_i, 'filled');
                            case 3
                                bthetat = scatter(C3.axes{k}, t_i(j), theta_i(j), circS_i, gc_col_i, 'filled');
                        end
                    end
                    
                    % updated the figure
                    drawnow();
                    
                    if vidF_i
                        
                        % store the frame
                        writeVideo(video,getframe(f));      

                    end

                end
                
                % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            end

            if vidF_i
                
                % close the video
                close(video);

            end

            % Return empty out structs
            dataij = [];

        case 3

            % Obtain the path information for i and j contact states
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
            cs_idx_i = datai{3}.cs_idx;
            ank_i = datai{3}.ank;
            
            path_j = dataj{5};
            pltkin_j = dataj{4};
            a_j = dataj{3}.aa; l_j = dataj{3}.ll;
            lW_j = dataj{1}.lW;
            lW_r_j = dataj{1}.lW_r;
            lW_kq_j = dataj{1}.lW_kq;
            lW_b_j = dataj{1}.lW_b;
            lW_qf_j = dataj{1}.lW_qf;
            circS_j = dataj{1}.circS;
            frame_scale_j = 0.25;
            c_j = path_j.path_active_color;
            zx_j = path_j.net_displacement(1,:);
            zy_j = path_j.net_displacement(2,:);
            ztheta_j = path_j.net_displacement(3,:);
            dnum_j = dataj{3}.dnum; 
            cs_idx_j = dataj{3}.cs_idx;
            ank_j = dataj{3}.ank;

            % get the integration direction
            dirn_i = path_i.int_dirn;
            dirn_j = path_j.int_dirn;

            % compute the 2-beat gait properties
            dataij = noslip2bgaits(path_i, path_j, dataij);

            % initialize the tiled layouts
            C3.Layout_Obj = tiledlayout(P,C3.grid(1),C3.grid(2),'TileSpacing','tight','Padding','tight'); 
            C3.Layout_Obj.Layout.Tile = C3.tile_start; C3.Layout_Obj.Layout.TileSpan = C3.span_grid;
            C4.Layout_Obj = tiledlayout(P,C4.grid(1),C4.grid(2),'TileSpacing','tight','Padding','tight'); 
            C4.Layout_Obj.Layout.Tile = C4.tile_start; C4.Layout_Obj.Layout.TileSpan = C4.span_grid;
            
            % Simulation parameters ----------------------------------------------------------------------------------------------------------------------------
            % number of gaits to animate
            numU = size(dataij.u, 1);
            % number of gait-cycles to plot for a chosen gait
            if ~isfield(dataij, 'gaitC_num')
                gaitC_num = 10;
            else
                gaitC_num = dataij.gaitC_num;
            end

            % plot the stratified panels for ith and jth contact states ----------------------------------------------------------------------------------------

            % Child 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ax = cell(1, C3.num);
            for i = 1:C3.num
                ax{i} = nexttile(C3.Layout_Obj,i);
                contourf(ax{i},a1_j,a2_j,dataj{2}.(C3.sweeptxt{i}),cfLvl_j,'FaceAlpha',fA_j,'LineWidth',lW_c_j,'LineStyle','none');
                axis equal tight; hold on; view(2);
                contour(ax{i}, a1_j,a2_j,ksq_sweep_j,[ksq_lb_j ksq_lb_j],'k--','LineWidth',lW_c_j);
                colormap(ax{i},CUB_j); clim(ax{i},C3_lim);
                set(get(ax{i},'YLabel'),'rotation',0,'VerticalAlignment','middle');
                title(ax{i},C3.titletxt{i},'Color','k',FontSize=titleFS_j); 
                if i == 1
                    xlabel(ax{i},x_label_txt_j,FontSize=labelFS_j); 
                    ylabel(ax{i},y_label_txt_j,FontSize=labelFS_j);
                end
                xticks(ax{i}, xtickval_j); yticks(ax{i}, ytickval_j);
                xticklabels(ax{i}, xticklab_j); yticklabels(ax{i}, yticklab_j);
                ax{i}.XAxis.FontSize = tickFS_j; ax{i}.YAxis.FontSize = tickFS_j; 
                set(ax{i},'Color',col_backg_j);
                xlim(xlimits_j); ylim(ylimits_j);
            end
            C3.axes = ax;
            if cond == 1
                C3.colorB = colorbar(C3.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS_j); C3.colorB.Layout.Tile = 'South';
%             elseif cond == 3
%                 title(C3.Layout_Obj, ['$S_{' num2str(cs_j(1)) num2str(cs_j(2)) '}$'], 'Color', gc_col_j, 'Interpreter', 'latex', FontSize=titleFS_j);
            end
            
            % Child 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ax = cell(1, C4.num);
            for i = 1:C4.num
                ax{i} = nexttile(C4.Layout_Obj,i);
                contourf(ax{i},a1_j,a2_j,dataj{2}.(C4.sweeptxt{i}),cfLvl_j,'FaceAlpha',fA_j,'LineWidth',lW_c_j,'LineStyle','none');
                axis equal tight; hold on; view(2);
                contour(ax{i}, a1_j,a2_j,ksq_sweep_j,[ksq_lb_j ksq_lb_j],'--k','LineWidth',lW_c_j);
                colormap(ax{i},CUB_j); clim(ax{i},C4_lim);
                set(get(ax{i},'YLabel'),'rotation',0,'VerticalAlignment','middle');
                title(ax{i},C4.titletxt{i},'Color','k',FontSize=titleFS_j);
                xticks(ax{i}, xtickval_j); yticks(ax{i}, ytickval_j);
                xticklabels(ax{i}, xticklab_j); yticklabels(ax{i}, yticklab_j);
                ax{i}.XAxis.FontSize = tickFS_j; ax{i}.YAxis.FontSize = tickFS_j; 
                set(ax{i},'Color',col_backg_j);
                xlim(xlimits_j); ylim(ylimits_j);
            end
            C4.axes = ax;
            if cond == 1
                C4.colorB = colorbar(C4.axes{i},'TickLabelInterpreter','latex','FontSize',cbarFS_j); C4.colorB.Layout.Tile = 'South';
            end

            % Unpack data from computed gait sweeps ------------------------------------------------------------------------------------------------------------
            u_i_setpt = dataij.u(:,1);  % input gait setpoints
            u_j_setpt = dataij.u(:,2);
            u_i = dataij.u_i;           % arrays
            u_j = dataij.u_j;
            gaits = dataij.gaits;       % ui, uj swept gaits
            
            % Setup the video and start animation layout -------------------------------------------------------------------------------------------------------
            if dataij.vidF
                video = VideoWriter(['data_animation','.mp4'],'MPEG-4');
                video.FrameRate = 30; % 20
                video.Quality = 100;
                open(video);
            end
            
            % create axes for the animation
            axA = nexttile(P, [3, 3]);
            title(P, ['$$S_{' num2str(cs_i(1)) num2str(cs_i(2))...
                '} \leftrightarrow S_{' num2str(cs_j(1)) num2str(cs_j(2)) '}$$'], 'Color', 'k', 'Interpreter', 'latex', FontSize=sgtitleFS_j);

            % ANIMATE/PLOT SYSTEM CONFIG -----------------------------------------------------------------------------------------------------------------------
            for i = 1:numU % iterate over gaits to animate

                % Find the gait we needed
                idxi = find(u_i == u_i_setpt(i)); idxj = find(u_j == u_j_setpt(i));

                % Find the path-lengths and points of interest
                t_int_i = path_i.path_length{ idxi };
                ic_i    = path_i.point_of_interest;
                t_int_j = path_j.path_length{ idxj };
                ic_j    = path_j.point_of_interest;

                % Obtain the kinematic data, plot it, and extend it for the number of gait cycles needed -------------------------------------------------------
                a1_i    = gaits{idxi, idxj}.trajectory{5} ;
                a2_i    = gaits{idxi, idxj}.trajectory{6} ;
                a1_j    = gaits{idxi, idxj}.trajectory{7} ;
                a2_j    = gaits{idxi, idxj}.trajectory{8} ;
                pbq     = gaits{idxi, idxj}.trajectory{9} ;
                idxiA   = gaits{idxi, idxj}.trajectory{10};
                idxiC   = gaits{idxi, idxj}.trajectory{11};
                idxjA   = gaits{idxi, idxj}.trajectory{12};
                idxjC   = gaits{idxi, idxj}.trajectory{13};
                phi_tau = gaits{idxi, idxj}.trajectory{14};
                
                % initialize plot object containers (if they already exist, they will be reinitialized and plots from previous gaits vanish)
                h1_A = cell(1, C1.num);     % child 1
                h1_N = h1_A;
                h1_s = h1_A;
                h1_arr = cell(1, C1.num);
                h2_A = cell(1, C2.num);     % child 2
                h2_N = h2_A;
                h2_s = h2_A;
                h2_arr = cell(1, C2.num);
                h3_A = cell(1, C3.num);     % child 3
                h3_N = h3_A;
                h3_s = h3_A;
                h3_arr = cell(1, C3.num);
                h4_A = cell(1, C4.num);     % child 4
                h4_N = h4_A;
                h4_s = h4_A;
                h4_arr = cell(1, C4.num);
                hA = cell(1, 0);            % animation plots

                % set the arrow size and angle for manual plotting
                arrSize = domainPercentage/100*2*ank_i;
                
                % Plot the shape-space trajectory in the respective shape-space slices
                for j = 1:C1.num
                    h1_A{j} = plot(C1.axes{j}, a1_i(idxiA), a2_i(idxiA), 'LineWidth', lW_s_i, 'Color', gc_col_i); % plot the active path for i
                    % h1_N{j} = plot(C1.axes{j}, a1_i(idxiC), a2_i(idxiC), '--', 'LineWidth', lW_s_i, 'Color', col_i(7,:)); % plot the inactive path for i
                    h1_arr{j} = plotpatharrow(C1.axes{j}, a1_i(idxiA), a2_i(idxiA), arrSize*t_int_i/2, arrAngle, lW_s_i, gc_col_i); % plot the arrow in the middle of the active path
                end
                for j = 1:C2.num
                    h2_A{j} = plot(C2.axes{j}, a1_i(idxiA), a2_i(idxiA), 'LineWidth', lW_s_i, 'Color', gc_col_i);
                    % h2_N{j} = plot(C2.axes{j}, a1_i(idxiC), a2_i(idxiC), '--', 'LineWidth', lW_s_i, 'Color', col_i(7,:));
                    h2_arr{j} = plotpatharrow(C2.axes{j}, a1_i(idxiA), a2_i(idxiA), arrSize*t_int_i/2, arrAngle, lW_s_i, gc_col_i);
                end
                for j = 1:C3.num
                    h3_A{j} = plot(C3.axes{j}, a1_j(idxjA), a2_j(idxjA), 'LineWidth', lW_s_j, 'Color', gc_col_j);
                    % h3_N{j} = plot(C3.axes{j}, a1_j(idxjC), a2_j(idxjC), '--', 'LineWidth', lW_s_j, 'Color', col_j(7,:));
                    h3_arr{j} = plotpatharrow(C3.axes{j}, a1_j(idxjA), a2_j(idxjA), arrSize*t_int_j/2, arrAngle, lW_s_j, gc_col_j);
                end
                for j = 1:C4.num
                    h4_A{j} = plot(C4.axes{j}, a1_j(idxjA), a2_j(idxjA), 'LineWidth', lW_s_j, 'Color', gc_col_j);
                    % h4_N{j} = plot(C4.axes{j}, a1_j(idxjC), a2_j(idxjC), '--', 'LineWidth', lW_s_j, 'Color', col_j(7,:));
                    h4_arr{j} = plotpatharrow(C4.axes{j}, a1_j(idxjA), a2_j(idxjA), arrSize*t_int_j/2, arrAngle, lW_s_j, gc_col_j);
                end
                
                % Obtain the integration time for the ic and main path
                temp_ti = cumsum(path_i.int_time);
                temp_tj = cumsum(path_j.int_time);
                t_int_i = path_i.path_length{ idxi }/temp_ti(2) * temp_ti;
                t_int_j = path_j.path_length{ idxj }/temp_tj(2) * temp_tj;

                a1_i    = repmat(a1_i,    1, gaitC_num);
                a2_i    = repmat(a2_i,    1, gaitC_num);
                a1_j    = repmat(a1_j,    1, gaitC_num);
                a2_j    = repmat(a2_j,    1, gaitC_num);
                pbq     = repmat(pbq,     1, gaitC_num);
%                 idxiA   = repmat(idxiA,   1, gaitC_num);
%                 idxiC   = repmat(idxiC,   1, gaitC_num);
%                 idxjA   = repmat(idxjA,   1, gaitC_num);
%                 idxjC   = repmat(idxjC,   1, gaitC_num);
                phi_tau = repmat(phi_tau, 1, gaitC_num);
                
                % Extend the position trajectory to number of gait cycles needed
                t       = gaits{idxi, idxj}.trajectory{1} ;
                x       = gaits{idxi, idxj}.trajectory{2} ;
                y       = gaits{idxi, idxj}.trajectory{3} ;
                theta   = gaits{idxi, idxj}.trajectory{4} ;
                zx      = gaits{idxi, idxj}.trajectory{15};
                zy      = gaits{idxi, idxj}.trajectory{16};
                ztheta  = gaits{idxi, idxj}.trajectory{17};
                tf      = sum(gaits{idxi, idxj}.periods.phi_tau);
                temp_t  = t;
                z = [zx; zy; ztheta]; ztemp = z;
                temp = [x; y; theta]; b = temp;
                for j = 2:gaitC_num % iterate and extend
                    % compute next
                    xyztemp = ztemp + rot_SE2(ztemp(3))*temp;   % update trajectory
                    ztemp= ztemp + rot_SE2(ztemp(3))*z;  % update net displacement
                    temp_t = tf+temp_t;
                    % update current
                    t = [t, temp_t];
                    b = [b, xyztemp];
                end
                x = b(1,:); y = b(2,:); theta = b(3,:);
                dnum = numel(t);

                % compute the leg angles
                a_n            = nan(4, numel(t));
                a_n(cs_i(1), :) = a1_i;
                a_n(cs_i(2), :) = a2_i;
                a_n(cs_j(1), :) = a1_j;
                a_n(cs_j(2), :) = a2_j;
                
                % compute the animation limits
                bl = 4*l_j;
                thresh = 1.25;
                anim_lim = [min(x, [], 'all')-thresh*bl, max(x, [], 'all')+thresh*bl, min(y, [], 'all')-thresh*bl, max(y, [], 'all')+thresh*bl];

                % Compute the animation quantities for the entire gait
                            % contact state i
                leg_i1__x = pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_x'])(a_i, l_i, a1_i, x, y, theta); leg_i1__x = [leg_i1__x(1:dnum); leg_i1__x(dnum+1:end)];
                leg_i1__y = pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_y'])(a_i, l_i, a1_i, x, y, theta); leg_i1__y = [leg_i1__y(1:dnum); leg_i1__y(dnum+1:end)];
                leg_i2__x = pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_x'])(a_i, l_i, a2_i, x, y, theta); leg_i2__x = [leg_i2__x(1:dnum); leg_i2__x(dnum+1:end)];
                leg_i2__y = pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_y'])(a_i, l_i, a2_i, x, y, theta); leg_i2__y = [leg_i2__y(1:dnum); leg_i2__y(dnum+1:end)];
                legtip_i1__x = pltkin_i.(['leg' num2str(cs_i(1)) '_x'])(a_i, l_i, a1_i, x, y, theta);
                legtip_i1__y = pltkin_i.(['leg' num2str(cs_i(1)) '_y'])(a_i, l_i, a1_i, x, y, theta);
                legtip_i2__x = pltkin_i.(['leg' num2str(cs_i(2)) '_x'])(a_i, l_i, a2_i, x, y, theta); 
                legtip_i2__y = pltkin_i.(['leg' num2str(cs_i(2)) '_y'])(a_i, l_i, a2_i, x, y, theta);
                mean_leg_i1__x = pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_x'])(a_i, l_i, zeros(1,dnum), x, y, theta); mean_leg_i1__x = [mean_leg_i1__x(1:dnum); mean_leg_i1__x(dnum+1:end)];
                mean_leg_i1__y = pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_y'])(a_i, l_i, zeros(1,dnum), x, y, theta); mean_leg_i1__y = [mean_leg_i1__y(1:dnum); mean_leg_i1__y(dnum+1:end)];
                mean_leg_i2__x = pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_x'])(a_i, l_i, zeros(1,dnum), x, y, theta); mean_leg_i2__x = [mean_leg_i2__x(1:dnum); mean_leg_i2__x(dnum+1:end)];
                mean_leg_i2__y = pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_y'])(a_i, l_i, zeros(1,dnum), x, y, theta); mean_leg_i2__y = [mean_leg_i2__y(1:dnum); mean_leg_i2__y(dnum+1:end)];
                ksqi__x = pltkin_i.(['k_leg' num2str(cs_i(1)) '_leg' num2str(cs_i(2)) '_x'])(a_i, l_i, a1_i, a2_i, x, y, theta); ksqi__x = [ksqi__x(1:dnum); ksqi__x(dnum+1:end)];
                ksqi__y = pltkin_i.(['k_leg' num2str(cs_i(1)) '_leg' num2str(cs_i(2)) '_y'])(a_i, l_i, a1_i, a2_i, x, y, theta); ksqi__y = [ksqi__y(1:dnum); ksqi__y(dnum+1:end)];
                            % contact state j
                leg_j1__x = pltkin_j.(['legbase' num2str(cs_j(1)) '_leg' num2str(cs_j(1)) '_x'])(a_j, l_j, a1_j, x, y, theta); leg_j1__x = [leg_j1__x(1:dnum); leg_j1__x(dnum+1:end)];
                leg_j1__y = pltkin_j.(['legbase' num2str(cs_j(1)) '_leg' num2str(cs_j(1)) '_y'])(a_j, l_j, a1_j, x, y, theta); leg_j1__y = [leg_j1__y(1:dnum); leg_j1__y(dnum+1:end)];
                leg_j2__x = pltkin_j.(['legbase' num2str(cs_j(2)) '_leg' num2str(cs_j(2)) '_x'])(a_j, l_j, a2_j, x, y, theta); leg_j2__x = [leg_j2__x(1:dnum); leg_j2__x(dnum+1:end)];
                leg_j2__y = pltkin_j.(['legbase' num2str(cs_j(2)) '_leg' num2str(cs_j(2)) '_y'])(a_j, l_j, a2_j, x, y, theta); leg_j2__y = [leg_j2__y(1:dnum); leg_j2__y(dnum+1:end)];
                legtip_j1__x = pltkin_j.(['leg' num2str(cs_j(1)) '_x'])(a_j, l_j, a1_j, x, y, theta);
                legtip_j1__y = pltkin_j.(['leg' num2str(cs_j(1)) '_y'])(a_j, l_j, a1_j, x, y, theta);
                legtip_j2__x = pltkin_j.(['leg' num2str(cs_j(2)) '_x'])(a_j, l_j, a2_j, x, y, theta); 
                legtip_j2__y = pltkin_j.(['leg' num2str(cs_j(2)) '_y'])(a_j, l_j, a2_j, x, y, theta);
                mean_leg_j1__x = pltkin_j.(['legbase' num2str(cs_j(1)) '_leg' num2str(cs_j(1)) '_x'])(a_j, l_j, zeros(1,dnum), x, y, theta); mean_leg_j1__x = [mean_leg_j1__x(1:dnum); mean_leg_j1__x(dnum+1:end)];
                mean_leg_j1__y = pltkin_j.(['legbase' num2str(cs_j(1)) '_leg' num2str(cs_j(1)) '_y'])(a_j, l_j, zeros(1,dnum), x, y, theta); mean_leg_j1__y = [mean_leg_j1__y(1:dnum); mean_leg_j1__y(dnum+1:end)];
                mean_leg_j2__x = pltkin_j.(['legbase' num2str(cs_j(2)) '_leg' num2str(cs_j(2)) '_x'])(a_j, l_j, zeros(1,dnum), x, y, theta); mean_leg_j2__x = [mean_leg_j2__x(1:dnum); mean_leg_j2__x(dnum+1:end)];
                mean_leg_j2__y = pltkin_j.(['legbase' num2str(cs_j(2)) '_leg' num2str(cs_j(2)) '_y'])(a_j, l_j, zeros(1,dnum), x, y, theta); mean_leg_j2__y = [mean_leg_j2__y(1:dnum); mean_leg_j2__y(dnum+1:end)];
                ksqj__x = pltkin_j.(['k_leg' num2str(cs_j(1)) '_leg' num2str(cs_j(2)) '_x'])(a_j, l_j, a1_j, a2_j, x, y, theta); ksqj__x = [ksqj__x(1:dnum); ksqj__x(dnum+1:end)];
                ksqj__y = pltkin_j.(['k_leg' num2str(cs_j(1)) '_leg' num2str(cs_j(2)) '_y'])(a_j, l_j, a1_j, a2_j, x, y, theta); ksqj__y = [ksqj__y(1:dnum); ksqj__y(dnum+1:end)];
                            % standard stuff
                leg_1__x = pltkin_i.legbase1_leg1_x(a_i, l_i, a_n(1, :), x, y, theta); leg_1__x = [leg_1__x(1:dnum); leg_1__x(dnum+1:end)];
                leg_1__y = pltkin_i.legbase1_leg1_y(a_i, l_i, a_n(1, :), x, y, theta); leg_1__y = [leg_1__y(1:dnum); leg_1__y(dnum+1:end)];
                leg_2__x = pltkin_i.legbase2_leg2_x(a_i, l_i, a_n(2, :), x, y, theta); leg_2__x = [leg_2__x(1:dnum); leg_2__x(dnum+1:end)];
                leg_2__y = pltkin_i.legbase2_leg2_y(a_i, l_i, a_n(2, :), x, y, theta); leg_2__y = [leg_2__y(1:dnum); leg_2__y(dnum+1:end)];
                leg_3__x = pltkin_i.legbase3_leg3_x(a_i, l_i, a_n(3, :), x, y, theta); leg_3__x = [leg_3__x(1:dnum); leg_3__x(dnum+1:end)];
                leg_3__y = pltkin_i.legbase3_leg3_y(a_i, l_i, a_n(3, :), x, y, theta); leg_3__y = [leg_3__y(1:dnum); leg_3__y(dnum+1:end)];
                leg_4__x = pltkin_i.legbase4_leg4_x(a_i, l_i, a_n(4, :), x, y, theta); leg_4__x = [leg_4__x(1:dnum); leg_4__x(dnum+1:end)];
                leg_4__y = pltkin_i.legbase4_leg4_y(a_i, l_i, a_n(4, :), x, y, theta); leg_4__y = [leg_4__y(1:dnum); leg_4__y(dnum+1:end)];
                top__x = pltkin_i.topright2topleft_x(l_i, x, y, theta); top__x = [top__x(1:dnum); top__x(dnum+1:end)];
                top__y = pltkin_i.topright2topleft_y(l_i, x, y, theta); top__y = [top__y(1:dnum); top__y(dnum+1:end)];
                left__x = pltkin_i.topleft2botleft_x(l_i, x, y, theta); left__x = [left__x(1:dnum); left__x(dnum+1:end)];
                left__y = pltkin_i.topleft2botleft_y(l_i, x, y, theta); left__y = [left__y(1:dnum); left__y(dnum+1:end)];
                bot__x = pltkin_i.botleft2botright_x(l_i, x, y, theta); bot__x = [bot__x(1:dnum); bot__x(dnum+1:end)];
                bot__y = pltkin_i.botleft2botright_y(l_i, x, y, theta); bot__y = [bot__y(1:dnum); bot__y(dnum+1:end)];
                right__x = pltkin_i.botright2topright_x(l_i, x, y, theta); right__x = [right__x(1:dnum); right__x(dnum+1:end)];
                right__y = pltkin_i.botright2topright_y(l_i, x, y, theta); right__y = [right__y(1:dnum); right__y(dnum+1:end)];
                body__x = pltkin_i.body_x(x, y, theta); 
                body__y = pltkin_i.body_y(x, y, theta);
                bodyf__x = pltkin_i.bodyf_x(x, y, theta); 
                bodyf__y = pltkin_i.bodyf_y(x, y, theta);
                
                % time iteration
                if ~dataij.vidF
%                     T = t(pbq == 1); % this gets a little too busy for figure generation purposes
                    T = t(find(pbq == 1, 1, 'last'));
                else
                    T = t(1:gaitC_num:numel(t)); % add the body configuration points to be plotted
                end
                m = 1; % plotting count

                % Iterate over time and animate ----------------------------------------------------------------------------------------------------------------
                for j = 1:numel(T)
                    
                    % delete everything we don't need
                    if j ~= 1 % && j ~= numel(T)

                        for k = 1:numel(hA)
                            delete(hA{k});
                        end
                        for k = 1:numel(h1_s)
                            delete(h1_s{k});
                        end
                        for k = 1:numel(h2_s)
                            delete(h2_s{k});
                        end
                        for k = 1:numel(h3_s)
                            delete(h3_s{k});
                        end
                        for k = 1:numel(h4_s)
                            delete(h4_s{k});
                        end

                    end

                    % find the current step
                    k = find(t == T(j));
                    if numel(T) > 1
                        if j == 1
                            if T(j) == 0
                                kold = nan;
                            else
                                kold = 1;
                            end
                        else
                            if T(j-1) == 0
                                kold = 1;
                            else
                                kold = find(t == T(j-1)) - 1;
                            end
                        end
                    else
                        kold = 1;
                    end
                    
                    % plot the standard stuff and body axes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    hA{m}  = plot(axA, leg_1__x(:,k), leg_1__y(:,k), 'Color', col_i(7,:), 'LineWidth', lW_i); m = m + 1;
                    if j == 1
                        set(axA, 'xticklabel', []); set(axA, 'yticklabel', []); box("off");
                        xline(axA, 0, ':', 'LineWidth', 0.5, 'Color', 'k');
                        yline(axA, 0, ':', 'LineWidth', 0.5, 'Color', 'k');
                    end 
                    hold on; axis equal;
                    axis(anim_lim);
                    hA{m}  = plot(axA, leg_2__x(:,k), leg_2__y(:,k), 'Color', col_i(7,:), 'LineWidth', lW_i); m = m + 1;
                    hA{m}  = plot(axA, leg_3__x(:,k), leg_3__y(:,k), 'Color', col_j(7,:), 'LineWidth', lW_j); m = m + 1;
                    hA{m}  = plot(axA, leg_4__x(:,k), leg_4__y(:,k), 'Color', col_j(7,:), 'LineWidth', lW_j); m = m + 1;
                    hA{m}  = plot(axA, top__x(:,k), top__y(:,k), 'Color', col_i(7,:), 'LineWidth', lW_b_i); m = m + 1;
                    hA{m}  = plot(axA, left__x(:,k), left__y(:,k), 'Color', col_i(7,:), 'LineWidth', lW_b_i); m = m + 1;
                    hA{m}  = plot(axA, bot__x(:,k), bot__y(:,k), 'Color', col_j(7,:), 'LineWidth', lW_b_j); m = m + 1;
                    hA{m}  = plot(axA, right__x(:,k), right__y(:,k), 'Color', col_j(7,:), 'LineWidth', lW_b_j); m = m + 1;

                    % plot the trajectory ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    if ~isnan(kold) % if this is not the first time step
                        ptrajectory2bnoslip(axA, phi_tau, k, kold, x, y, cs_idx_i, cs_idx_j, gc_col_i, lW_s_i, gc_col_j, lW_s_j, col_i(7,:) );
                    end
                    
                    % plot the contact state related stuff ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    % current contact state related plotting parameters, and active legs with inter-leg vector
                    if phi_tau(k) == cs_idx_i % 1st state
                        frame_scale = frame_scale_i;
                        lW_qf = lW_qf_i;
                        col_now = gc_col_i;
                        c_1 = col_now;
                        c_2 = col_i(7,:);
                        hA{m} = plot(axA, leg_i1__x(:,k), leg_i1__y(:,k), 'Color', col_now, 'LineWidth', lW_i); m = m + 1;
                        hA{m} = plot(axA, leg_i2__x(:,k), leg_i2__y(:,k), 'Color', col_now, 'LineWidth', lW_i); m = m + 1;
                        hA{m} = plot(axA, mean_leg_i1__x(:,k), mean_leg_i1__y(:,k), 'LineStyle', '--', 'Color', col_now, 'LineWidth', lW_r_i); m = m + 1;
                        hA{m} = plot(axA, mean_leg_i2__x(:,k), mean_leg_i2__y(:,k), 'LineStyle', '--', 'Color', col_now, 'LineWidth', lW_r_i); m = m + 1;
                        hA{m} = scatter(axA, legtip_i1__x(k), legtip_i1__y(k), circS_i, col_now, 'filled'); m = m + 1;
                        hA{m} = scatter(axA, legtip_i2__x(k), legtip_i2__y(k), circS_i, col_now, 'filled'); m = m + 1;
                        hA{m} = plot(axA, ksqi__x(:,k), ksqi__y(:,k), 'Color', c_i, 'LineWidth', lW_kq_i, 'LineStyle', '--'); m = m + 1; % ksq_plot
                    elseif phi_tau(k) == cs_idx_j % 2nd state
                        frame_scale = frame_scale_j;
                        lW_qf = lW_qf_j;
                        col_now = gc_col_j;
                        c_1 = col_i(7,:);
                        c_2 = col_now; 
                        hA{m} = plot(axA, leg_j1__x(:,k), leg_j1__y(:,k), 'Color', col_now, 'LineWidth', lW_j); m = m + 1;
                        hA{m} = plot(axA, leg_j2__x(:,k), leg_j2__y(:,k), 'Color', col_now, 'LineWidth', lW_j); m = m + 1;
                        hA{m} = plot(axA, mean_leg_j1__x(:,k), mean_leg_j1__y(:,k), 'LineStyle', '--', 'Color', col_now, 'LineWidth', lW_r_j); m = m + 1;
                        hA{m} = plot(axA, mean_leg_j2__x(:,k), mean_leg_j2__y(:,k), 'LineStyle', '--', 'Color', col_now, 'LineWidth', lW_r_j); m = m + 1;
                        hA{m} = scatter(axA, legtip_j1__x(k), legtip_j1__y(k), circS_j, col_now, 'filled'); m = m + 1;
                        hA{m} = scatter(axA, legtip_j2__x(k), legtip_j2__y(k), circS_j, col_now, 'filled'); m = m + 1;
                        hA{m} = plot(axA, ksqj__x(:,k), ksqj__y(:,k), 'Color', c_j, 'LineWidth', lW_kq_j, 'LineStyle', '--'); m = m + 1; % ksq_plot
                    else % no active state
                        frame_scale = frame_scale_j;
                        lW_qf = lW_qf_j;
                        col_now = col_j(7,:);
                        c_1 = col_i(7,:);
                        c_2 = col_j(7,:); 
                    end

                    % title based on the control input
% % % % %                     pathtitle(axA, [cs_i; cs_j], [u_i; u_j], [idxi; idxj], [ic_i; ic_j], [dirn_i*t_int_i; dirn_j*t_int_j], titleFS_j); %%%%%%%%%%%%%%%%%%% LEGACIED ON 20230404 (YYYYMMDD)
                    pathtitle(axA, [cs_i; cs_j], [u_i; u_j], [idxi; idxj], [ic_i; ic_j], [dirn_i*t_int_i; dirn_j*t_int_j], titleFS_j); 
%                     title(axA, ['$$ \left(' num2str(u_i(idxi), 3) '\right) \, \psi^{' num2str(cs_i(1)) num2str(cs_i(2))...
%                         '} \left( (' num2str(ic_i(1), 3) ', ' num2str(ic_i(2), 3) '), ' num2str(t_int_i, 3) ' \right) + \left('...
%                         num2str(u_j(idxj), 3) '\right) \, \psi^{' num2str(cs_j(1)) num2str(cs_j(2))...
%                         '} \left( (' num2str(ic_j(1), 3) ', ' num2str(ic_j(2), 3) '), ' num2str(t_int_j, 3) ' \right) $$'], 'Color', 'k',... % col_j(1,:)
%                         'Interpreter', 'latex', FontSize=titleFS_j);
                    % SE(2) body frame 'b'
                    hA{m} = quiver(axA, body__x(k), body__y(k), frame_scale*bodyf__x(1,k), frame_scale*bodyf__x(2,k), 'LineWidth', lW_qf, 'Color', col_now,...
                            'AutoScale', 'off', 'ShowArrowHead', 'off'); m = m + 1;
                    hA{m} = quiver(axA, body__x(k), body__y(k), frame_scale*bodyf__y(1,k), frame_scale*bodyf__y(2,k), 'LineWidth', lW_qf, 'Color', col_now,...
                        'AutoScale', 'off', 'ShowArrowHead', 'off'); m = m + 1;

                    % layout scatter points ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    for l = 1:C1.num
                        h1_s{l} = scatter(C1.axes{l}, a1_i(k), a2_i(k), circS_i, c_1, 'filled');
                    end
                    for l = 1:C2.num
                        h2_s{l} = scatter(C2.axes{l}, a1_i(k), a2_i(k), circS_i, c_1, 'filled');
                    end
                    for l = 1:C3.num
                        h3_s{l} = scatter(C3.axes{l}, a1_j(k), a2_j(k), circS_j, c_2, 'filled');
                    end
                    for l = 1:C4.num
                        h4_s{l} = scatter(C4.axes{l}, a1_j(k), a2_j(k), circS_j, c_2, 'filled');
                    end

                    drawnow;

                    % save the frame if video is requested ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    if dataij.vidF
                        writeVideo(video,getframe(f));
                    end

                    
                end
                

            end
            
            if dataij.vidF
                close(video);
            end

    end

end

