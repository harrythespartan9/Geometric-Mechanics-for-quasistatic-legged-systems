% This script computes the kinematics of the level-2 contact states of a
% quadrupedal robot undergoing no-slip condition
function [plot_info, plot_kin, f] = quadrupedlevel2noslip(kin_info, plot_info)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Computed Plot Information %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% unpack user inputs ------------------------------------------------------
cs = kin_info.cs;
fields = kin_info.fields; configs = kin_info.configs;
ank = kin_info.ank; aa = kin_info.aa; ll = kin_info.ll;

dnum = plot_info.dnum; skipV = plot_info.skipV; sV = plot_info.sV;
cLvl = plot_info.cLvl;
fontscale = plot_info.fontscale;

% obtain the storage index of the current submanifold ---------------------
% The level-2 contact states (not more than two feet pinned to the ground 
% at any given point in time) or the contact submanifolds are organized as
% follows: [12, 23, 34, 41, 13, 24]. Each digit identifies the feet pinned
% to the ground. For the user chosen feet, we identify which contact state
% it belongs to through the function below.
kin_info = returnSijfxn(kin_info,cs(1),cs(2));

% -------------------------------------------------------------------------
% compute parameters dependent on the parameters provided -----------------
% -------------------------------------------------------------------------
plot_info.configs = num2cell([nan(3,1),configs]); % modify the config format
plot_info.fields = fields; % the columns requested
plot_info.m = 1920*(numel(fields)+1)/5; % scaled figure y-resolution
plot_info.n = 1440; % fixed figure x-resolution
plot_info.tickFS = 18*fontscale; 
plot_info.labelFS = 18*fontscale; % compute various font sizes
plot_info.titleFS = 20*fontscale; 
plot_info.sgtitleFS = 25*fontscale;
plot_info.cbarFS = 15*fontscale;
plot_info.lW_contour = (1.0*10)/cLvl; % contour linewidth % 1.2
plot_info.lW_Vector = (1.0/100)*skipV; % vector linewidth
plot_info.lW_V = (1.0/100)*sV;
plot_info.idxQ = 1:skipV:dnum; % indices to plot the connection vector field
plot_info.iQ = 1:sV:dnum; % gait constraint vector fields
plot_info.xtickval = -pi/3:pi/3:pi/3;
plot_info.xticklab = {'$-\frac{\pi}{3}$','$0$','$\frac{\pi}{3}$'};
plot_info.ytickval = plot_info.xtickval;
plot_info.yticklab = plot_info.xticklab;

% Layout column and row initialization ------------------------------------
plot_info.C_fields = (1:numel(fields)) + 1; % the default column is 1
comps = {'x','y','theta'};
plot_info.comps = comps; % this is predefined and fixed.
plot_info.R_comps = 1:numel(comps);

% custom colormaps --------------------------------------------------------
% for squared inter-leg distance 'ksq'
% jetDark = flipud([215,48,39; % diverging
%                 244,109,67;
%                 253,174,97;
%                 254,224,144;
%                 255,255,191;
%                 224,243,248;
%                 171,217,233;
%                 116,173,209;
%                 69,117,180])/255;
% jetDark = [128,125,186; 
%             106,81,163;
%             84,39,143;
%             63,0,125]/255; % sequential violet
% jetDark = [127,205,187;
%             65,182,196;
%             29,145,192;
%             34,94,168;
%             37,52,148]/255; % sequential blue
jetDark = flipud([174,1,126;
           221,52,151;
           247,104,161;
           250,159,181;
           116,169,207;
           54,144,192])/255; % diverging pink to blue
% jetDark = flipud([152,0,67;
%            206,18,86;
%            231,41,138;
%            223,101,176;
%            116,169,207;
%            54,144,192])/255; % diverging reddish-pink to blue
% jetDark = turbo(256); % initialize the colormap based on jet % 256
jetDark = interp1(linspace(0,100,size(jetDark,1)), jetDark, linspace(0,100,size(jet,1))); % initialize the map based on colorbrewer
% USE SCALING ONLY WHEN USING TURBO MAP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% % % jP = sqrt(jetDark(:,1).^2 + jetDark(:,2).^2 + jetDark(:,3).^2); % power in each slice
% % % jS = 0.9; % a proportional gain helping us shift the color levels % 0.8
% % % % 0.88 % 0.9 % 1.0 % max(jP)-- no changes to color intensity
% % % plot_info.jetDark = jS/max(jP)*repmat(jP,1,3).*jetDark; %%% proportional reduction with normalized power
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
plot_info.jetDark = jetDark; % don't scale for manually defined maps

% for stratified panels
CUB = flipud([159.3750,99.6030,63.4312;
          239.0625,149.4045,95.1469;
          255.0000,199.2060,126.8625;
            204,204,204;
            150,150,150;
            99,99,99])/255;
plot_info.CUB = interp1(linspace(0,100,size(CUB,1)), CUB, linspace(0,100,size(turbo,1)));
% plot_info.col = (1/255)*[215,25,28; % contact state colors (distinct)
%                         96,0,220;
%                         44,123,182;
%                         77,175,74;
%                         255,127,0;
%                         231,41,138;
%                         99,99,99];
plot_info.col = (1/255)*[0,0,0; % contact state colors (active- black and inactive- grey)
            0,0,0;
            0,0,0;
            0,0,0;
            0,0,0;
            0,0,0;
            72,72,72];
plot_info.gc_col = plot_info.col(kin_info.cs_idx,:); % get the current gait constraint color
plot_info.col_backg = [255,255,255]/255; % plot background color

% shape-space grid and plotting stuff -------------------------------------
% grid
plot_kin = [];
dnum_right = ceil(dnum/2); dnum_left = dnum - dnum_right;
temp_left = linspace(-ank,0,dnum_left+1);  temp_right = linspace(0,ank,dnum_right);
tempX = [temp_left(1:end-1), temp_right]; tempY = tempX;
[ai,aj] = meshgrid(tempX,tempY); % origin needs to be a part of the grid
plot_kin.ai = ai;
plot_kin.aj = aj;

% robot config selection and heuristics -----------------------------------
ijm = find(tempX>-pi/2 & tempX<pi/2, 1); 
ijM = find(tempX>-pi/2 & tempX<pi/2, 1, 'last');
ii = randi([ijm ijM]); jj = randi([ijm ijM]); % get a random shape over the restricted
                                  % range of [-pi/2,pi/2]
plot_info.i = ii; plot_info.j = jj;
% plot_info.i = 729; plot_info.j = 733; % manually chosen 
plot_info.xx = 3; plot_info.yy = 3; plot_info.th = pi/4; % pi/4
limX = [-1, 7]; limY = [-1, 7];
scaleDraw = max([diff(limX),diff(limY)])/23.5;
lW = 1.2/scaleDraw; lW_r = 0.5/scaleDraw; lW_b = 0.8/scaleDraw;
lW_kq = 0.6/scaleDraw; lW_qf = 0.8/scaleDraw;
frame_scale = 0.25; % length of the frame arrows compared to body length
% lW_O = 0.8; alp_O = 0.2; lW_traj = 2.4/scaleDraw; 
circS = 75/scaleDraw;
circS_q = 200; % config highlight
boxW = 1.2; % config plot box linewidth
lW_m = 0.8; % fixed marker edge width and type for robot configuration
plot_info.lW = lW; plot_info.lW_r = lW_r; plot_info.lW_b = lW_b;
plot_info.lW_kq = lW_kq; plot_info.lW_qf = lW_qf; plot_info.lW_m = lW_m;
plot_info.circS = circS; plot_info.circS_q = circS_q;
plot_info.boxW = boxW;

% -------------------------------------------------------------------------
% Robot kinematic heatmaps and vector field computation 
% shape-space grid --------------------------------------------------------
% -------------------------------------------------------------------------

% texts to use when plotting ----------------------------------------------
% Ideally you want to iterate through all the shapes in a loop, but since
% we are in a lvl2 submanifold, we can do this manually. From, level-3
% onwards we can consider loping.
cs1_shape_txt = ['alpha_' num2str(cs(1))]; 
cs2_shape_txt = ['alpha_' num2str(cs(2))];
x_label_txt = ['$\' 'alpha_{' num2str(cs(1)) '}' '$']; % attaching dollahs for latex interp
y_label_txt = ['$\' 'alpha_{' num2str(cs(2)) '}' '$'];
s_txt = [num2str(cs(1)) num2str(cs(2))];
sgtitle_txt = ['$S_{' s_txt '}$'];
q_title_txt = '$q$'; 
ksq_title_text = ['$k_{' s_txt '}^{2}$'];
dphi_title_text = ['$d\phi_{' s_txt '}$'];
plot_info.cs1_shape_txt = cs1_shape_txt;
plot_info.cs2_shape_txt = cs2_shape_txt;
plot_info.x_label_txt = x_label_txt;
plot_info.y_label_txt = y_label_txt;
plot_info.s_txt = s_txt;
plot_info.sgtitle_txt = sgtitle_txt;
plot_info.q_title_txt = q_title_txt;
plot_info.ksq_title_text = ksq_title_text;
plot_info.dphi_title_text = dphi_title_text;

% get the symbolic variables needed to setup the kinematics ---------------
load('case_1_kinematics');
syms a l real positive
assume(a<=1);
eval(['syms ' cs1_shape_txt ' ' cs2_shape_txt ' real']);

% compute everything we need based on the input submanifold ---------------
a1 = 0; a2 = 0;
a3 = 0; a4 = 0; % config initialization
switch kin_info.cs_idx
    case 1 % S12 
        % Compute the configuration to plot
        a1 = ai(ii,jj); a2 = aj(ii,jj);
    case 2 % S23
        a2 = ai(ii,jj); a3 = aj(ii,jj);
    case 3 % S34
        a3 = ai(ii,jj); a4 = aj(ii,jj);
    case 4 % S41
        a4 = ai(ii,jj); a1 = aj(ii,jj);
    case 5 % S13
        a1 = ai(ii,jj); a3 = aj(ii,jj);
    case 6 % S24
        a2 = ai(ii,jj); a4 = aj(ii,jj);
end

% Compute the kinematics functions for the submanifold --------------------
% flags to check which functions to compute based on user input fields
aF = false; daF = false; lbaF = false; gcurlaF = false; dzF = false; % init
if sum(strcmp('-A',fields))
    aF = true;
end
if sum(strcmp('-dA',fields))
    daF = true;
end
if sum(strcmp('[A_1,A_2]',fields))
    lbaF = true;
end
if sum(strcmp('D(-A)',fields))
    gcurlaF = true;
end
if sum(strcmp('dz',fields))
    dzF = true;
end

% submanifold index and symbolic variables needed to compute functions
i = kin_info.cs_idx; % assign the submanifold index
symvarsij = eval(['[a, l, ' cs1_shape_txt ', ' cs2_shape_txt ']']); % fxn vars

% compute functions, sweeps, and limits
% ksq (always computed)
ksq = kin.ksq_ij{i};
ksq_sweep = ksq(aa, ll, ai, aj); plot_kin.ksq_sweep = conditiondatasweep(ksq_sweep,[dnum,dnum]);
plot_info.col_q = interp1(linspace(min(ksq_sweep,[],'all'), max(ksq_sweep,[],'all')...
    , size(jetDark,1)), jetDark, ksq_sweep(ii, jj)); % get the inter-leg color for the chosen ksq val
% dphi (always computed)
% dphi_x = kin.dphi_ij_s{i,1}; dphi_y = kin.dphi_ij_s{i,2}; % scaled version
dphi = simplify(kin.dphi_ij{i}/norm(kin.dphi_ij{i}),'Steps',10); 
dphi_x = matlabFunction(dphi(1),'Vars',symvarsij); 
dphi_y = matlabFunction(dphi(2),'Vars',symvarsij); % normalized version
dphi_x_sweep = dphi_x(aa, ll, ai, aj); plot_kin.dphi_x_sweep = conditiondatasweep(dphi_x_sweep,[dnum,dnum]);
dphi_y_sweep = dphi_y(aa, ll, ai, aj); plot_kin.dphi_y_sweep = conditiondatasweep(dphi_y_sweep,[dnum,dnum]);
% connection
if aF
    A__x_1 = matlabFunction(kin.Ax_ij{i}{1,1},'Vars',symvarsij);
    A__x_2 = matlabFunction(kin.Ax_ij{i}{1,2},'Vars',symvarsij);
    A__y_1 = matlabFunction(kin.Ay_ij{i}{1,1},'Vars',symvarsij); 
    A__y_2 = matlabFunction(kin.Ay_ij{i}{1,2},'Vars',symvarsij);
    A__theta_1 = matlabFunction(kin.Atheta_ij{i}{1,1},'Vars',symvarsij); 
    A__theta_2 = matlabFunction(kin.Atheta_ij{i}{1,2},'Vars',symvarsij);
    A__x_1_sweep = A__x_1(aa, ll, ai, aj); plot_kin.A__x_1_sweep = conditiondatasweep(A__x_1_sweep,[dnum,dnum]);
    A__x_2_sweep = A__x_2(aa, ll, ai, aj); plot_kin.A__x_2_sweep = conditiondatasweep(A__x_2_sweep,[dnum,dnum]);
    A__y_1_sweep = A__y_1(aa, ll, ai, aj); plot_kin.A__y_1_sweep = conditiondatasweep(A__y_1_sweep,[dnum,dnum]);
    A__y_2_sweep = A__y_2(aa, ll, ai, aj); plot_kin.A__y_2_sweep = conditiondatasweep(A__y_2_sweep,[dnum,dnum]);
    A__theta_1_sweep = A__theta_1(aa, ll, ai, aj); plot_kin.A__theta_1_sweep = conditiondatasweep(A__theta_1_sweep,[dnum,dnum]);
    A__theta_2_sweep = A__theta_2(aa, ll, ai, aj); plot_kin.A__theta_2_sweep = conditiondatasweep(A__theta_2_sweep,[dnum,dnum]);
end
% continuous kinematics-- ross and howie flavor
da_xy_lim = nan(1,2); lba_xy_lim = nan(1,2); gcurla_xy_lim = nan(1,2); 
da_theta_lim = nan(1,2); lba_theta_lim = nan(1,2); gcurla_theta_lim = nan(1,2); % limits init
if daF
    mdA__x = matlabFunction(kin.mdA_x{i},'Vars',symvarsij);
    mdA__y = matlabFunction(kin.mdA_y{i},'Vars',symvarsij);
    mdA__theta = matlabFunction(kin.mdA_theta{i},'Vars',symvarsij);
    mdA__x_sweep = mdA__x(aa, ll, ai, aj); plot_kin.mdA__x_sweep = conditiondatasweep(mdA__x_sweep,[dnum,dnum]);
    mdA__y_sweep = mdA__y(aa, ll, ai, aj); plot_kin.mdA__y_sweep = conditiondatasweep(mdA__y_sweep,[dnum,dnum]);
    mdA__theta_sweep = mdA__theta(aa, ll, ai, aj); plot_kin.mdA__theta_sweep = conditiondatasweep(mdA__theta_sweep,[dnum,dnum]); % sweeps
    [da_xy_lim,da_theta_lim] = se2limits(mdA__x_sweep,mdA__y_sweep,mdA__theta_sweep); % limits
end
if lbaF
    lb_A__x = matlabFunction(kin.lb_A_x{i},'Vars',symvarsij);
    lb_A__y = matlabFunction(kin.lb_A_y{i},'Vars',symvarsij);
    lb_A__theta = matlabFunction(kin.lb_A_theta{i},'Vars',symvarsij);
    lb_A__x_sweep = lb_A__x(aa, ll, ai, aj); plot_kin.lb_A__x_sweep = conditiondatasweep(lb_A__x_sweep,[dnum,dnum]);
    lb_A__y_sweep = lb_A__y(aa, ll, ai, aj); plot_kin.lb_A__y_sweep = conditiondatasweep(lb_A__y_sweep,[dnum,dnum]);
    lb_A__theta_sweep = lb_A__theta(aa, ll, ai, aj); plot_kin.lb_A__theta_sweep = conditiondatasweep(lb_A__theta_sweep,[dnum,dnum]);
    [lba_xy_lim,lba_theta_lim] = se2limits(lb_A__x_sweep,lb_A__y_sweep,lb_A__theta_sweep);
end
if gcurlaF
    D_mA__x = matlabFunction(kin.D_mA_x{i},'Vars',symvarsij);
    D_mA__y = matlabFunction(kin.D_mA_y{i},'Vars',symvarsij);
    D_mA__theta = matlabFunction(kin.D_mA_theta{i},'Vars',symvarsij);
    D_mA__x_sweep = D_mA__x(aa, ll, ai, aj); plot_kin.D_mA__x_sweep = conditiondatasweep(D_mA__x_sweep,[dnum,dnum]);
    D_mA__y_sweep = D_mA__y(aa, ll, ai, aj); plot_kin.D_mA__y_sweep = conditiondatasweep(D_mA__y_sweep,[dnum,dnum]);
    D_mA__theta_sweep = D_mA__theta(aa, ll, ai, aj); plot_kin.D_mA__theta_sweep = conditiondatasweep(D_mA__theta_sweep,[dnum,dnum]);
    [gcurla_xy_lim,gcurla_theta_lim] = se2limits(D_mA__x_sweep,D_mA__y_sweep,D_mA__theta_sweep);
end
C1_lim = [min([da_xy_lim lba_xy_lim gcurla_xy_lim]) max([da_xy_lim lba_xy_lim gcurla_xy_lim])]; % limits for each child layout
C3_lim = [min([da_theta_lim lba_theta_lim gcurla_theta_lim]) max([da_theta_lim lba_theta_lim gcurla_theta_lim])];
% discrete kinematics-- stratified panel
C2_lim = nan(1,2); C4_lim = nan(1,2);
if dzF
    dz_phi_x = kin.dz_phi_x{i};
    dz_phi_y = kin.dz_phi_y{i};
    dz_phi_theta = kin.dz_phi_theta{i};
    dz__x_sweep = dz_phi_x(aa, ll, ai, aj); plot_kin.dz__x_sweep = conditiondatasweep(dz__x_sweep,[dnum,dnum]);
    dz__y_sweep = dz_phi_y(aa, ll, ai, aj); plot_kin.dz__y_sweep = conditiondatasweep(dz__y_sweep,[dnum,dnum]);
    dz__theta_sweep = dz_phi_theta(aa, ll, ai, aj); plot_kin.dz__theta_sweep = conditiondatasweep(dz__theta_sweep,[dnum,dnum]);
    [C2_lim,C4_lim] = se2limits(dz__x_sweep,dz__y_sweep,dz__theta_sweep);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Tiled layout Properties %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Size of the parent tiledlayout
P = [];
P.grid(1) = 3; % this is fixed since we are looking at the SE(2) kinematics
P.grid(2) = numel(fields)+1;

% Connection layout-- the A matrix for the fixed feet connection
CA = []; % form a structure
CA.start = nan(1,2); CA.start(1) = 1;
CA.grid(1) = 3;
CA_pre = 1; % the first column is always present with the gait constraint
CA_end = CA_pre; % initialize
if aF
    CA_end = CA_end + 1;
end
if CA_end ~= CA_pre
    CA.grid(2) = CA_end - CA_pre;
    CA.start(2) = CA_pre + 1;
else
    CA.start(2) = nan;
end
CA.flag = ~isnan(CA.start(2));

% Child 1 layout-- continous SE(2) translation kinematics heatmap
% Starting row-- 1
C1 = []; C1.limits = C1_lim; % previously defined color limits
C1.start = nan(1,2); C1.start(1) = 1;
C1.grid(1) = 2; % x and y rows, hence fixed.
C1_pre = CA_end; % column before child 1 layout
C1_end = C1_pre; % initialize
if daF % check if -dA heatmap is needed
    C1_end = C1_end + 1;
end
if lbaF % check if the lie-bracket is needed
    C1_end = C1_end + 1;
end
if gcurlaF
    C1_end = C1_end + 1;
end
if C1_end > C1_pre
    C1.grid(2) = C1_end - C1_pre; % get the number of columns
    C1.start(2) = C1_pre + 1; % get the first position
else
    C1.start(2) = nan; % no tiles in child 1 layout
end
C1.flag = ~isnan(C1.start(2)); % assign a flag if tiles exist

% Child 2 layout-- discrete SE(2) translation kinematics heatmap
% Starting row-- 1
C2 = []; C2.limits = C2_lim;
C2.start = nan(1,2); C2.start(1) = 1;
C2.grid(1) = 2;
C2.grid(2) = 1; % size of this layout is fixed
C2_pre = C1_end;
C2_end = C2_pre;
if dzF
    C2_end = C2_end + 1;
end
if C2_end > C2_pre
    C2.start(2) = C2_pre + 1;
else
    C2.start(2) = nan;
end
C2.flag = ~isnan(C2.start(2));

% Child 3 layout-- continous SE(2) rotation kinematics heatmap
% Starting row-- 3
C3 = []; C3.limits = C3_lim;
C3.start = nan(1,2); C3.start(1) = 3;
C3.grid(1) = 1;
C3_pre = CA_end;
C3_end = C3_pre;
if daF
    C3_end = C3_end + 1;
end
if lbaF
    C3_end = C3_end + 1;
end
if gcurlaF
    C3_end = C3_end + 1;
end
if C3_end > C3_pre
    C3.grid(2) = C3_end - C3_pre;
    C3.start(2) = C3_pre + 1;
else
    C3.start(2) = nan;
end
C3.flag = ~isnan(C3.start(2));

% Child 4 layout-- discrete SE(2) rotation kinematics heatmap
% Starting row-- 3
C4 = []; C4.limits = C4_lim;
C4.start = nan(1,2); C4.start(1) = 3;
C4.grid(1) = 1;
C4.grid(2) = 1; % size of this layout is fixed
C4_pre = C1_end;
C4_end = C4_pre;
if dzF % if translation kinematics exist, rotation should as well
    C4_end = C4_end + 1;
end
if C4_end > C4_pre
    C4.start(2) = C4_pre + 1;
else
    C4.start(2) = nan;
end
C4.flag = ~isnan(C4.start(2));

% Compute the grid properties ---------------------------------------------
% -- starting tile and indices wrt to the parent layout

% Let's compute the tile locations manually for the first column in the
% parent grid; this is different from the other child layouts because the
% parent grid spans all the child layouts as well.
P.tileIdx = 1 + ((1:P.grid(1)) - 1)*P.grid(2);

vF = 1; % vector field identification flag (need this for conn computation)

if CA.flag
    [CA, swtxt] = ...
        tileprops(plot_info, P, CA, vF);
    % first call create the sweep text cell array, and subsequent calls
    % udpate this.
    CA.vecF = true; % is a vector field child layout
end
if C1.flag
    [C1, swtxt] = ...
        tileprops(plot_info, P, C1, ~vF, swtxt);
    % from second call onwards we update this structure
    C1.vecF = false; % heatmap child layout
end
if C2.flag
    [C2, swtxt] = ...
        tileprops(plot_info, P, C2, ~vF, swtxt);
    C2.vecF = false;
end
if C3.flag
    [C3, swtxt] = ...
        tileprops(plot_info, P, C3, ~vF, swtxt);
    C3.vecF = false;
end
if C4.flag
    [C4, swtxt] = ...
        tileprops(plot_info, P, C4, ~vF, swtxt);
    C4.vecF = false;
end

% Store the sweep text in the information structure -----------------------
plot_info.heat_sweep_txt = swtxt.hS_txt;
plot_info.vecF_txt = swtxt.vfS_txt;
plot_info.title_txt = swtxt.t_txt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Suppress all warnings:
warning('off','all');

% Set the default interpreter before plotting to latex
%%%% https://www.mathworks.com/matlabcentral/answers/346436-how-to-use-latex-interpreter-for-xticklabels
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
% -------------------------------------------------------------------------
f = figure('units','pixels','position',[0 0 plot_info.m plot_info.n],...
    'Color','w');
set(gcf,'Visible','on'); % pop-out the current figure

% Create the parent tiled layout chart ------------------------------------
P.LayoutObj = tiledlayout(P.grid(1),P.grid(2),'TileSpacing','tight',...
    'Padding','tight');

% Create the children tiled layout charts ---------------------------------
if CA.flag
    CA.Layout_Obj = tiledlayout(P.LayoutObj,CA.grid(1),CA.grid(2)); % tile layout child connection
    CA.Layout_Obj.Layout.Tile = CA.tile_start; CA.Layout_Obj.Layout.TileSpan = CA.grid;
end
if C1.flag
    C1.Layout_Obj = tiledlayout(P.LayoutObj,C1.grid(1),C1.grid(2)); % tile layout child 1
    C1.Layout_Obj.Layout.Tile = C1.tile_start; C1.Layout_Obj.Layout.TileSpan = C1.grid;
end
if C2.flag
    C2.Layout_Obj = tiledlayout(P.LayoutObj,C2.grid(1),C2.grid(2)); % tile layout child 2
    C2.Layout_Obj.Layout.Tile = C2.tile_start; C2.Layout_Obj.Layout.TileSpan = C2.grid;
end
if C3.flag
    C3.Layout_Obj = tiledlayout(P.LayoutObj,C3.grid(1),C3.grid(2)); % tile layout child 3
    C3.Layout_Obj.Layout.Tile = C3.tile_start; C3.Layout_Obj.Layout.TileSpan = C3.grid;
end
if C4.flag
    C4.Layout_Obj = tiledlayout(P.LayoutObj,C4.grid(1),C4.grid(2)); % tile layout child 4
    C4.Layout_Obj.Layout.Tile = C4.tile_start; C4.Layout_Obj.Layout.TileSpan = C4.grid;
end

% Parent plots
xx = plot_info.xx; yy = plot_info.yy; th = plot_info.th; % unpack some plot parameters
col = plot_info.col; col_q = plot_info.col_q; col_backg = plot_info.col_backg;
titleFS = plot_info.titleFS; tickFS = plot_info.tickFS; labelFS = plot_info.labelFS; 
lW_contour = plot_info.lW_contour; cbarFS = plot_info.cbarFS; iQ = plot_info.iQ;
lW_V = plot_info.lW_V; gc_col = plot_info.gc_col; cs_idx = kin_info.cs_idx;

axP = cell(1,prod(P.grid)); % create the parent layout axes

for k = 1:numel(P.tileIdx) % iterate
    axP{k} = nexttile(P.LayoutObj,P.tileIdx(k));

    switch k
        
        case 1 % robot configuration 'q'
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% find a better way to do this later. %%%%%%%%%%%%%%
            plot(axP{k}, eval(['pltkin.legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_x(1, 1, a' num2str(cs(1)) ', xx, yy, th)']),...
                eval(['pltkin.legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_y(1, 1, a' num2str(cs(1)) ', xx, yy, th)']), 'Color', col(cs_idx,:), 'LineWidth', lW);
            axis equal square; hold on; 
            set(axP{k}, 'xTick',[]); set(axP{k}, 'yTick',[]);
            plot(axP{k}, eval(['pltkin.legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_x(1, 1, a' num2str(cs(2)) ', xx, yy, th)']),...
                eval(['pltkin.legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_y(1, 1, a' num2str(cs(2)) ', xx, yy, th)']), 'Color', col(cs_idx,:), 'LineWidth', lW);
            plot(axP{k}, eval(['pltkin.legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_x(1, 1, 0, xx, yy, th)']),...
                eval(['pltkin.legbase' num2str(cs(1)) '_leg' num2str(cs(1)) '_y(1, 1, 0, xx, yy, th)']), 'LineStyle', '--', 'Color', col(cs_idx,:), 'LineWidth', lW_r);
            plot(axP{k}, eval(['pltkin.legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_x(1, 1, 0, xx, yy, th)']),...
                eval(['pltkin.legbase' num2str(cs(2)) '_leg' num2str(cs(2)) '_y(1, 1, 0, xx, yy, th)']), 'LineStyle', '--', 'Color', col(cs_idx,:), 'LineWidth', lW_r);
            scatter(axP{k}, eval(['pltkin.leg' num2str(cs(1)) '_x(1, 1, a' num2str(cs(1)) ', xx, yy, th)']),...
                eval(['pltkin.leg' num2str(cs(1)) '_y(1, 1, a' num2str(cs(1)) ', xx, yy, th)']), circS, col(cs_idx,:), 'filled');
            scatter(axP{k}, eval(['pltkin.leg' num2str(cs(2)) '_x(1, 1, a' num2str(cs(2)) ', xx, yy, th)']),...
                eval(['pltkin.leg' num2str(cs(2)) '_y(1, 1, a' num2str(cs(2)) ', xx, yy, th)']), circS, col(cs_idx,:), 'filled');
            quiver(axP{k}, sum([1,0].*eval(['pltkin.k_leg' num2str(cs(1)) '_leg' num2str(cs(2)) '_x(1,1,a' num2str(cs(1)) ',a' num2str(cs(2)) ',xx,yy,th)'])),...
                sum([1,0].*eval(['pltkin.k_leg' num2str(cs(1)) '_leg' num2str(cs(2)) '_y(1,1,a' num2str(cs(1)) ',a' num2str(cs(2)) ',xx,yy,th)'])),...
                eval(['pltkin.k_leg' num2str(cs(1)) '_leg' num2str(cs(2)) '_u(1,1,a' num2str(cs(1)) ',a' num2str(cs(2)) ',xx,yy,th)']),...
                eval(['pltkin.k_leg' num2str(cs(1)) '_leg' num2str(cs(2)) '_v(1,1,a' num2str(cs(1)) ',a' num2str(cs(2)) ',xx,yy,th)']),...
                'Color', col_q, 'LineWidth', lW_kq, 'LineStyle', ':', 'AutoScale', 'off', 'ShowArrowHead', 'off'); % ksq line
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            plot(axP{k}, pltkin.legbase1_leg1_x(1, 1, a1, xx, yy, th), pltkin.legbase1_leg1_y(1, 1, a1, xx, yy, th), 'Color', col(1,:), 'LineWidth', lW);
            plot(axP{k}, pltkin.legbase2_leg2_x(1, 1, a2, xx, yy, th), pltkin.legbase2_leg2_y(1, 1, a2, xx, yy, th), 'Color', col(1,:), 'LineWidth', lW);
            plot(axP{k}, pltkin.legbase3_leg3_x(1, 1, a3, xx, yy, th), pltkin.legbase3_leg3_y(1, 1, a3, xx, yy, th), 'Color', col(7,:), 'LineWidth', lW);
            plot(axP{k}, pltkin.legbase4_leg4_x(1, 1, a4, xx, yy, th), pltkin.legbase4_leg4_y(1, 1, a4, xx, yy, th), 'Color', col(7,:), 'LineWidth', lW);
            plot(axP{k}, pltkin.topright2topleft_x(1, xx, yy, th), pltkin.topright2topleft_y(1, xx, yy, th), 'Color', col(7,:), 'LineWidth', lW_b);
            plot(axP{k}, pltkin.topleft2botleft_x(1, xx, yy, th), pltkin.topleft2botleft_y(1, xx, yy, th), 'Color', col(7,:), 'LineWidth', lW_b);
            plot(axP{k}, pltkin.botleft2botright_x(1, xx, yy, th), pltkin.botleft2botright_y(1, xx, yy, th), 'Color', col(7,:), 'LineWidth', lW_b);
            plot(axP{k}, pltkin.botright2topright_x(1, xx, yy, th), pltkin.botright2topright_y(1, xx, yy, th), 'Color', col(7,:), 'LineWidth', lW_b);
            quiver(axP{k}, pltkin.body_x(xx, yy, th), pltkin.body_y(xx, yy, th), frame_scale*sum([1,0]*pltkin.bodyf_x(xx, yy, th)),...
                frame_scale*sum([0,1]*pltkin.bodyf_x(xx, yy, th)),...
                'LineWidth', lW_qf, 'Color', col(1,:), 'AutoScale', 'off', 'ShowArrowHead', 'off');
            quiver(axP{k}, pltkin.body_x(xx, yy, th), pltkin.body_y(xx, yy, th), frame_scale*sum([1,0]*pltkin.bodyf_y(xx, yy, th)), frame_scale*sum([0,1]*pltkin.bodyf_y(xx, yy, th)),...
                'LineWidth', lW_qf, 'Color', col(1,:), 'AutoScale', 'off', 'ShowArrowHead', 'off');
            quiver(axP{k}, 0, 0, frame_scale*1, 0, 'LineWidth', lW_qf, 'Color', 'k', 'AutoScale', 'off', 'ShowArrowHead', 'off');
            quiver(axP{k}, 0, 0, 0, frame_scale*1, 'LineWidth', lW_qf, 'Color', 'k', 'AutoScale', 'off', 'ShowArrowHead', 'off');
            axis(axP{k}, [limX, limY]); axP{k}.XColor = col_q; axP{k}.YColor = col_q; axP{k}.LineWidth = boxW;
            set(axP{k},'Color',col_backg); title(plot_info.q_title_txt,FontSize=titleFS); 

        case 2 % k^2 level sets
            contour(axP{k}, ai,aj,ksq_sweep,cLvl,'LineWidth',lW_contour);
            axis equal tight; hold on; view(2);
            scatter(axP{k},eval(['a' num2str(cs(1))]),eval(['a' num2str(cs(2))]),circS_q,col_q,'filled','MarkerEdgeColor','k','LineWidth',lW_m,'Marker','square'); % config
            colormap(axP{k}, plot_info.jetDark); colorbar(axP{k}, 'TickLabelInterpreter','latex','FontSize',cbarFS); %%%%%%%%%%%%%%%%%%%%%%%%%%%
            set(get(axP{k}, 'YLabel'),'rotation',0,'VerticalAlignment','middle');
            title(axP{k}, plot_info.ksq_title_text,FontSize=titleFS);
            xticks(axP{k}, plot_info.xtickval); yticks(axP{k}, plot_info.ytickval);
            xticklabels(axP{k}, plot_info.xticklab);
            yticklabels(axP{k}, plot_info.xticklab);
            xlabel(axP{k}, plot_info.x_label_txt,FontSize=labelFS); ylabel(axP{k}, plot_info.y_label_txt,FontSize=labelFS);
            axP{k}.XAxis.FontSize = tickFS; axP{k}.YAxis.FontSize = tickFS; set(axP{k}, 'Color',col_backg);

        case 3 % iso-k^2 gait constraint vector field (+ve) 
            quiver(axP{k}, ai(iQ,iQ),aj(iQ,iQ),dphi_x_sweep(iQ,iQ),dphi_y_sweep(iQ,iQ),0.5,...
                'LineWidth',lW_V,'Color',gc_col);
            axis equal tight; hold on; view(2);
            scatter(axP{k},eval(['a' num2str(cs(1))]),eval(['a' num2str(cs(2))]),circS_q,col_q,'filled','MarkerEdgeColor','k','LineWidth',lW_m,'Marker','square'); % config
            set(get(axP{k},'YLabel'),'rotation',0,'VerticalAlignment','middle');
            title(axP{k}, plot_info.dphi_title_text,'Color',gc_col,FontSize=titleFS);
            xticks(axP{k}, plot_info.xtickval); yticks(axP{k}, plot_info.ytickval);
            xticklabels(axP{k}, plot_info.xticklab);
            yticklabels(axP{k}, plot_info.xticklab);
            axP{k}.XAxis.FontSize = tickFS; axP{k}.YAxis.FontSize = tickFS; set(axP{k}, 'Color',col_backg);

    end

end
plot_info.axP = axP; % store the parent axes

% Call the children layout plot function
plot_info.axC = plotchildlayout({CA,C1,C2,C3,C4},plot_kin,plot_info); % arrange the child layout structures in a cell array

% Sumanifold kinematics title
title(P.LayoutObj,plot_info.sgtitle_txt,'Color',plot_info.col(1,:),'Interpreter','latex','FontSize',plot_info.sgtitleFS);