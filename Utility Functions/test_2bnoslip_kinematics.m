% this script tests the motion planning code with sanity check plots-- feel free to add more stuff to this.
%% WITHOUT RUNNING CODE

xi = S12_data_y{5}.open_trajectory{10}{2}; yi = S12_data_y{5}.open_trajectory{10}{3}; thetai = S12_data_y{5}.open_trajectory{10}{4};
xj = S34_data_y{5}.open_trajectory{10}{2}; yj = S34_data_y{5}.open_trajectory{10}{3}; thetaj = S34_data_y{5}.open_trajectory{10}{4};
pi = [xi; yi; thetai];

figure()
plot(xi, yi, 'r');
hold on; grid on; axis equal square;
plot(xj, yj, 'b--');

pj_pullback = adginv(thetai(end))*[xj; yj; thetaj]; % do the pullback

figure()
plot(xi, yi, 'r');
hold on; grid on; axis equal square;
plot(pj_pullback(1,:), pj_pullback(2,:), 'b--');

pj = pi(:,end) + pj_pullback; % offset it by the end of path 1

figure()
plot(xi, yi, 'r');
hold on; grid on; axis equal square;
plot(pj(1,:), pj(2,:), 'b--');


%% WHILE RUNNING CASE 3 MOTION PLANNING CODE

figure()
plot(a1_i, a2_i, 'r');
hold on; grid on; axis equal square;
plot(a1_j, a2_j, 'b--');

figure()
plot(xi, yi, 'r');
hold on; grid on; axis equal square;
plot(xj, yj, 'b--');

figure()
plot(thetai, 'r');
hold on; grid on;
plot(thetaj, 'b--');

figure()
plot(phi_i(1,:), phi_i(2,:), 'r');
grid on; hold on; axis equal square;
plot(phi_j(1,:), phi_j(2,:), 'b--');

figure()
plot(phi_i(3,:), 'r');
grid on; hold on;
plot(phi_j(3,:), 'b--');

%% 

figure()
plot(b_phi(1,:), b_phi(2,:))
%% 
figure()
plot(idxiA, 'r');
hold on;
plot(idxiC, 'b');

figure()
plot(idxjA, 'r');
hold on;
plot(idxjC, 'b');
%% 
figure()
plot(a1_i(idxiA), a2_i(idxiA), 'r--');
hold on; grid on; axis equal square;
plot(a1_j(idxjA), a2_j(idxjA), 'b--');
plot(a1_i(idxiC), a2_i(idxiC), 'r:');
% plot(a1_j(idxjC), a2_j(idxjC), 'b:');

figure()
plot(a1_j(idxjC), a2_j(idxjC), 'b--');

%% 
figure()
plot(idxjA, 'r');
hold on;
plot(idxjC, 'b');
plot(a1_j/max(a1_j), 'k');
plot(a2_j/max(a2_j), 'k--');
%% 
figure()
plot(x, y)
%% 
figure()
plot(a_n(3,:))
%% LEG1
% we are in the first to third submanifold-- hence leg 1 and leg 2 active are i, and leg 3 and leg 4 active are j
figure()
plot(pltkin_i.(['legbase' num2str(cs_i(1)) '_leg' num2str(cs_i(1)) '_x'])(a_i, l_i, a1_i, x, y, theta), 'r')
hold on;
plot(pltkin_i.legbase1_leg1_x(a_i, l_i, a_n(1, :), x, y, theta), 'b--')
%% LEG2
figure()
plot(pltkin_i.(['legbase' num2str(cs_i(2)) '_leg' num2str(cs_i(2)) '_x'])(a_i, l_i, a2_i, x, y, theta), 'r')
hold on;
plot(pltkin_i.legbase2_leg2_x(a_i, l_i, a_n(2, :), x, y, theta), 'b--')
%% LEG3
figure()
plot(pltkin_j.(['legbase' num2str(cs_j(1)) '_leg' num2str(cs_j(1)) '_x'])(a_j, l_j, a1_j, x, y, theta), 'r')
hold on;
plot(pltkin_j.legbase3_leg3_x(a_j, l_j, a_n(3, :), x, y, theta), 'b--')
%% LEG3
figure()
plot(pltkin_j.(['legbase' num2str(cs_j(2)) '_leg' num2str(cs_j(2)) '_x'])(a_j, l_j, a2_j, x, y, theta), 'r')
hold on;
plot(pltkin_j.legbase4_leg4_x(a_j, l_j, a_n(4, :), x, y, theta), 'b--')
%%
% SE2 trajectories
figure()
yyaxis left
plot(t,x,'k');
hold on; grid on;
plot(t,y,'k');
yyaxis right
plot(t,theta,'r'); grid on;
figure()
plot(x,y,'k')
%% 
figure()
plot(t,theta)
%% 
figure()
% plot(x, y, 'r--');
% hold on; axis equal;
plot(b(1,:), b(2,:), 'r');
%% 
figure()
plot(b(1,:))
%% 
figure()
plot(b(2,:))
%% 
figure()
plot(b(3,:))
%% 
figure()
plot(path12.open_trajectory{20}{2}, path12.open_trajectory{20}{3}, 'r');
hold on; grid on;
plot(path12.open_trajectory{1}{2}, path12.open_trajectory{1}{3}, 'b');
%% 
figure()
plot(path_1{2}, path_1{3}, 'r', 'LineWidth', 1.2);
hold on; grid on; axis equal square;
plot(path_2{2}, path_2{3}, 'r--', 'LineWidth', 1.2);
axis([-1 1 -1 1]);

figure()
plot(path_1{1}, path_1{4}, 'r', 'LineWidth', 1.2);
hold on; grid on;
plot(path_2{1}, path_2{4}, 'r--', 'LineWidth', 1.2);

figure()
plot(path_1{5}, path_1{6}, 'r', 'LineWidth', 1.2);
hold on; grid on; axis equal square;
plot(path_2{5}, path_2{6}, 'r--', 'LineWidth', 1.2);
axis(pi/2*[-1 1 -1 1]);
%% 







