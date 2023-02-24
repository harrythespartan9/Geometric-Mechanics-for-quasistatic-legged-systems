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










