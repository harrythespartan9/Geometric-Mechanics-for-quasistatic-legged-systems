% This functions takes the symbolic kinematics of a no-slip, quadrupedal robot, and constraints the level-2 shape-space slice to avoid ill-conditioned regions 
% near the contracting singularity to aid motion planning. If you add a very small perturbation to the shape-space near this singularity, the resulting 
% displacement in the position space is very large (unstable region), and hence undesirable for motion planning.
function [idxV, ksq_lb] = avoidcontractsingularity(kin_info, ksq, ksq_sweep, aa, ll, lb)

% initialize vars
C = [eye(2,2),zeros(2,1)];
syms l real

% get the contact states and transforms needed
csi = kin_info.lvl2_cs(kin_info.cs_idx, 1); csj = kin_info.lvl2_cs(kin_info.cs_idx, 2);
h_b__ib = kin_info.(['h_b__' num2str(csi) 'b']);
h_b__ib(3) = kin_info.(['h_' num2str(csi) num2str(csj) 'b__' num2str(csi) 'b'])(3);
h_b__jb = kin_info.(['h_b__' num2str(csj) 'b']);
h_b__jb(3) = kin_info.(['h_' num2str(csi) num2str(csj) 'b__' num2str(csj) 'b'])(3);

% Compute the contracting singularity point for Sij
thetai = h_b__ib(3); thetaj = h_b__jb(3);
ht__j2i = double(subs(C*(h_b__ib - h_b__jb), l, ll));
c__alpha_i = -thetai + atan2(ht__j2i(2),ht__j2i(1)) + pi; 
c__alpha_j = -thetaj + atan2(ht__j2i(2),ht__j2i(1));
ksq_lb = double(ksq(aa, ll, c__alpha_i, c__alpha_j)) + lb;
idxV = (ksq_sweep <= ksq_lb);


end