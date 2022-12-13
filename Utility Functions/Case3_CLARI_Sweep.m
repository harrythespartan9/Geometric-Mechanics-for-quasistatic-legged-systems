% This script manually sweeps the shape space to explore the structure of
% the pfaffian constraint matrix created for the top-half of CLARI system
clear all; close all; clc;

% Load the symbolic constraint matrix and the necessary symbols
load("pfaffian_constraint_CLARI.mat");
% Get the size of the constraint matrix:
rowP = size(pfaff_CM,1); colP = size(pfaff_CM,2);

% Convert the constraint matrix into a function from a symbol:
pfaff_CM = matlabFunction(pfaff_CM,'Vars',[a, l, gamma, alpha_1, alpha_2]);


% Clear the symbols:
clear a l gamma alpha_1 alpha_2;

% Uniform discretization of the shape-space
dnum = 100; %10 %25 %50 %75 %100 % some example values (use lowest value 
                                 % for testing

% Define the range for each shape mode and other system parameters:
a = 1;                                 % leg to half link length ratio
l = 1;                                 % half link length

ank = pi/4;                            % symmetric leg ankle limit
sank = [0,pi/2];                       % sprawl limits (ordered)

a1 = linspace(-ank,ank,dnum);          % right leg angle
a2 = a1;                               % left leg angle
g = linspace(sank(1),sank(2),dnum);    % sprawl angle

[a1,a2,g] = meshgrid(a1,a2,g);         % create a 3D grid of values in each 
                                       % shape

% Create a pfaffian constraint matrix at each data point, the value of the 
% determinant of omega_gbg (matrix formed by the first 4 cols, and this
% needs to be inverted to find a local connection), and the inverse of the
% omega matrix under consideration

% Initialize containers:
pfaff_CM_sweep = nan(dnum,dnum,dnum,rowP,colP);
det_omega_gbg_sweep = nan(dnum,dnum,dnum);
omega_gbg_inv_sweep = nan(dnum,dnum,dnum,4,4);

% Sweep and store the results:
for i = 1:dnum
    for j = 1:dnum
        for k = 1:dnum
            
            % Compute the constraint at this point:
            pfaff_CM_sweep(i,j,k,:,:) = pfaff_CM(a, l, g(i,j,k), a1(i,j,k),...
                a2(i,j,k));

            % Compute the determinant:
            det_omega_gbg_sweep(i,j,k) = det(reshape(pfaff_CM_sweep(i,j,k,:,1:4),[4,4]));

            % Compute the inverse of the omega matrix:
            if det_omega_gbg_sweep(i,j,k) ~= 0
                omega_gbg_inv_sweep(i,j,k,:,:) = inv(reshape(pfaff_CM_sweep(i,j,k,:,1:4),[4,4]));
            end

        end
    end
end

% Save the daya:
save('pfaffian_constraint_CLARI_sweep.mat','det_omega_gbg_sweep','pfaff_CM_sweep','omega_gbg_inv_sweep','a1','a2','g');
