function [S, L] = interpswingliftSE3(He, H, idx, j, r)
    
    % initialize symbols and output containers
    syms alpha_1 beta_1 alpha_2 beta_2 alpha_3 beta_3 alpha_4 beta_4 real
    S = cell(1, 10);
    L = cell(1, 10);

    % the shape elements we are interested in for the current leg
    ri = r(2*j-1:2*j);

    % Substitute all the other values to obtain the transform
    oldvars = []; newvars = []; vars = [];
    for i = 1:4
        if i ~= j
            oldvars = [oldvars, eval(['alpha_' num2str(i)]), eval(['beta_' num2str(i)])]; % shape elements not a function of the interpolation leg
            newvars = [newvars, r{2*i-1}(idx), r{2*i}(idx)];
        else
            vars = [vars, eval(['alpha_' num2str(i)]), eval(['beta_' num2str(i)])]; % shape elements of the interpolation leg
            % break;
        end
    end
    H = subs(H, oldvars, newvars);
    fH = matlabFunction(H, 'Vars', vars);
    
    % Swing and lift values of interest
    ri_alpha = linspace(0, ri{1}(idx), 10);
    ri_beta = linspace(0, ri{2}(idx), 10);

    % Compute the foot tip at the specific swing and lift values
    for i = 1:10
        S{i} = He*fH(ri_alpha(i), 0); % calculate the swing first
        L{i} = He*fH(ri_alpha(10), ri_beta(i)); % calculate the swing from the end of the lift
    end

end