% this script computes the title for the plotted/animated path based on the shape-space slice it belongs to, initial condition of the slice under consideration,
% and integration time.
function pathtitle(ax, cs, u, idx, ic, t_int, titleFS)

% number of paths
numP = size(cs, 1);

% initialize
psi_text = '';

% create text based on the number of paths involved
for i = 1:numP
    % if this is not the first path, add a plus sign
    if i == 1
        temp = '$$ ';
    else
        temp = ' + ';
    end
    % condition ic if it is a very small number
    if abs(ic(i,1)) < 1e-2
        ic(i,1) = 0;
    end
    if abs(ic(i,2)) < 1e-2
        ic(i,2) = 0;
    end
    % Compute the current character
%     if u(i, idx(i)) == 1 % if the input is equal to 1                                                     %%%%%%%%%%%%%%%%%%% COMMENTED ON 20230404 (YYYYMMDD)
%         psi_text = [psi_text, temp, '\psi^{' num2str(cs(i, 1)) num2str(cs(i, 2)) '}_{' num2str(t_int(i), 3)...
%             '} \left(' num2str(ic(i,1), 3) ', ' num2str(ic(i,2), 3) '\right)'];
%     else
%         psi_text = [psi_text, temp, '\left(' num2str(u(i, idx(i)), 3) '\right) \psi^{' num2str(cs(i, 1)) num2str(cs(i, 2)) '}_{' num2str(t_int(i), 3)...
%             '} \left(' num2str(ic(i,1), 3) ', ' num2str(ic(i,2), 3) '\right)'];
%     end
    psi_text = [psi_text, temp, '\hat{\phi}_{' num2str(cs(i, 1)) num2str(cs(i, 2)) '}' ' \left( ' num2str(u(i, idx(i)), 3) ', ' num2str(-t_int(i, 1), 3)...
        ', ' num2str(t_int(i, 2), 3) ', \left( ' num2str(ic(i, 1), 3) ', ' num2str(ic(i, 2), 3) ' \right) \right)'];
end
% finish this text off with equation ending decorators
psi_text = [psi_text, ' $$'];

% add the title in the axes provided
title(ax, psi_text, 'Color', 'k', 'Interpreter', 'latex', FontSize=titleFS);

end