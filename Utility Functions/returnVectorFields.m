function [dphi, dz] =  returnVectorFields(kinfunc, i)
    
    % Obtain the symbolic functions
    dphi = kinfunc.dphi;
    dz = kinfunc.dz_phi;

    % Define the level-2 contact states
    cslvl2 = [1, 2;
            2, 3;
            3, 4;
            4, 1;
            1, 3;
            2, 4]; % all lvl-2 contact states

    % Obtain the contact state needed
    cs = cslvl2(i,:);
    
    % Define the variables to generate a numeric function
    symvarsijwt = eval(['[t, a, l, ' 'alpha_' num2str(cs(1))...
        ', ' 'alpha_' num2str(cs(2)) ']']);

    % Convert the symbolic functions to numeric functions
    dphi = matlabFunction(dphi, 'Vars', symvarsijwt);
    dz = matlabFunction(dz, 'Vars', symvarsijwt);

end