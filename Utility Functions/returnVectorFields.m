function [dq, dphi] =  returnVectorFields(kinfunc, i)
    
    % define the level-2 contact states
    cslvl2 = [1, 2;
              2, 3;
              3, 4;
              4, 1;
              1, 3;
              2, 4]; % all lvl-2 contact states

    % obtain the contact state needed
    cs = cslvl2(i,:);

    % obtain the symbolic functions and vars
    dphi = kinfunc.dphi;
    dz = kinfunc.dz_phi;
    eval(['syms t a l ' 'alpha_' num2str(cs(1)) ' alpha_' num2str(cs(2)) ' real']);
    
    % define the variables to generate a numeric function
    symvarsijwt = eval(['[t, a, l, ' 'alpha_' num2str(cs(1)) ', ' 'alpha_' num2str(cs(2)) ']']);

    % convert the symbolic functions to numeric functions
    dq = [dz; dphi];
    dq = matlabFunction(dq, 'Vars', symvarsijwt);
    dphi = matlabFunction(dphi, 'Vars', symvarsijwt);

end