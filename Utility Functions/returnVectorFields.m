function [dz, dpsi, ddpsi, A, Adot, ksq, symvarsijwt] =  returnVectorFields(kinfunc, i)
    
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
    dpsi = kinfunc.dpsi;
    ddpsi = kinfunc.ddpsi;
    A = kinfunc.A;
    Adot = kinfunc.Adot;
    dz = kinfunc.dz_psi;
    ksq = kinfunc.ksq;
    
    % define the variables to generate a numeric function
    symvarsijwt = cell(1,2);
    symvarsijwt{1} = ['syms t a l x y theta ' 'alpha_' num2str(cs(1)) ' alpha_' num2str(cs(2)) ' real;'];
    symvarsijwt{2} = ['[t, a, l, ' 'alpha_' num2str(cs(1)) ', ' 'alpha_' num2str(cs(2)) ']'];
    symvarsijwt{3} = ['[t, a, l, x, y, theta, ' 'alpha_' num2str(cs(1)) ', ' 'alpha_' num2str(cs(2)) ']'];

%     % convert the symbolic functions to numeric functions
%     dq = [dz; dpsi];

end