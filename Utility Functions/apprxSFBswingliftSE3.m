% This function computes the approximation for the swing and lift angles when provided with the body and leg trajectories in a world rest frame.
function out = apprxSFBswingliftSE3(b, ht3_e__i_exp, kin)
    
    % check if the inputs are in the correct format
    if iscell(b) && iscell(ht3_e__i_exp)
        if numel(b) == 6 && numel(ht3_e__i_exp) == 4
            verifylength(b); verifylength(ht3_e__i_exp);
            if numel(b{1}) ~= size(ht3_e__i_exp{1}, 1)
                error('ERROR! The body and shape trajectories should have the same number of discretizations.');
            else
                t = numel(b{1});
            end
        else
            error('ERROR! The number of cells in b and h should be 6 and 4 respectively.');
        end
    else
        error('ERROR! The body and shape trajectories need to be cell arrays.');
    end
    
    syms alpha beta 
    H3_b__i = kin.H3_b__i;                                                               % analytical transforms to each leg
    r_bound = deg2rad(30)*[-1, 1];                                                       % symmetric lift and swing bounds for fmincon optimization
    out = cell(8, t);                                                                    % approximated lift and swing angles
    
    for i = 1:t
        
        H3_e__b = kin.fH3_e__b( b{1}(i), b{2}(i), b{3}(i), b{4}(i), b{5}(i), b{6}(i) );  % current transform to body coordinates
        
        sq_err_sum = sym('0');                                                           % 'zero' symbolic initialization of squared error container

        for j = 1:4

        err = ht3_e__i_exp{j}(:, i) - t_SE3(H3_e__b*H3_b__i{j});
        sq_err_sum = sq_err_sum + err'*err;                                              % for each foot, obtain the squared error between the observation 
                                                                                         % and analytical calculation-- this will be summed between every leg
        end

        ht3_e__i_sqerr = matlabFunction(sq_err_sum, 'Vars', kin.shape_space.r);          % convert this to a function

        out{:, i} = num2cell(fmincon(ht3_e__i_sqerr, zeros(8, 1), [], [], [], [],...
            r_bound, r_bound));                                                          % run fmincon to compute the swing and lift angles that minimize

    end

end