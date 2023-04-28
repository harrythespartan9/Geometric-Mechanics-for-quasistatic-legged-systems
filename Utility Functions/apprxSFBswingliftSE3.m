% This function computes the approximation for the swing and lift angles when provided with the body and leg trajectories in a world rest frame.
function out = apprxSFBswingliftSE3(b, ht3_e__i_exp, kin)
    
    % check if the inputs are in the correct format
    if iscell(b) && iscell(ht3_e__i_exp)
        if numel(b) == 6 && numel(ht3_e__i_exp) == 4
            verifylength(b); verifylength(ht3_e__i_exp);
            if numel(b{1}) ~= size(ht3_e__i_exp{1}, 2)
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
    
    symb = kin.transforms.sym;                                                            % unpack everything you need from HAMR6's kinematic struct
    func = kin.transforms.fun;
    Rsymb= kin.config.R; assume(Rsymb, 'real');                                           % an important assumption about our inputs

    obj_fun = @fxninx;                                                                    % declare the static path to the objective function outside the loop

    H3_b__i = symb.H3_b__i;                                                               % analytical transforms to each leg from the body frame

    r_bound = repmat(deg2rad(89)*[-1, 1], [8, 1]);                                        % symmetric lift and swing bounds for fmincon optimization-- 89 degs

    out = cell(1, t);                                                                     % approximated lift and swing angles -- as a cell array

    for i = 1:t

        sq_err = sym('0');                                                                % squared error norm of the approximate vs actual foot location
                                                                                                               % added over all feet for a given t--
                                                                                                               % reinitialize this every time!!!
        
        H3_e__b = func.fH3_e__b( b{1}(i), b{2}(i), b{3}(i), b{4}(i), b{5}(i), b{6}(i) );  % current transform to body coordinates

        for j = 1:4
        
            err = ht3_e__i_exp{j}(:, i) - p_SE3(H3_e__b*H3_b__i{j});
            sq_err = sq_err +  err'*err;                                                  % for each foot, obtain the squared error between the observation 
                                                                                          % and analytical calculation-- this will be summed between every leg
        end

        fsq_err = matlabFunction(simplify(sq_err), 'Vars', Rsymb);                        % create a function with all the errors

        out{i} = fmincon(@(x) obj_fun(x, fsq_err), zeros(8, 1),...                        % obtain the shape elements that minimize the objective function
                [], [], [], [], r_bound(:, 1), r_bound(:, 2), [], optimoptions('fmincon', 'Display', 'off', 'UseParallel', true));

    end

end