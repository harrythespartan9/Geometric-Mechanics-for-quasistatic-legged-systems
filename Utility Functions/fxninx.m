% This function acts as a processing function for running fmincon on leg trajectory data.
% Inputs: 1) x: this is the input vector from fmincon function
%         2) fun: this is the objective function we need to evaluate over
function out = fxninx(x, fun)

    if ~isa(fun, 'function_handle') % check for consistency in the second input
        error('ERROR! The second input has to be an objective function to evaluate over.');
    end

    n = numel(x);                                                            % number of input arguments to separate out

    for i = 1:n
    str_temp = ['in', num2str(i)];                                           % "str_temp": ith input string
    eval([str_temp, ' = x(', num2str(i), ');']);                             % separate out the inputs from x using "str_temp"
        switch i
            case 1
                in_str = str_temp;                                           % initialize an input string in the first iteration as "str_temp"
            otherwise
                in_str = [in_str, ', ', str_temp];                           % concatenate "str_temp" with a comma
        end
    end

    out = eval(['fun(', in_str, ')']);                                       % evaluate the function with input string generated

end