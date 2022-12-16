% This script computes the sweep data based on a given function and a set 
% of inputs sweep varibales and ensures that they are of the same size 
% after computation for the case 1, lvl 2 kinematics. This is needed since 
% the functions are constructed from symbolic expressions that can take a 
% single value some times irrespective of the input. Hence, we obtain the
% scalar value and then scale it to the appropriate size
function sw = conditiondatasweep(sw,s)

if sum(size(sw) == s) < numel(s)
    % if data sweep is not scalar, and not the size of the input
    if numel(sw) ~= 1
        error(['ERROR: the sweep data provided must be the same size as the ' ...
            'input grid or needs to be a scalar value.']);
    end
    % if the data sweep is scalar
    sw = sw*ones(s);
end

end