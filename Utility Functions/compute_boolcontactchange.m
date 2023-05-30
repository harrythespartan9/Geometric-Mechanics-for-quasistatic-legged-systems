function c_dot = compute_boolcontactchange(c)
%COMPUTE_BOOLCONTACTCHANGE compute a boolean contact change vector given a cell array of contact changes for each leg.
%   'c' cell array contains the contact state of each leg as a function of time. This function uses diff to obtain in change in c{i} for each i'th leg and then
%   concatenates them together to create a boolean, contact change array.
    if nargin < 1
        error('ERROR! Please provide an input to the function.');
    end
    if iscell(c)
        temp = [];
        for i = 1:numel(c)
            temp = [temp; abs(diff(c{i}))];
        end
        c_dot = (   abs(  diff( sum(temp, 1) )  )  > 0   );
    else
        if isempty(c)
            error('ERROR! The input array shouldnt be empty.');
        elseif ~isnumeric(c)
            error('ERROR! The input array should be numeric.')
        end
        c_dot = (  abs( diff(c) ) > 0  );
    end

end

