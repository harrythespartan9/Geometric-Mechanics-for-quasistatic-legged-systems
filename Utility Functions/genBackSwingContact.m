function c_out = genBackSwingContact(r_dot)
%GENBACKSWINGCONTACT compute the contact state trajectory given swing trajectory in HAMR VI motion capture format
%   Given swing velocity time-series, the contact trajectory is 1 when swing vel < 0 and 0 when otherwise.
    switch iscell(r_dot)
        case 1
            c_out = cell(size(r_dot));
            for i = 1:numel(r_dot)
                c_out{i} = zeros(size(r_dot{i})); c_out{i}(r_dot{i} < 0) = 1;
            end
        case 0
            c_out = zeros(size(r_dot)); c_out(r_dot < 0) = 1;
    end
end