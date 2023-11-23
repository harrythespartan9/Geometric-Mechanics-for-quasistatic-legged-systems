function tempp = condition_contour2(temp, iter_old)
%CONDITION_CONTOUR conditions an input contour matrix
%   Sometimes the contour matrices generated for a single level splits it up over sub-arrays-- the goal is to number each contour result.
    
    % Initialize the return array, a counter, and a point count container (based on an input or fresh start)
    tempp = []; count = size(temp, 2); N = 0;
    switch nargin
        case 2
            iter = iter_old;
        case 1
            iter = 0;
        otherwise
            error('ERROR! This function can only have two inputs.');
    end
    
    % update the number of entries and the z-value
    z = temp(1,1);
    
    % Start the loop based on the number of columns in temp and decrement after conditioning
    while count > 0
        % increment the iteration
        iter = iter + 1;
        % get the count
        N = N + temp(2,1);
        % update the return array
        tempp = [ tempp, [iter*ones(1, temp(2,1)); temp( :, 2:temp(2,1)+1 )] ];
        % updated the counter and input array
        temp = temp(:, temp(2,1)+2:end);
        % update the new size
        count = size(temp, 2);
    end
    
    % add the z-value and total contour points N as the first column: [z; N]
    tempp = [ [nan; z; N], tempp ];
    
end

