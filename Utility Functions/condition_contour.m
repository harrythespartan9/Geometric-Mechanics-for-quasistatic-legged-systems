function tempp = condition_contour(temp)
%CONDITION_CONTOUR conditions an input contour matrix
%   Sometimes the contour matrices generated for a single level splits it up over sub-arrays-- the goal is to concatenate into a single array in this step.

    % Initialize the return array, a counter, and a point count container
    tempp = []; count = size(temp, 2); N = 0;
    
    % update the number of entries and the z-value
    z = temp(1,1);

    % Start extracting the chunks
    while count > 0
        % get the count
        N = N + temp(2,1);
        % update the return array
        tempp = [tempp, temp( :, 2:temp(2,1)+1 )];
        % updated the counter and input array
        temp = temp(:, temp(2,1)+2:end);
        count = size(temp, 2);
    end

    % add the z-value and total contour points N as the first column: [z; N]
    tempp = [ [z; N], tempp ];

end

