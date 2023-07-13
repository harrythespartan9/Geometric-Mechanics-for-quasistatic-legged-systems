%seqSE2transformation compute a sequence of SE2 transformations given in a left-to-right stacked input array
function cum_vector = seqSE2transformation(in_arr)
    %   Each column in the array corresponds to the group element operation from left to right. The columns are SE2 vectors, so they need to be converted into their
    %   corresponding matrix forms to act on each other.

    % Initialize SE(2) transform containers
    if isa(in_arr, 'sym')
        cum_angle = sym(0);
        cum_transform = sym(eye(3));
    else
        cum_angle = 0;
        cum_transform = eye(3);
    end

    % Iterate over each transform in the order of the input array arranged from left to right
    for i = 1:size(in_arr, 2)
        
        % updated the cumulative transform angle
        cum_angle = cum_angle + in_arr(3,i);

        % update the transform
        cum_transform = cum_transform *v2M_SE2( in_arr(:,i) );

    end
    
    % return the vector form of this cumulative transform
    cum_vector = [cum_transform(1,3), cum_transform(2,3), cum_angle]';
    
end

