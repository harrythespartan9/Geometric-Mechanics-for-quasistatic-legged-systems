% This function normalizes the 2D bounding box around a sprawled, quadrupedal robot based on the requirement. If this bounding box is meant to be a square, then 
% the function takes the norm of the vectors provided between the frames and then averages them. If this bounding box is not a square, then only opposing
% vectors are averaged out.
function out = boundboxmean(in, flag)

    % Check the flag-- if true, it is a square, else it is not.
    switch flag
    
        case true % square box
            
            % The scaling values for each vector
            scaleV = scalevec(in);

            % Scale the vectors
            out = round(in.*scaleV, 3, 'significant'); % round the output to 3 significant digits
    
        case false % non-square box

            % Split the input into two groups for opposite and adjacent sides
            in_top = in(1:2:end, :); in_bot = in(2:2:end, :);
            scaleV_top = scalevec(in_top); scaleV_bot = scalevec(in_bot);
            out_top = round(scaleV_top.*in_top, 3, 'significant'); out_bot = round(scaleV_bot.*in_bot, 3, 'significant'); 
            out = nan(size(in)); % initialize the output container since we are assigning values partially
            out(1:2:end, :) = out_top; out(2:2:end, :) = out_bot; % combine them into the output we need.
    
    end

end