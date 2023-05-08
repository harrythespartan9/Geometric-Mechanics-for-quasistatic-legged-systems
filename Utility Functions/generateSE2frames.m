% This function generates SE(2) frames (x_hat and y_hat quivers compatible with MATLAB's proprietary QUIVER2 function)
function ax = generateSE2frames(H, l)

    % some quantities to help
    row12_select = [eye(2,2), zeros(2,1)];  % just first two rows
    col3_select = [zeros(2,1); 1]; % just 3rd col (translation vector)
    
    % initialize the output axes
    ax = cell(1, 3);

    % generate frames
    ax{1} = row12_select*(H*t_SE2([l, 0]) - H)*col3_select;
    ax{2} = row12_select*(H*t_SE2([0, l]) - H)*col3_select;

end