% This function 
function ax = generateSE3frames(H, l)

    % some quantities to help
    row123_select = [eye(3,3), zeros(3,1)];  % just first three rows
    col4_select = [zeros(3,1); 1];
    
    % initialize the output axes
    ax = cell(1, 3);

    % generate frames
    ax{1} = row123_select*(H*t_SE3([l, 0, 0]) - H)*col4_select;
    ax{2} = row123_select*(H*t_SE3([0, l, 0]) - H)*col4_select;
    ax{3} = row123_select*(H*t_SE3([0, 0, l]) - H)*col4_select;

end