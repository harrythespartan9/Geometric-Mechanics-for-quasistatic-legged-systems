% This function 
function ax = generateSE3frames(H, l)

    % some quantities to help
    row_select = [eye(3,3), zeros(3,1)]; translation_select = [zeros(3,1); 1];
    
    % initialize the output axes
    ax = cell(1, 3);

    % generate frames
    ax{1} = row_select*(H*t_SE3([l, 0, 0]) - H)*translation_select;
    ax{2} = row_select*(H*t_SE3([0, l, 0]) - H)*translation_select;
    ax{3} = row_select*(H*t_SE3([0, 0, l]) - H)*translation_select;

end