% This function computes the adjoint and adjoint inverse transforms to map SE(3) twists between the frames involved. Takes an SE(3) transformation matrix as an
% input and if you are inputting multiple transformations, it should be a cell array.
function out = adj_adjinv_SE3(in, f)

    % Check if the input is a cell array ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if ~iscell(in) % if not a cell array ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        % If an output is received, this is the single transformation case.
        out = compute_adj_adjinv_SE3(in, f);

    else
        
        % Initialize the output container
        out = cell(1, numel(in));

        % Iterate over all the transforms (checking each) and compute the adjoint or adj-inv
        for i = 1:numel(in)
            
            out{i} = compute_adj_adjinv_SE3(in{i}, f);

        end

        % Condition to the output cell array to have the same shape as the input array
        out = reshape(out, size(in));

    end

end