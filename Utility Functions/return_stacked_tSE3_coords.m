% This function returns the location of the translational coordinates from the provided SE3 transforms.
function out = return_stacked_tSE3_coords(in)

    % Check if everything is in standard SE3 transformation coords and do the computation
    if iscell(in)
        if numel(in) == 1
            if sum(size(in) == [4, 4]) ~= 2 % check
                error('ERROR! The input SE(3) transform is not a 4x4 matrix.')
            else
                out_x = in(1, 4); out_y = in(2, 4); out_z = in(3, 4); % compute
            end
        else
            out_x = nan(numel(in), 1); out_y = out_x; out_z = out_y;
            for i = 1:numel(in)
                if sum(size(in{i}) == [4, 4]) ~= 2 % iteratively check
                    error(['ERROR! The ', num2str(i), 'th transform is not a 4x4 matrix.']);
                end
                out_x(i) = in{i}(1, 4); out_y(i) = in{i}(2, 4); out_z(i) = in{i}(3, 4);
            end
        end
    else
        if sum(size(in) == [4, 4]) ~= 2
            error('ERROR! The input SE(3) transform is not a 4x4 matrix.')
        else
            out_x = in(1, 4); out_y = in(2, 4); out_z = in(3, 4);
        end
    end

    % Pack everything
    out = [out_x(:),  out_y(:), out_z(:)];

end