% This function computes the SE3 transforms of a series of inputs provided in the given order from left to right.
function out = chaincomputeSE3transforms(in)




    % Check if the input is well-conditioned SE(3) transformation matrices
    if iscell(in)
        if numel(in) <= 1
            error('ERROR! There has to be more than one term in the input.');
        else
            for i = 2:numel(in)
                if i > 1
                    n_minus = numel(in{i-1}); n = numel(in{i});
                    if n ~= n_minus % Check
                        error('ERROR! The inputs have unequal number of elements.');
                    end
                end
                for j = 1:numel(in{i})
                    if sum(size(in{i}{j}) == [4, 4]) ~= 2
                        error('ERROR! Every single SE(3) transformation matrix provided should be a 4x4 matrix.');
                    end
                end
            end
            % if we are here, everything checks out-- perform the computation after initializing the output
            out = cell(numel(in{1}), 1);
            for j = 1:numel(out)
                tempH = eye(4, 4);
                for i = 1:numel(in)
                    tempH = tempH*in{i}{j};
                end
                out{j} = tempH;
            end
        end
    else
        error('ERROR! If you want to find the SE(3) transform of n-inputs, each input needs to be wrapped in an array.');
    end




end