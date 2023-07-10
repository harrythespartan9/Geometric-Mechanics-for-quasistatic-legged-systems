function h = SE2GroupDiff(g_i, g_j)
%GROUPDIFF computes the difference between two SE(2) group elements (between "g_j" and "g_i") series
%   For two cell arrays with the SE(2) group strcture, this function takes the group difference an defines the error with another group element.
%   Consider h(g_i) = g_j => h = g_j * (g_i)^-1 %%%%%%%%%%%%%%%%%%
%   Example:%%% Consider the group error between a nominal group trajectory and a identity trajectory

    % make the sure the series/sequences have the same length and then convert them into group operating type to find the difference
    switch numel(g_i) == numel(g_j)
        case 0
            error('ERROR! The sequences should have the same length.');
        case 1
            switch iscell(g_i) && iscell(g_j)
                case 0
                    error('ERROR! Please provide the sequences in cell arrays even if the length is one.');
                case 1
                    if numel(g_i) == 3
                        g_i = convertToSE2vec(g_i);
                    end
                    if numel(g_j) == 3
                        g_j = convertToSE2vec(g_j); % convert them to the correct vector format
                    end
                    [chk_i, g_i] = checkAndComputeSE2operator(g_i, 1); % pass second argument as 1 to obtain the form
                    if ~chk_i
                        error('ERROR! "g_i" is not an SE(2) compatible sequence.');
                    end
                    [chk_j, g_j] = checkAndComputeSE2operator(g_j, 1);
                    if ~chk_j
                        error('ERROR! "g_j" is not an SE(2) compatible sequence.');
                    end
                    h = cell(size(g_i)); % initialize
                    for i = 1:numel(g_i) % iterate and compute the difference
                        h{i} = g_j{i}/g_i{i};
                    end
                    h = SE2_mat2vec(h); % convert it to a vector form
            end
    end

end

