% This function generates and orders the contact submanifolds from level-0 to level-n for n-contacting points.
function out = gencontactsubmanifolds(n)
    
    % contacting point array
    c = 1:n;

    % output container
    out = double.empty(0, n);

    % start generating the contact submanifolds
    for i = 1:n
        
        tempi = gpuArray(nchoosek(c, i)); % compute the contacting points of the submanifold-- we shall consider each row in "tempi" array to be a set
        
        if i > 1
            
            for j = 1: size(tempi, 1)
                
                flag_i = ndimlevicivita( tempi(j, :) );

                if flag_i == -1

                    tempj = perms( tempi(j, :) ); flag_j = nan(size(tempj, 1), 1);      % create all permutations of the problem set
                    
                    for k = 1: size(tempj, 1)
                        
                        flag_j(k) = ndimlevicivita( tempj(k, :) );                      % check each permutation if it is even

                    end

                    tempj = tempj(flag_j == 1);                                         % reduce to only even permutations
                    [~, row_idx] = min( tempj(:, 1) );                                  % find the row no even permutation with the smallest leading entry
                    tempi(j, :) = tempj(row_idx, :);                                    % replace the initial set with the computed smallest leading entry
                                                                                                                                        % even permute set

                end

            end
            
            % this section generates "tempi_tau" from tempi as a cyclic phase value-- this makes the difference generation around n points equally distributed
            % on a circle-- this means the difference between 1 and 2 is the same as the difference between 4 and 1
            switch rem(n, 2)                                            
                case 0
                    tempi_C = pi*(2*tempi - 1)/n;                                   % n is even
                case 1                                                  
                    tempi_C = pi*(tempi - 1)/n;                                     % n is odd
            end
            tempi_C = cos(tempi_C) + 1i*sin(tempi_C);                               % convert phases into complex numbers along a unit circle- C1
            

            [diffipool, circshnum] = c1diff(tempi_C);                               % initialize total C1 difference
            tempi(circshnum == 1, :) = circshift(tempi(circshnum == 1, :), -1);
            diffipool(circshnum == 1, :) = -diffipool(circshnum == 1, :);
            totaldiff = unique(diffipool);                                          % find the distinct pools
            pool = [];                                                              % initialize an empty pool array to obtain the sorted tempi
    
            for j = 1:numel(totaldiff)
                pool_temp = tempi(diffipool == totaldiff(j), :);                    % obtain the current pool
                pool_temp = sortfirstcolumn(pool_temp);                           % sort it by each column
                pool = [pool; pool_temp];                                           % append the sorted set to the bottom
            end

        else

            pool = tempi;

        end

        out = [out; [pool, nan(size(pool, 1), n - size(pool, 2))] ];                % concatenate lc +1ed, pooled, and L2Rcolwise sorted sets to the output

    end


end