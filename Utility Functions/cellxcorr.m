function [outXYxcorr, outshiftY] = cellxcorr(refX, inY, max_shift)
%CELLXCORR this function computes the cross correlation for each cell input
%   Given the task of computing cross-correlation between time series, vector trajectories, this function computes it for each corresponding element. Each
%   element-wise time series are separated in separate cells arranged row-wise in a column cell array.
%   The max_shift input is computed outside this function using the gait frequency and discretization information so that two signals being cross-correlated 
%   aren't shifted more than half the gait time period. We divide the correlation obtained by the max possible correlation obtainable-- 0 auto-correlation of
%   "refX".

    switch iscell(refX) && iscell(inY)
        
        case 0
            error('ERROR! Both reference vector signal and input vector signal need to have the same data structure.');
    
        case 1
    
            switch numel(refX) == numel(inY)
                case 0
                    error('ERROR! Both signals should have the same number of elements.');
                case 1
                    outXYxcorr = cell(size(inY)); outshiftY = outXYxcorr; % initialize
                    for i = 1:numel(refX)
                        [maxXcorr, shift_temp] = xcorr(refX{i}, refX{i}, max_shift); maxXcorr = maxXcorr(shift_temp == 0); % compute max auto-correlation of reference signal
                        [outXYxcorr{i}, outshiftY{i}] = xcorr(refX{i}, inY{i}, max_shift); outXYxcorr{i} = outXYxcorr{i}/maxXcorr;
                    end
            end
    
    end


end

