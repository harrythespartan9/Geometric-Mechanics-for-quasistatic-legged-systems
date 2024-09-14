function pltBody = formatBodyTimeseriesForPlotting...
                (g, c, pltMode, lineSty, scatterFlag, stanceObjs)
%FORMATBODYTIMESERIESFORPLOTTING formats body timeseries for plotting
%   Plots the body timeseries in a column tiledlayout when provided with
%   the timeseries, stance phase indices, the plotting structure: mode and
%   corresponding color, and line style.
%   The stanceObjs' kth index might be in the 'c' variable

    % define defaults if not all arguments are present and error out for
    % wrong data
    if nargin < 1
        error(['ERROR! An SE(2) body trajectory timeseries, "g", is ' ...
            'needed with multiple rows corresponding to body position at ' ...
            'each point.']);
    end
    if size(g, 2) ~= 3
        error(['ERROR! For SE(2) body trajectories, three columns ' ...
            'corresponding to x, y, and theta directions (from left to ' ...
            'right) are needed.']);
    end
    if size(g, 1) <= 1
        error(['ERROR! For SE(2) body trajectories, more than one ' ...
            'datapoint is required along the rows.']);
    end
    num = size(g, 1);
    if nargin < 2 % different stance modes as a timeseries vector
        c = ones([num, 1]);
    end
    if ~(isrow(c) || iscolumn(c))
        error(['ERROR! The contact state timeseries should be a row or ' ...
            'column vector.']);
    end
    if size(g, 1) ~= numel(c)
        error(['ERROR! The number of points in the body trajectory ' ...
            'timeseries should be equal to the contact state timeseries']);
    end
    if nargin < 6
        pltMode= 'k';
    end
    if nargin < 5
        scatterFlag = false;
    end
    if nargin < 4 % default plotting line style
        lineSty = '-';
    end

    % get the indices where the stance phase changed
    stanceChangeLocs = find( diff(c) ~= 0 ) + 1;
    startEndLocs = stanceChangeLocs; % get the plot locations
    if stanceChangeLocs(1) ~= 1
        startEndLocs = [1, startEndLocs];
    end
    if stanceChangeLocs(end) ~= num
        startEndLocs = [startEndLocs, num];
    end
    startIdx = startEndLocs(1:end-1);
    endIdx = startEndLocs(2:end);
    numSegmts = numel(startIdx);

    % append the first index to the "stanceChangeLocs" if the last segment
    % of the trajectory is not the same as the first segment of the
    % trajectory
    if c(end) ~= c(1)
        stanceChangeLocs = [1, stanceChangeLocs];
    end
    numChange = numel(stanceChangeLocs);

    % get the colors at the points of change and at the different
    % trajectory segments
    stanceColors = nan(numChange, 3); % remains NaN for case 'k'
    switch pltMode
        case 'stance_colored'
            % ... need colors when the stance phase or contact state
            % ... changes and the colors for each segment
            for i = 1:numChange
                stanceColors(i, :) = ...
                    stanceObjs{c(stanceChangeLocs(i))}.p_info.gc_col;
            end
            trajSegmentColors = nan(numSegmts, 3);
            for i = 1:numSegmts
                trajSegmentColors(i, :) = ...
                    stanceObjs{c(startIdx(i))}.p_info.gc_col;
            end
        case 'k'
            % ... default case, so we use the default color
            trajSegmentColors = repmat(zeros(1, 3), [numSegmts, 1]);
    end

    % directly init and assign return struct
    pltBody.g = g;
    pltBody.startIdx = startIdx;
    pltBody.endIdx = endIdx;
    pltBody.stanceChangeIdx = stanceChangeLocs;
    pltBody.stanceColors = stanceColors;
    pltBody.trajSegmentColors = trajSegmentColors;
    pltBody.lineSty = lineSty;
    pltBody.scatterFlag = scatterFlag;

end

