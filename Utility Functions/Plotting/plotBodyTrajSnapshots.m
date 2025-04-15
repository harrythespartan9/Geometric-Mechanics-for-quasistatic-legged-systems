function plotBodyTrajSnapshots(BL, ...
                    timeAlongTrajs, gTrajs, bodyTrajs, ...
                    feetStanceOrder, stanceBodyTrajs, trajStr)
%PLOTBODYTRAJSNAPSHOTS plot the body trajectory snapshots for a given gait
%cycle
%   Detailed explanation goes here

    % obtain the number of gaits (separate out the full time vector) and 
    % setup colors for each
    tFull = gTrajs{1}; gTrajs = gTrajs(2:end);
    bNum = numel(gTrajs);
    load('Data\Plotting\qualitativeColorMap', '-mat', 'qualColMap');

    % if multiple trajectories are provided, just choosed a fixed color for
    % each trajectory, else use black with end of stance phases
    % highlighted
    switch bNum ~= 1
        case 1
            stanceFlag = false;
            bCol = resizeColorMap( qualColMap, bNum );
        case 0
            stanceFlag = true;
            load('Data\Sinusoidal Gaits\clariConnVecF.mat', ...
                'col13', 'col24');
    end
    load('Data\Sinusoidal Gaits\clariConnVecF.mat', 'lW_s', 'lW_V');

    % plot the local connection vector fields
    % ... plot it in 2 cols for each contact state
    fS = 12; lWb = 0.05; circS = 200; circSfs = 100;
    % 
    f = figure('units', 'pixels', 'position', [3200 -300 700 600]); 
    set(f,'Visible','on');
    % %
    ax = gca; hold(ax, "on");
    % plot they x and y-axes to indicate the origin
    p = xline( ax, 0, "k:", "LineWidth", 0.5*lW_V);
    set(get(get(p,'Annotation'),'LegendInformation'), ...
        'IconDisplayStyle','off');
    p = yline( ax, 0, "k:", "LineWidth", 0.5*lW_V);
    set(get(get(p,'Annotation'),'LegendInformation'), ...
        'IconDisplayStyle','off');
    % plot the body trajectory
    switch stanceFlag
        case 1
            plotBodyTrajectoryWStanceHighlights...
                (ax, ...
                tFull, timeAlongTrajs, gTrajs, bodyTrajs, ...
                feetStanceOrder, stanceBodyTrajs, ...
                BL, lWb, lW_s, lW_V, circS, circSfs, ...
                col13, col24);
            title(ax, [trajStr{1} ' gait'], FontSize=fS);
        case 0
            for b = 1:bNum
                plotBodyTrajectoryWOStanceHighlights...
                (ax, b, ...
                timeAlongTrajs, gTrajs, bodyTrajs, feetStanceOrder, BL, ...
                lWb, lW_s, circS, circSfs, bCol, trajStr);
            end
            title(ax, 'Body trajectory', FontSize=fS);
            legend(ax, 'location', 'northeastoutside');
    end    
    axis(ax, "equal"); ax.FontSize = fS;
    [xL, yL, xT, yT] = findLimits(bodyTrajs, BL);
    xlim(ax, xL); ylim(ax, yL);
    xticks(ax, xT); yticks(ax, yT);
    xlabel(ax, '$x$-position (BL)', FontSize=fS);
    ylabel(ax, '$y$-position (BL)', FontSize=fS);
end

%% AUXILIARY FUNCTIONS

% plot the body trajector with or without the stance phase highlighted with
% the corresponding colors for the two contact states
function plotBodyTrajectoryWOStanceHighlights...
            (ax, b, T, gT, bT, feetStOrd, BL, lWb, lW_s, circS, circSfc, bCol, str)
    % get the time points at each stance
    stIdx = cell(1, 2);
    stIdx{1} = (T{b} >= 0 & T{b} <= 0.5);
    stIdx{2} = (T{b} >= 0.5 & T{b} <= 1.0);
    % unpack vars
    currBody = bT.body{b}; 
    currFeet = bT.foot{b}; numFeet = numel(currFeet);
    currTraj = gT{b};
    % plot the body's bounding boxes
    p = quiver(ax, ...
                currBody.X/BL, currBody.Y/BL, ...
                currBody.U/BL, currBody.V/BL, ...
                "LineWidth", lWb, "Color", bCol(b, :), ...
                "AutoScale", 'off', 'ShowArrowHead', 'off');
    set(get(get(p,'Annotation'),'LegendInformation'), ...
        'IconDisplayStyle','off');
    % plot the foot locations
    for i = 1:numFeet
        currFoot = currFeet{i};
        p = scatter(ax, ...
                currFoot.X( stIdx{feetStOrd(i)} )/BL, ...
                currFoot.Y( stIdx{feetStOrd(i)} )/BL, ...
                circS, "Marker", 'o', "MarkerEdgeColor",  bCol(b, :));
        set(get(get(p,'Annotation'),'LegendInformation'), ...
            'IconDisplayStyle','off');
    end
    % plot the entire body trajectory
    plot(ax, currTraj(:, 1)/BL, currTraj(:, 2)/BL, ...
        'LineWidth', 3*lW_s, 'Color', bCol(b, :), ...
        'DisplayName', str{b});
    % scatter a final body location
    p = scatter(ax, currTraj(end, 1)/BL, currTraj(end, 2)/BL, ...
        circSfc, bCol(b, :), ...
        'filled', 'Marker', 'o', 'MarkerEdgeColor', 'k');
    set(get(get(p,'Annotation'),'LegendInformation'), ...
            'IconDisplayStyle','off');
    % add a quiver to indicate the orientation of the robot
    % ... set the size of the arrow (in percentage, last argument) to 10% 
    % ... of the body length or smaller
    qFC = ... % added BL adjustments
    returnQuiverSize(BL, currTraj(end, :), 2);
    p = quiver(ax, ...
                qFC.X, qFC.Y, qFC.U, qFC.V, ...
                "LineWidth", 2*lW_s, "Color", bCol(b, :), ...
                "AutoScale", 'off');
    set(get(get(p,'Annotation'),'LegendInformation'), ...
        'IconDisplayStyle','off');
end

function plotBodyTrajectoryWStanceHighlights...
    (ax, t, T, gT, bT, ...
    feetStOrd, sbT, BL, lWb, lW_s, lW_V, circS, circSfc, colI, colJ)
    % get the time points at each stance
    stIdx = cell(1, 2);
    stIdx{1} = (T{1} >= 0 & T{1} <= 0.5);
    stIdx{2} = (T{1} >= 0.5 & T{1} <= 1.0);
    tIdx{1} = (t >= 0 & t <= 0.5);
    tIdx{2} = (t >= 0.5 & t <= 1.0);
    % unpack vars
    currBody = bT.body{1}; 
    currFeet = bT.foot{1}; numFeet = numel(currFeet);
    currTraj = gT{1};
    currStBody = cell(1, 2); 
        currStFeet = cell(1, 2); 
            currStCentroid = cell(1, 2);
    currStBody{1} = sbT{1}{1}.body; 
            currStFeet{1} = sbT{1}{1}.foot;
                    currStCentroid{1} = sbT{1}{1}.COM;
    currStBody{2} = sbT{1}{2}.body; 
            currStFeet{2} = sbT{1}{2}.foot;
                    currStCentroid{2} = sbT{1}{2}.COM;
    % plot the body's bounding boxes
    quiver(ax, ... % 1st stance body trajectory
            currBody.X(stIdx{1}, :)/BL, currBody.Y(stIdx{1}, :)/BL, ...
            currBody.U(stIdx{1}, :)/BL, currBody.V(stIdx{1}, :)/BL, ...
            'k-', "LineWidth", lWb, ...
            "AutoScale", 'off', 'ShowArrowHead', 'off');
    quiver(ax, ... % position @ end of 1st stance
            currStBody{1}.X/BL, currStBody{1}.Y/BL, ...
            currStBody{1}.U/BL, currStBody{1}.V/BL, ...
            "LineWidth", lW_V, 'Color', colI, ...
            "AutoScale", 'off', 'ShowArrowHead', 'off');
    quiver(ax, ... % 2nd stance phase body trajectory
            currBody.X(stIdx{2}, :)/BL, currBody.Y(stIdx{2}, :)/BL, ...
            currBody.U(stIdx{2}, :)/BL, currBody.V(stIdx{2}, :)/BL, ...
            "LineWidth", lWb, "Color", colJ, ...
            "AutoScale", 'off', 'ShowArrowHead', 'off');
    quiver(ax, ... % position @end of 2nd stance
            currStBody{2}.X/BL, currStBody{2}.Y/BL, ...
            currStBody{2}.U/BL, currStBody{2}.V/BL, ...
            "LineWidth", lW_V, 'Color', colJ, ...
            "AutoScale", 'off', 'ShowArrowHead', 'off');
    % plot the foot locations
    for i = 1:numFeet
        currFoot = currFeet{i};
        switch feetStOrd(i)
            case 1
                colNow = colI; currStFoot = currStFeet{1}{i};
            case 2
                colNow = colJ; currStFoot = currStFeet{2}{i};
        end
        scatter(ax, ... % locations during stance
                currFoot.X( stIdx{feetStOrd(i)} )/BL, ...
                currFoot.Y( stIdx{feetStOrd(i)} )/BL, ...
                circS, 'k', "Marker", '.');
        scatter(ax, ... % final location at the end of stance
                currStFoot.X/BL, currStFoot.Y/BL, ...
                circS, colNow, 'filled', 'LineWidth', lW_V, ...
                "Marker", 'o', "MarkerEdgeColor",  'k');
    end
    % plot the entire body trajectory, a location of the body using
    % scatter, and the orientation of the body using quiver
    % ... things are ordered based on order in which the happen
    scatter(ax, ...
        currTraj(1, 1)/BL, currTraj(1, 2)/BL, circSfc, ...
        'k', 'filled', 'Marker', 'o', ...
        'MarkerEdgeColor', 'k', 'LineWidth', lW_V);
    qS0 = returnQuiverSize(BL, currTraj(1, :), 2);
    quiver(ax, qS0.X, qS0.Y, qS0.U, qS0.V, ...
        "LineWidth", 2*lW_s, "Color", 'k', "AutoScale", 'off');
    plot(ax, currTraj(tIdx{1}, 1)/BL, currTraj(tIdx{1}, 2)/BL, ...
        'LineWidth', 3*lW_s, 'Color', colI);
    scatter(ax, ...
        currStCentroid{1}.X/BL, currStCentroid{1}.Y/BL, circSfc, ...
        colI, 'filled', 'Marker', 'o', ...
        'MarkerEdgeColor', 'k', 'LineWidth', lW_V);
    qS1 = returnQuiverSize(BL, ...
        [currStCentroid{1}.X, currStCentroid{1}.Y, currStCentroid{1}.TH], ...
        2);
    quiver(ax, qS1.X, qS1.Y, qS1.U, qS1.V, ...
        "LineWidth", 2*lW_s, "Color", colI, "AutoScale", 'off');
    % % % % % %  repeat for the final stance
    plot(ax, currTraj(tIdx{2}, 1)/BL, currTraj(tIdx{2}, 2)/BL, ...
        'LineWidth', 3*lW_s, 'Color', colJ);
    scatter(ax, ...
        currStCentroid{2}.X/BL, currStCentroid{2}.Y/BL, circSfc, ...
        colJ, 'filled', 'Marker', 'o', ...
        'MarkerEdgeColor', 'k', 'LineWidth', lW_V);
    qS2 = returnQuiverSize(BL, ...
        [currStCentroid{2}.X, currStCentroid{2}.Y, currStCentroid{2}.TH], ...
        2);
    quiver(ax, qS2.X, qS2.Y, qS2.U, qS2.V, ...
        "LineWidth", 2*lW_s, "Color", colJ, "AutoScale", 'off');
end

% compute a quiver at the final orientation of the body after executing the
% gait cycle
function quivOut = returnQuiverSize(BL, gT, prct)
    % find the rotation matrix at the final orientation of the body
    % trajectory
    rotMat = rot_SE2(gT(3));
    % quiver is at the final body position
    quivOut.X = gT(1)/BL; quivOut.Y = gT(2)/BL;
    % quiver is pointing in the y-direction, which is extracted and size
    % modified from the rotation matrix generated earlier
    quivOut.U = prct/100*BL*[1, 0, 0]*rotMat*[0; 1; 0];
    quivOut.V = prct/100*BL*[0, 1, 0]*rotMat*[0; 1; 0];
end

% compute the ticks and limits for plotting the body trajectory
function [xL, yL, xT, yT] = findLimits(bT, BL)
    % define a resolution for the ticks
    blRes = 0.50;
    % obtain all the x and y plotting locations
    xLoc = []; yLoc = [];
    for b = 1:numel(bT.body)
        currBody = bT.body{b}; currFeet = bT.foot{b}; % unpack
        numFeet = numel(currFeet);
        xLoc = [ xLoc, currBody.X(:)' ]; % concatenate
        yLoc = [ yLoc, currBody.Y(:)' ];
        for i = 1:numFeet % iterate, unpack, and concatenate
            currFoot = currFeet{i};
            xLoc = [ xLoc, currFoot.X(:)' ];
            yLoc = [ yLoc, currFoot.Y(:)' ];
        end
    end
    % normalize the these locations with the body length and compute the
    % limits and ticks locations
    xLoc = xLoc/BL; yLoc = yLoc/BL;
    [xL, xT] = computeLimitsAndTicks(blRes, xLoc);
    [yL, yT] = computeLimitsAndTicks(blRes, yLoc);
end
function [datLim, datTic] = computeLimitsAndTicks(tickRes, dat)
    % find the actual limits, mean of the data, and range
    datMin = min(dat, [], "all"); datMax = max(dat, [], 'all'); 
    datMean = (datMax + datMin)/2; datRange = datMax - datMin;
    % pad the range first by 10% and then by whatever is needed to make it 
    % an exact integer multiple of the tick resolution
    datModRange = tickRes*ceil(datRange*1.10/tickRes);
    % find a new domain for the limits
    datLim = datMean*[1, 1] + datModRange/2*[-1, 1];
    % find a slightly smaller domain for the limits
    datTicLim = datMean*[1, 1] + 0.90*datModRange/2*[-1, 1];
    % now return the ticks for the same
    datTic = round(datTicLim(1):tickRes:datTicLim(2), 1);
end
