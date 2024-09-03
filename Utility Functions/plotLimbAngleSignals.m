function plotLimbAngleSignals(limbAngleStruct)
%PLOTLIMBANGLESIGNALS plot the limb angle signals in a tiledlayout
    % given structs holding the limb angle data, we extract relevant
    % information and plot the signals in the column array of tiledlayouts

    % set the fontsize here
    fS = 25;

    % get the number of components in each substruct
    numStances = size(limbAngleStruct, 1); % number of stances in each trace
    numTraces = size(limbAngleStruct, 2); % number of different types of traces
    numComponents = numel(limbAngleStruct{1, 1}); % in each stance, number of components
    numPlotTiles = numComponents*numStances;

    % start plotting
    f = figure('Visible', 'on', 'Units', 'pixels',...
                                                'Position', [0 0 900 700]);
    tl = tiledlayout(f, numPlotTiles, 1, "TileSpacing", "tight",...
                                                    "Padding", "tight");
    for i = 1:numStances
        for j = 1:numComponents
            ax = nexttile(tl); ax.FontSize = fS; hold(ax, "on");
            % iterate over each trace
            for k = 1:numTraces
                % unpack
                tau = limbAngleStruct{i, k}{j}.tau;
                beta = limbAngleStruct{i, k}{j}.beta;
                signal = limbAngleStruct{i, k}{j}.signal;
                stanceColor = limbAngleStruct{i, k}{j}.stanceColor;
                swingColor = limbAngleStruct{i, k}{j}.swingColor;
                scatterFlag = limbAngleStruct{i, k}{j}.scatterFlag;
                lineSty = limbAngleStruct{i, k}{j}.lineSty;
                cs = limbAngleStruct{i, k}{j}.cs;
                % plot
                switch isempty(scatterFlag)
                    case 1
                        plotLimbAngleTimeseriesOnAxes...
                            (ax, tau, beta, rad2deg(signal), ...
                            stanceColor, swingColor);
                    case 0
                        switch isempty(lineSty)
                            case 1
                                plotLimbAngleTimeseriesOnAxes...
                                    (ax, tau, beta, rad2deg(signal), ...
                                    stanceColor, swingColor, ...
                                    scatterFlag);
                            case 0
                                plotLimbAngleTimeseriesOnAxes...
                                    (ax, tau, beta, rad2deg(signal), ...
                                    stanceColor, swingColor, ...
                                    scatterFlag, lineSty);
                        end
                end
            end
            axis(ax, "padded"); xlim(ax, [0, 2*pi]); grid(ax, "on");
            ylabel(['$$\alpha_{' num2str(cs) '}$$'], 'FontSize', fS, ...
                                                'Interpreter', 'latex');
            ax.YColor = stanceColor;
            switch (i-1)*numStances + j < numPlotTiles
                case 1 % not last tile
                    ax.XTickLabel = ''; 
                case 0 % LAST tile
                    xlabel(ax, '$$\tau$$', 'FontSize', fS,...
                                                'Interpreter', 'latex');
            end
        end
    end
    
    %%% END OF FUNCTION %%%
end

%%
% f = figure('Visible', 'on', 'Units', 'pixels', 'Position', [0 0 900 700]);
% tl = tiledlayout(f, 4, 1, "TileSpacing", "tight", "Padding", "tight");
% ax = nexttile(tl); ax.FontSize = fS; hold(ax, "on");
% plotLimbAngleTimeseriesOnAxes...
%     (ax, tau13, beta13, rad2deg(subgait13(:, 1)), stanceColor13, swingColor13);
% axis(ax, "padded"); xlim(ax, [0, 2*pi]); 
% ylabel(['$$\alpha_{' num2str(FRHLstance.cs(1)) '}$$'], 'FontSize', fS, 'Interpreter', 'latex');
% ax.XTickLabel = ''; grid(ax, "on");
% ax = nexttile(tl); ax.FontSize = fS; hold(ax, "on");
% plotLimbAngleTimeseriesOnAxes...
%     (ax, tau13, beta13, rad2deg(subgait13(:, 2)), stanceColor13, swingColor13);
% axis(ax, "padded"); xlim(ax, [0, 2*pi]); 
% ylabel(['$$\alpha_{' num2str(FRHLstance.cs(2)) '}$$'], 'FontSize', fS, 'Interpreter', 'latex');
% ax.XTickLabel = ''; grid(ax, "on");
% ax = nexttile(tl); ax.FontSize = fS; hold(ax, "on");
% plotLimbAngleTimeseriesOnAxes...
%     (ax, tau24, beta24, rad2deg(subgait24(:, 1)), stanceColor24, swingColor24);
% axis(ax, "padded"); xlim(ax, [0, 2*pi]); 
% ylabel(['$$\alpha_{' num2str(FLHRstance.cs(1)) '}$$'], 'FontSize', fS, 'Interpreter', 'latex');
% ax.XTickLabel = ''; grid(ax, "on");
% ax = nexttile(tl); ax.FontSize = fS; hold(ax, "on");
% plotLimbAngleTimeseriesOnAxes...
%     (ax, tau24, beta24, rad2deg(subgait24(:, 2)), stanceColor24, swingColor24);
% axis(ax, "padded"); xlim(ax, [0, 2*pi]); 
% ylabel(['$$\alpha_{' num2str(FLHRstance.cs(2)) '}$$'], 'FontSize', fS, 'Interpreter', 'latex');
% xlabel('$$\tau$$', 'FontSize', fS, 'Interpreter', 'latex');  grid(ax, "on");