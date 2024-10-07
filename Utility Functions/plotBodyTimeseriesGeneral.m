function plotBodyTimeseriesGeneral( pltBody, pltType, traceTickLabels )
%PLOTBODYTIMESERIESGENERAL plot the body timeseries
%   Given a structure of different body cell arrays, we plot them in order
%   prescribed by the linestyle within the struct, "pltBody".
%   For more information on the body trajectory structure and its fields,
%   check "Utility Functions\formatBodyTimeseriesForPlotting.m" and for
%   examples of usage refer to the methods in the class "Utility 
%   Functions\altQuadGait.m" and livescript 
%   "se2_toyproblems_case_1_mobility.mlx".

    % define the ylabels
    xLabelTxt = '$$\tau$$';
    if nargin < 2
        pltType = 'g'; % set it to body position timeseries by default
    end
    switch pltType
        case 'g' % just body position trajectory
            yLabelTxt = {'$$x$$', '$$y$$', '$$\theta$$'};
        case 'gCirc' % just body average velocity trajectory
            yLabelTxt = {'$$\xi^{x}$$', '$$\xi^{y}$$', '$$\xi^{\theta}$$'};
    end

    % setup
    % ... set the fontsize here
    fS = 25;
    % ... initialize figure and tiledlayouts
    f = figure('Visible', 'on', 'Units', 'pixels',...
                                                'Position', [0 0 900 900]);
    tl = tiledlayout(f, 3, 1, "TileSpacing", "tight",...
                                                    "Padding", "tight");

    % iterate, unpack, and plot
    switch isstruct(pltBody)
        case 1 % a single trajectory struct (WORKS!)
            % .. unpack
            g = pltBody.g;
            gLimits = pltBody.gLimits;
            startIdx = pltBody.startIdx; 
            endIdx = pltBody.endIdx;
            stanceChangeLocs = pltBody.stanceChangeIdx;
            stanceColors = pltBody.stanceColors;
            trajSegmentColors = pltBody.trajSegmentColors;
            lineSty = pltBody.lineSty;
            scatterFlag = pltBody.scatterFlag;
                numSegmnts = numel(startIdx); 
                numCh = numel(stanceChangeLocs);
            % define the gait phase
            % ... because the body timeseries are defined over a gait 
            % ... cycles, we need a gait phase vector \in [0 to 2\pi)
            tau = linspace(0, 2*pi, size(g, 1)+1); tau = tau(1:end-1);
            % ... plot
            for i = 1:3
                ax = nexttile(tl); ax.FontSize = fS; hold(ax, "on");
                for k = 1:numSegmnts
                    idxNow = startIdx(k):endIdx(k);
                    plot(ax, ...
                        tau(idxNow), g(idxNow, i), lineSty, ...
                        'LineWidth', 2.0, 'Color', trajSegmentColors(k,:));
                end
                if scatterFlag
                    for k = 1:numCh
                        locNow = stanceChangeLocs(k);
                        scatter(ax, ...
                            tau(locNow), g(locNow, i), ...
                            100, 'o', "filled", ...
                            "MarkerFaceColor", stanceColors(k, :), ...
                            "MarkerEdgeColor", 'k', ...
                            "LineWidth", 1.2);
                    end
                end
                gLimitsComp = gLimits(i, :);
                gLimitsComp = gLimitsComp + 0.15*diff(gLimitsComp)*[-1 +1];
                axis(ax, "padded"); grid(ax, "on");
                xlim(ax, [0, 2*pi]); ylim(ax, gLimitsComp);
                if i == 3
                    xlabel(ax, xLabelTxt, 'FontSize', fS,...
                                                'Interpreter', 'latex');
                else
                    ax.XTickLabel = '';
                end
                ylabel(ax, yLabelTxt{i}, 'FontSize', fS, ...
                                                'Interpreter', 'latex');
                % set(get(ax,'YLabel'), 'rotation', 0, ...
                %                             'VerticalAlignment', 'middle');
            end
        case 0 % a cell array of trajectories as structs (UNTESTED)
            % iterate and plot over each object
            numObjs = numel(pltBody);
            % get the ticklabels, else create it along the way
            if nargin < 2
                getLabelFlag = true;
                traceTickLabels = cell(1, numObjs);
            else
                getLabelFlag = false;
            end
            for j = 1:numObjs
                if ~isstruct(pltBody{j})
                    error(['ERROR! The "pltBody" input argument should be ' ...
                        'a cell array of body timeseries structures.']);
                else
                    g{j} = pltBody{j}.g;
                    gLimits{j} = pltBody{j}.gLimits;
                    startIdx{j} = pltBody{j}.startIdx; 
                    endIdx{j} = pltBody{j}.endIdx;
                    stanceChangeLocs{j} = pltBody{j}.stanceChangeIdx;
                    stanceColors{j} = pltBody{j}.stanceColors;
                    % ... ideally, "trajSegmentColors" is the same for all
                    % ... trajectories
                    trajSegmentColors{j} = pltBody{j}.trajSegmentColors;
                    lineSty{j} = pltBody{j}.lineSty;
                    scatterFlag{j} = pltBody{j}.scatterFlag;
                        numSegmnts{j} = numel(startIdx{j}); 
                            numCh{j} = numel(stanceChangeLocs{j});
                    tau{j} = linspace(0, 2*pi, size(g{j}, 1)+1); 
                        tau{j} = tau{j}(1:end-1);
                end
            end
            for i = 1:3
                ax = nexttile(tl); ax.FontSize = fS; hold(ax, "on");
                for j = 1:numObjs
                    for k = 1:numSegmnts{j}
                        plot(ax, ...
                            tau{j}(startIdx{j}(k):endIdx{j}(k)), ...
                            g{j}(startIdx{j}(k):endIdx{j}(k), i), ...
                            lineSty{j}, 'LineWidth', 2.0, ...
                            'Color', trajSegmentColors{j}(k, :));
                    end
                    if scatterFlag{j}
                        for k = 1:numCh{j}
                            scatter(ax, ...
                                tau(stanceChangeLocs{j}(k)), ...
                                g(stanceChangeLocs{j}(k), i), ...
                                100, 'o', "filled", ...
                                "MarkerFaceColor", stanceColors{j}(k, :), ...
                                "MarkerEdgeColor", 'k', ...
                                "LineWidth", 1.2);
                        end
                    end
                    if getLabelFlag
                        traceTickLabels{j} = num2str(j);
                    end
                    switch j
                        case 1
                            ithAxLim = gLimits{j}(i, :);
                        otherwise
                            ithAxLim = [ithAxLim; gLimits{j}(i, :)];
                    end
                end
                gLimitsComp = [min(ithAxLim(:, 1)), max(ithAxLim(:, 2))];
                gLimitsComp = gLimitsComp + 0.15*diff(gLimitsComp)*[-1 +1];
                axis(ax, "padded"); grid(ax, "on");
                xlim(ax, [0, 2*pi]); ylim(ax, gLimitsComp);
                if i == 3
                    xlabel(ax, xLabelTxt, 'FontSize', fS,...
                                                'Interpreter', 'latex');
                else
                    ax.XTickLabel = '';
                end
                ylabel(ax, yLabelTxt{i}, 'FontSize', fS, ...
                                                'Interpreter', 'latex');
                % set(get(ax,'YLabel'), 'rotation', 0, ...  
                %                           'VerticalAlignment', 'middle');
            end
            % add the colorbar on the outside
            % ... first obtain the colormap from the segmented data
            colorMapNow = nan(numel(trajSegmentColors), 3);
            for j = 1:size(colorMapNow, 1)
                colorMapNow(j ,:) = mean(trajSegmentColors{j});
            end
            % ... add the map and assign the colorbar
            tempArr = linspace(0, 1, numObjs+1);
            traceTickLocations = tempArr(1:end-1) + 0.5*diff(tempArr);
            colormap(f, colorMapNow); 
            cb = colorbar(ax, ...
                'TickLabelInterpreter', 'latex',...
                'FontSize', fS, ...
                'Ticks', traceTickLocations, ...
                'TickLabels', traceTickLabels, ...
                'TickDirection', 'in');
            cb.Layout.Tile = 'east';
    end
    %%%% END OF FUNCTION
end

