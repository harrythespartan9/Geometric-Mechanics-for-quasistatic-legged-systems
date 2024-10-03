function plotBodyTimeseriesGeneral( pltBody )
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
    yLabelTxt = {'$$x$$', '$$y$$', '$$\theta$$'};

    % setup
    % ... set the fontsize here
    fS = 25;
    % ... initialize figure and tiledlayouts
    f = figure('Visible', 'on', 'Units', 'pixels',...
                                                'Position', [0 0 900 700]);
    tl = tiledlayout(f, 3, 1, "TileSpacing", "tight",...
                                                    "Padding", "tight");

    % iterate, unpack, and plot
    switch isstruct(pltBody)
        case 1 % a single trajectory struct (WORKS!)
            % .. unpack
            g = pltBody.g;
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
                axis(ax, "padded"); xlim(ax, [0, 2*pi]); grid(ax, "on");
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
            numObjs = numel(pltBody);
            for j = 1:numObjs
                if ~isstruct(pltBody{j})
                    error(['ERROR! The "pltBody" input argument should be ' ...
                        'a cell array of body timeseries structures.']);
                else
                    g{j} = pltBody{j}.g;
                    z{j} = pltBody{j}.z;
                    startIdx{j} = pltBody{j}.startIdx; 
                    endIdx{j} = pltBody{j}.endIdx;
                    stanceChangeLocs{j} = pltBody{j}.stanceChangeIdx;
                    stanceColors{j} = pltBody{j}.stanceColors;
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
                end
                axis(ax, "padded"); xlim(ax, [0, 2*pi]); grid(ax, "on");
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
    end
    %%%% END OF FUNCTION
end

