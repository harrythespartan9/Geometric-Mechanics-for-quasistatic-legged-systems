function f = plotSE2timeseries(input)
%PLOTSE2TIMESERIES This function plots the input cell array time series
%   Given an input cell array of time series, this function plots them versus time in the same order as outlined in the cell array.
    
    % Unpack
    t = input{1};
    in = input{2};                              % contains all the time series data in the order to be plotted
    arr_size = size(in);                        % size of the input data cell array
    if numel(arr_size) > 2
        error('ERROR! The plot data array can only be 2D for visualization.');
    end
    in_str = input{3};                          % all y-axis labels corresponding to the time series data in latex format
    legend_str = input{4};                      % strings corresponding to each time series trace under one cell
    trace_num = numel(input{2}{1, 1});          % get the number of traces in each time series
    % col = [zeros(1, 3); turbo(trace_num - 1)];
    col = turbo(trace_num);                     % get colors for each trace (first trace is the experimental result in black)

    % Setup
    f = figure('units','pixels','position',360*[0 0 arr_size(1) 1.5*arr_size(2)],'Color','w');
    switch arr_size(2)
        case 1
            Tobj = tiledlayout('vertical', 'TileSpacing', 'tight', 'Padding', 'tight');
            for i = 1:numel(in)
                ax = nexttile(Tobj);
                switch i == numel(in)
                    case 0
                        plotSE2timeseries_snapshot(ax, t, in{i}, in_str{i}, col);
                    case 1
                        plotSE2timeseries_snapshot(ax, t, in{i}, in_str{i}, col, legend_str);
                end
            end
        otherwise
            Tobj = tiledlayout(arr_size(1), arr_size(2), 'TileSpacing', 'tight', 'Padding', 'tight');
            for j = 1:arr_size(2)
                for i = 1:arr_size(1)
                    tile_idx = i + (j-1)*arr_size(1);
                    ax = nexttile(Tobj, tile_idx);
                    switch tile_idx == numel(in)
                        case 0
                            plotSE2timeseries_snapshot(ax, t, in{i}, in_str{i}, col);
                        case 1
                            plotSE2timeseries_snapshot(ax, t, in{i}, in_str{i}, col, legend_str);
                    end
                end
            end
    end


end

%% HELPER FUNCTIONS

function plotSE2timeseries_snapshot(ax, t, in, in_str, col, legend_str)
%PLOTSE2TIMESERIES_SNAPSHOT this function plots a single cell of the time series data
%   For instance, this cell could be the time series concerning the swing values of the FR leg for a bunch of estimates.
    
    % plot
    fS = 10;
    for i = 1:numel(in)
        plot(ax, t, in{i}, 'LineWidth', 2.0, 'Color', col(i, :));
        if i == 1
            grid on; hold on; set(ax,'TickLabelInterpreter','latex')
        end
    end
    ylabel(in_str, 'FontSize', fS, 'Interpreter', 'latex'); ax.FontSize = fS;
    
    switch nargin
        case 5
            xticklabels('');
        otherwise
            xlabel('$$t$$', 'FontSize', fS, 'Interpreter', 'latex');
            if ~isempty(legend_str) % if the legend string is not empty
                legend(ax, legend_str, 'Location', 'bestoutside', 'FontSize', fS);
            end
    end

end