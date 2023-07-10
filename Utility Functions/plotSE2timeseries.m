function f = plotSE2timeseries(input, plotF)
%PLOTSE2TIMESERIES This function plots the input cell array time series
%   Given an input cell array of time series and a plot-flag array, this function plots them versus time in the same order as outlined in the cell array. The
%   plot-flag array contains if a specific element should be skipped or plotted.
    
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
    if numel(input) < 5
        col = turbo(trace_num);                 % get colors for each trace (first trace is the experimental result in black)
        sty = cell(trace_num, 1);               % initialize line style container and set all of them to be a solid trace, '-' by default 
        lW = sty;                               % initialize line width container and set all of them to 2.0
        for i = 1:trace_num
            sty{i} = '-'; lW{i} = 2.0;
        end
    else
        col = input{5}{1}; sty = input{5}{2};   % user specified colors and line styles
        lW = input{5}{3};
    end
    trace_pack = {col, sty, lW};                % pass this to the trace plotter below

    % Setup
    f = figure('units','pixels','position',240*[0 0 arr_size(1) 1.5*arr_size(2)],'Color','w'); 
    set(f, 'Visible', 'on');
%     switch arr_size(2)
%         case 1
%             Tobj = tiledlayout(f, arr_size(1), arr_size(2), 'TileSpacing', 'tight', 'Padding', 'tight');
%             for i = 1:numel(in)
%                 ax = nexttile(Tobj);
%                 switch i == numel(in)
%                     case 0
%                         plotSE2timeseries_snapshot(ax, t, in{i}, in_str{i}, trace_pack);
%                     case 1
%                         plotSE2timeseries_snapshot(ax, t, in{i}, in_str{i}, trace_pack, legend_str);
%                 end
%             end
%         otherwise
    Tobj = tiledlayout(f, arr_size(1), arr_size(2), 'TileSpacing', 'tight', 'Padding', 'tight');
    for j = 1:arr_size(2)
        for i = 1:arr_size(1)
            tile_idx = i + (j-1)*arr_size(1);
            ax = nexttile(Tobj, tile_idx);
            switch tile_idx == numel(in)
                case 0
                    plotSE2timeseries_snapshot(ax, t, plotF, in{i}, in_str{i}, trace_pack);
                case 1
                    plotSE2timeseries_snapshot(ax, t, plotF, in{i}, in_str{i}, trace_pack, legend_str);
            end
        end
    end
%     end


end

%% HELPER FUNCTIONS

function plotSE2timeseries_snapshot(ax, t, plotF, in, in_str, in_col_sty_lW, legend_str)
%PLOTSE2TIMESERIES_SNAPSHOT this function plots a single cell of the time series data
%   For instance, this cell could be the time series concerning the swing values of the FR leg for a bunch of estimates.
    
    % unpack color and line style
    col = in_col_sty_lW{1}; sty = in_col_sty_lW{2}; lW = in_col_sty_lW{3};

    % plot
    fS = 10;
    for i = 1:numel(in)
        if plotF{i} % if needed, plot the corresponding component
            plot(ax, t, in{i}, sty{i}, 'LineWidth', lW{i}, 'Color', col(i, :));
        end
        if i == 1
            grid on; hold on; set(ax,'TickLabelInterpreter','latex');
            xlim([t(1) t(end)]);
        end
    end
    ylabel(in_str, 'FontSize', fS, 'Interpreter', 'latex'); ax.FontSize = fS;
    
    switch nargin
        case 6
            xticklabels('');
        otherwise
            xlabel('$$t$$', 'FontSize', fS, 'Interpreter', 'latex');
            if ~isempty(legend_str) % if the legend string is not empty
                legend(ax, legend_str, 'Location', 'bestoutside', 'FontSize', fS);
            end
    end

end
