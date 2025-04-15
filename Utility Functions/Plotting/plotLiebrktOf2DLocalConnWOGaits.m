function plotLiebrktOf2DLocalConnWOGaits...
    (a, aLim, aTic, Alim, BL, lieBrktOfA1A2, aString, lieBrktOfAstr)
%PLOTLIEBRKTOF2DLOCALCONNWOGAITS Plot the lie-bracket of two connection
%vector fields along the actuated directions of the shape space.

    % load the plotting parameters
    load('Data\Plotting\divergingColorMap', 'divColMap');

    % plot the dot product of the local connection vector field
    % ... plot it in 2 cols for each contact state
    fS = 12; cfLvl = size(a{1}, 1);
    % ... % lab setup: 3200 -300, % just laptop: 0 0
    f = figure('units', 'pixels', ...
        'position', [3200 -300 350 600]); 
    set(f,'Visible','on');
    tl = tiledlayout(f, 2, 1, ...
        "TileSpacing", "tight", "Padding", "tight");
    % %
    ax = nexttile(tl); % x-component of [A1, A2]
    contourf(ax, rad2deg(a{1}), rad2deg(a{2}), lieBrktOfA1A2{1}/BL, ...
        cfLvl, "LineStyle", 'none');
    axis(ax, "equal"); ax.FontSize = fS;
    colormap(ax, divColMap); clim(ax, Alim/BL);
    xlim(ax, aLim{1}); ylim(ax, aLim{2});
    xticks(ax, aTic{1}); yticks(ax, aTic{2});
    xlabel(ax, aString{1}, FontSize=fS);
    ylabel(ax, aString{2}, FontSize=fS);
    title(ax, ['$', lieBrktOfAstr, '^{x}$'], FontSize=fS);

    ax = nexttile(tl); % x-component of [A1, A2]
    contourf(ax, rad2deg(a{1}), rad2deg(a{2}), lieBrktOfA1A2{2}/BL, ...
        cfLvl, "LineStyle", 'none');
    axis(ax, "equal"); ax.FontSize = fS;
    colormap(ax, divColMap); clim(ax, Alim/BL);
    xlim(ax, aLim{1}); ylim(ax, aLim{2});
    xticks(ax, aTic{1}); yticks(ax, aTic{2});
    xlabel(ax, aString{1}, FontSize=fS);
    ylabel(ax, aString{2}, FontSize=fS);
    title(ax, ['$', lieBrktOfAstr, '^{y}$'], FontSize=fS);

    % % 
    % add a colorbar on the right side of the tiledlayout
    cb = colorbar(ax, 'FontSize', fS, 'TickLabelInterpreter', 'latex');
    cb.Layout.Tile = 'east';

end