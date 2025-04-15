function plot2DnormDotOfLocalConnWOGaits...
    (a, aLim, aTic, A1dotA2, aString, AdotString)
%PLOT2DNORMDOTOFLOCALCONNWOGAITS Plot the normalized dopt product of local 
%connections of 2D slices of the shape space

    % load the plotting parameters
    load('Data\Sinusoidal Gaits\clariConnVecF.mat', 'lW_Vector');
    load('Data\Plotting\divergingColorMap', 'divColMap');

    % change the value below closer or farther away from 1 to indicate the
    % places where the vectors are aligned and the span of the two vectors
    % is one-dimensional
    thresh_value = 0.99;

    % plot the dot product of the local connection vector field
    % ... plot it in 2 cols for each contact state
    fS = 12; cfLvl = size(a{1}, 1);
    % ... % lab setup: 3200 -300, % just laptop: 0 0
    f = figure('units', 'pixels', ...
        'position', [3200 -300 350 350]); 
    set(f,'Visible','on');
    tl = tiledlayout(f, 1, 1, ...
        "TileSpacing", "tight", "Padding", "tight");
    % %
    ax = nexttile(tl); % A1 dot with A2 (with highlighted ~+-1 contours)
    contourf(ax, rad2deg(a{1}), rad2deg(a{2}), A1dotA2, cfLvl, ...
        "LineStyle", 'none');
    hold(ax, 'on');
    contour(ax, rad2deg(a{1}), rad2deg(a{2}), A1dotA2, ...
        thresh_value*ones(1, 2), 'k--','LineWidth', lW_Vector); % parll
    contour(ax, rad2deg(a{1}), rad2deg(a{2}), A1dotA2, ...
        thresh_value*ones(1, 2), 'k--','LineWidth', lW_Vector); % antiparll
    axis(ax, "equal"); ax.FontSize = fS;
    colormap(ax, divColMap); clim(ax, [-1, 1]);
    xlim(ax, aLim{1}); ylim(ax, aLim{2});
    xticks(ax, aTic{1}); yticks(ax, aTic{2});
    xlabel(ax, aString{1}, FontSize=fS);
    ylabel(ax, aString{2}, FontSize=fS);
    title(ax, AdotString, FontSize=fS);

    % % 
    % add a colorbar on the right side of the tiledlayout
    cb = colorbar(ax, 'FontSize', fS, 'TickLabelInterpreter', 'latex');
    cb.Layout.Tile = 'east';

end

