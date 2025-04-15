function plot2DLocalConnWOGaits...
    (a, aLim, aTic, BL, A1, A2, aString, AString, iQ)
%PLOT2DLOCALCONNWOGAITS Plot the local connections of 2D slices of the
%shape space

    % load the plotting parameters
    load('Data\Sinusoidal Gaits\clariConnVecF.mat', 'lW_Vector');

    % if iQ or the subselection of points to plot the vector field at is
    % not provided, default it to 10% of the values
    if nargin < 8
        iQ = round(   linspace(  1, size(a{1}, 1), size(a{1}, 1)/10  )   );
    end

    % plot the local connection vector fields
    % ... plot it in 2 cols for each contact state
    fS = 12; 
    % ... % lab setup: 3200 -300, % just laptop: 0 0
    f = figure('units', 'pixels', ...
        'position', [3200 -300 300 900]);
    set(f,'Visible','on');
    tl = tiledlayout(f, 3, 1, ...
        "TileSpacing", "tight", "Padding", "tight");
    % %
    ax = nexttile(tl); % Ax in uI and uII directions
    quiver(ax, ...
        rad2deg(a{1}(iQ, iQ)), rad2deg(a{2}(iQ, iQ)), ...
        A1{1}(iQ, iQ)/BL, A2{1}(iQ, iQ)/BL, ...
        'k-', "LineWidth", lW_Vector);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, aLim{1}); ylim(ax, aLim{2});
    xticks(ax, aTic{1}); yticks(ax, aTic{2});
    xlabel(ax, aString{1}, FontSize=fS);
    ylabel(ax, aString{2}, FontSize=fS);
    title(ax, AString{1}, FontSize=fS);
    % %
    ax = nexttile(tl); % Ay
    quiver(ax, ...
        rad2deg(a{1}(iQ, iQ)), rad2deg(a{2}(iQ, iQ)), ...
        A1{2}(iQ, iQ)/BL, A2{2}(iQ, iQ)/BL, ...
        'k-', "LineWidth", lW_Vector);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, aLim{1}); ylim(ax, aLim{2});
    xticks(ax, aTic{1}); yticks(ax, aTic{2});
    xlabel(ax, aString{1}, FontSize=fS);
    ylabel(ax, aString{2}, FontSize=fS);
    title(ax, AString{2}, FontSize=fS);
    % %
    ax = nexttile(tl); % Atheta
    quiver(ax, ...
        rad2deg(a{1}(iQ, iQ)), rad2deg(a{2}(iQ, iQ)), ...
        A1{3}(iQ, iQ)/BL, A2{3}(iQ, iQ)/BL, ...
        'k-', "LineWidth", lW_Vector);
    axis(ax, "equal"); ax.FontSize = fS;
    xlim(ax, aLim{1}); ylim(ax, aLim{2});
    xticks(ax, aTic{1}); yticks(ax, aTic{2});
    xlabel(ax, aString{1}, FontSize=fS);
    ylabel(ax, aString{2}, FontSize=fS);
    title(ax, AString{3}, FontSize=fS);

end

