function fullSurf = fullStratSurfcompute(stancei, stancej, fullStratij, colorRelated, topSurfLvl)
%FULLSTRATSURFCOMPUTE compute the full stratified panel as a surface between the two stance phase paths
%   Given two stance phase paths in their reduced shape space {B_{13}, S_{13}} and the full stratified panel at each corresponding point, this function computes
%   the same panel as a surface defined by an `X', `Y', `Z', and `C' matrices. The `C' matrix defines the color of the surface. These four quantities are
%   computed for each SE(2) direction and provided as the output through a cell array.
    
    % number of discretizations along the path
    dnum = size(stancei, 2); % this will be used to create a square matrix representation of the surface

    % unpack colors data
    colLim = [];
    colLim.XY = colorRelated{1}{1}; colLim.Th = colorRelated{1}{2};
    colMapNow = colorRelated{2};
    
    % initialize the output container
    fullSurf = [];
    fullSurf.X = nan(dnum, dnum); fullSurf.Y = fullSurf.X; 
    fullSurf.C = cell(1, 3);

    % interpolate the paths at each infinitesimal gait cycle (each columnwise point)
    for idx = 1:dnum
        temp_x = interp1([1, dnum], [stancei(1, idx), stancej(1, end-idx+1)], 1:dnum); temp_x = temp_x(:);
        temp_y = interp1([1, dnum], [stancei(2, idx), stancej(2, end-idx+1)], 1:dnum); temp_y = temp_y(:); % compute
        fullSurf.X(:, idx) = temp_x;
        fullSurf.Y(:, idx) = temp_y; % assign
    end

    % compute the z-vals along the infinitesimal gait cycle
    fullSurf.Z = repmat( linspace(0, topSurfLvl, dnum)',  [1 dnum] );

    % compute the color matrix (Size = dnum X dnum X 3)
    for idxPos = 1:numel(fullSurf.C)
        fullPanelNow = fullStratij(idxPos, :)'; % the current panel
        switch idxPos
            case 3 % if theta panel
                colQ = 100*( fullPanelNow - colLim.Th(1) )/diff(colLim.Th); % where the current panel slice is in the 1D range
            otherwise % if NOT theta panel
                colQ = 100*( fullPanelNow - colLim.XY(1) )/diff(colLim.XY);
        end
        fullPanelColors = interp1(   linspace(0, 100, size(colMapNow, 1))', colMapNow, colQ, "spline"   ); 
        fullPanelColors(fullPanelColors > 1) = 1; % dnum X 3 color array (vals cant exceed 1!!)
        fullPanelColors = permute( reshape(fullPanelColors, [dnum, 1, 3]), [2 1 3] ); % 1 X dnum x 3 color array (required color array horz slice)
        fullSurf.C{idxPos} = repmat(fullPanelColors, [dnum, 1, 1]); % in the surf function (Size = dnum X dnum X 3) format
    end

end

