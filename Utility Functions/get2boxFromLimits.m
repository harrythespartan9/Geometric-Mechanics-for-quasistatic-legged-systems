function box = get2boxFromLimits(twoLimits)
%GET2BOXFROMLIMITS given the limits of a 2D space with flat or fixed
%limits, create a bounding box.
    xLimits = twoLimits(1, :);
    yLimits = twoLimits(2, :);
    BL = [xLimits(1), yLimits(1)]';
    BR = [xLimits(2), yLimits(1)]';
    TR = [xLimits(2), yLimits(2)]';
    TL = [xLimits(1), yLimits(2)]';
    XY = [BL, BR, TR, TL, BL];
    UV = diff(XY, 1, 2); 
    XY = XY(:, 1:end-1);
    box.X = XY(1, :);
    box.Y = XY(2, :);
    box.U = UV(1, :);
    box.V = UV(2, :);
end

