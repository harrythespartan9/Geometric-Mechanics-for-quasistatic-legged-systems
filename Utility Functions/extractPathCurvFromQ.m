function [netCurve, CurveTraj] = extractPathCurvFromQ(Qdot)
%EXTRACTPATHCURVFROMQ This function extracts the path curvature given local configuration velocity
%   We extract the local SE(2) velocity trajectory of the system (1 to 3) and compute the path curvature trajectory and net curvature trajectory from that.
    CurveTraj = Qdot(3, :)./sqrt(Qdot(1, :).^2 + Qdot(2, :).^2);
    netCurve = mean(CurveTraj);
end

