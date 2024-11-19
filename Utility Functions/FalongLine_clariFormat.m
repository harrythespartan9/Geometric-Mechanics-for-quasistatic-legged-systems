% compute the F-value at a point along a line from 'refPt' constructed
% using a 'unitPhasor' with a multiplier 'aValue'.
% ... Primarily used by the mobility framework for body morphing systems 
% ... and in the "Path2_Mobility" class for computing perpendicular 
% ... coordinates.
function fVal = FalongLine_clariFormat(aa, ll, alphab, F_fxn, refPt, unitPhasor, aValue)
    pt = returnPtAlongLineFromIC(refPt, unitPhasor, aValue)';
    fVal = F_fxn(aa, ll, alphab, pt(1), pt(2));
end

