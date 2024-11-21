% compute the F-value at a point along a line from 'refPt' constructed
% using a 'unitPhasor' with a multiplier 'aValue'.
% ... Primarily used by the mobility framework for body morphing systems 
% ... and in the "Path2_Mobility" class for computing perpendicular 
% ... coordinates.
function fVal = FalongLine_arbFormat(aa_bX, aa_bY, aa, ll, gamma, ...
                                        F_fxn, refPt, unitPhasor, aValue)
    pt = returnPtAlongLineFromIC(refPt, unitPhasor, aValue)';
    fVal = F_fxn(aa_bX, aa_bY, aa, ll, gamma, pt(1), pt(2));
end

