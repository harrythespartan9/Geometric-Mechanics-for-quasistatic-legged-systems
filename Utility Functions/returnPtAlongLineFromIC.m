function pReq = returnPtAlongLineFromIC(pIC, unitPhasor, a)
%RETURNPTALONGLINEFROMIC return a point along a line
%   Given the line IC and a phasor multiplier, find a point along the line

    % return the point as a column vector
    pReq = pIC(:) + a*unitPhasor;

end

