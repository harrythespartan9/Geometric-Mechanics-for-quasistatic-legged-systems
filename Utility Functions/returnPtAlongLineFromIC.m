function pReq = returnPtAlongLineFromIC(pIC, unitPhasor, a)
%RETURNPTALONGLINEFROMIC return a point along a line
%   Given the line IC, 'pIC' and a phasor multiplier, 'a', find a point 
%   along the line, or equivalently, given a list of multiplier values, 
%   find all the points they correspond to along a line.

    % return the point as a column vector
    switch numel(a)
        case 1
            pReq = pIC(:)+ a*unitPhasor(:);
        otherwise
            numPts = size(a, 2);
            if numPts < 2
                a = a'; numPts = size(a, 2); % condition
            end
            pReq = repmat(pIC(:), [1, numPts]) + ...
                repmat(a, [2, 1]).*...
                repmat(unitPhasor(:), [1, numPts]); % vectorially compute 
    end

end

