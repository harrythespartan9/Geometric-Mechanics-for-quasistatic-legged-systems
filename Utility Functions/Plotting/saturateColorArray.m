function colArrOut = saturateColorArray(colArrIn, intenFact)
%DESATURATESINGLEPIXEL Saturates a color based on its luminance.
%   The function takes a color array (1x3 or 3x1) and a saturation factor 
%   as input. It calculates the luminance of the pixel and blends the color 
%   away from its grayscale equivalent based on the saturation factor.
%
%   Inputs:
%       colArrIn           - Input color (1x3 or 3x1)
%                            (range between 0 and 1 for each RGB value)
%       intenFact          - Intensity factor between 0 and 1. 
%                            (0 = original color, 1 = white)
%
%   Output:
%       colArrOut          - Output color (1x3 or 3x1)

    % Check if intensity factor is within the valid range
    if intenFact < 0 || intenFact > 1
        error('Intensity factor must be between 0 and 1.');
    end

    % Check if the input pixel is a valid RGB pixel (1x3 or 3x1)
    if ~( all(size(colArrIn) == [1, 3]) || all(size(colArrIn) == [3, 1]) )
        error('Input color array must be a 1x3 or 3x1 array.');
    elseif all(size(colArrIn) == [3, 1])
        % if not a row-vector, convert it to one
        colArrIn = colArrIn';
        % set the transpose flag
        transpFlag = true;
    else
        % unset the transpose flag
        transpFlag = false;
    end

    % Create the average grayscale color array
    wColArr = ones(1, 3);
    
    % Blend the original color towards white based on the intensity factor
    % ... ensure the color have no values below 0 and above 1
    % ... convert the output color to the input color size if necessary
    colArrOut = (1 - intenFact)*colArrIn + intenFact*wColArr;
    colArrOut(colArrOut > 1) = 1; colArrOut(colArrOut < 0) = 0;
    if transpFlag
        colArrOut = colArrOut';
    end

end

