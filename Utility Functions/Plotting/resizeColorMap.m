function colMapOut = resizeColorMap(colMapIn, numNeeded)
%RESIZECOLORMAP Adjusts the size of a colormap to match a specified range.
%   This function modifies the colormap of a figure or axes to fit a given
%   range of data values. It can be used to ensure that the color mapping
%   corresponds accurately to the desired data range, improving the
%   visualization of plots.
%
%   Inputs:
%       colMapIn   - A matrix representing the input colormap. Each row
%                    corresponds to an RGB triplet (values between 0 and 1).
%       numNeeded  - The desired number of colors in the output colormap.
%
%   Outputs:
%       colMapOut  - A matrix representing the resized colormap. It will
%                    have `numNeeded` rows, each corresponding to an RGB
%                    triplet.
%
%   Example:
%       % Example usage of the function
%       colMapIn = jet(10); % Original colormap with 10 colors
%       numNeeded = 20;     % Desired number of colors
%       colMapOut = resizeColorMap(colMapIn, numNeeded);
%       colormap(colMapOut); % Apply the resized colormap to a plot
%
%   Notes:
%       - Specifically useful for resizing custom colormaps.
%       - Ensure that the input colormap (`colMapIn`) is a valid matrix
%         where each row is an RGB triplet with values in the range [0, 1].
%       - The function uses linear interpolation to resize the colormap.
%       - This function is particularly useful for visualizing data with
%         non-uniform ranges or when a specific number of colors is needed.
%
%   See also: colormap, interp1, linspace
colMapOut = interpColorAndCondition(...
                (1:size(colMapIn, 1))', ...
                colMapIn, ...
                linspace(1, size(colMapIn, 1), numNeeded)' ...
                             );
end

