function out = rijLimits(in, degPad)
%RIJLIMITS compute the symmetric limits for plotting the input, shape-space slice trajectories with a padding of "degPad" degrees

    out = nan(1, numel(in)); % initialize
    for i = 1:numel(in)
        out(i) = findswinglimits(in{i}) + deg2rad( degPad ); % compute limit and add padding
    end
    out = max(out); % find the maximum limit and stick with that

end

