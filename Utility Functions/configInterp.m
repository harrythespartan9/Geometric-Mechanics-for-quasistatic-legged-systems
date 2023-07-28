function Qt_out = configInterp(tq, Qt)
%CONFIGINTERP interpolate configuration trajectory in SE(2)
%   Given a cell array with the first entry time, then the next 3 entries with the position space trajectory, and the rest with shape space trajectories, this
%   function interpolates each for a given query time array. It also zeroes the initial conditions of the SE(2) trajectories after interpolating.

    if ~iscell(Qt)
        error('ERROR! The configuration trajectory should be a cell array.');
    else
        if numel(Qt) < 4
            error('ERROR! The configuration trajectory have a time array and atleast 3 SE(2) arrays.');
        else
            t = Qt{1}; % time array
            Qt_out = cell(size(Qt)); % initialize output
            for i = 2:numel(Qt)
                Qt_out{i} = interp1(t, Qt{i}, tq, 'pchip'); % interpolate
                if i <= 4
                    Qt_out{i} = Qt_out{i} - Qt_out{i}(1); % zero out initial condition if SE(2) direction
                end
            end
            Qt_out{1} = tq - tq(1); % assign the time array as the first entry
        end
    end

end

