function r_out = apprxsineswing(traj)
%APPRXSINESWING approximates the input experimental swing timeseries data as a pure sinusoid.
%   This function approximates the input function as pure sinusoid by identifying postive and negative peaks. Also, this function assumes that the 
    
    % Unpack
    % t = traj.exp.t;
    r = traj.exp.r;
    Fs = numel(traj.exp.t)/(traj.exp.t(end) - traj.exp.t(1));         % sampling frequency
    T  = 1/traj.exp.f;                                                % time period

    % Iterate and compute
    r_out = cell(size(r));
    for i = 1:numel(r)
        [pos_p, t_p] = findpeaks(r{i}, Fs, 'MinPeakDistance', 0.96*T);  % pos peaks-- 4% thresh-held peak location should match stride time period
        [pos_m, t_m] = findpeaks(-r{i}, Fs, 'MinPeakDistance', 0.96*T); pos_m = -pos_m; % neg peaks
        yamp = (mean(pos_p) - mean(pos_m))/2;
        y_dc = (mean(pos_p) + mean(pos_m))/2;
        if t_p(1) < t_m(1)
            tau = t_p(1); mul = 1;
        else
            tau = t_m(1); mul = -1; % cosine function definition-- mul*yamp*cos(2*pi*f*(t - tau)) + y_dc {OUTPUT IN THE SAME ORDER}
        end
        r_out{i} = [mul, yamp, traj.exp.f, tau, y_dc]';
    end

end

